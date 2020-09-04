"""Implementation of ControlInterface for Sawyer. Class definition and execution.
Runs in a separate python2.7 environment on Sawyer Workstation
"""
from __future__ import division
import redis
import sys, time, copy
from threading import Thread, Event
import tf

try:
    import moveit_commander
    from moveit_msgs.msg import (
        Grasp,
        GripperTranslation,
        DisplayTrajectory
    )
except ImportError:
    print('Problems with moveit!')

import rospy
import rospkg
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand

try:

    import intera_interface as iif
    from intera_interface import CHECK_VERSION
    from intera_core_msgs.srv import (
        SolvePositionIK,
        SolvePositionIKRequest,
        SolvePositionFK,
        SolvePositionFKRequest
    )
    from intera_core_msgs.msg import JointCommand, InteractionControlCommand
    from intera_motion_interface import (
        MotionTrajectory,
        MotionWaypoint,
        MotionWaypointOptions,
        InteractionOptions
    )
    from intera_motion_msgs.msg import TrajectoryOptions
except ImportError:
    print('intera_interface not imported, did you remember to run cd ~/ros_ws;./intera.sh?')

import numpy as np
import logging
logging.basicConfig(level=logging.DEBUG)
import pybullet as pb
import time
# import PyKDL as KDL
# from kdl_parser_py.urdf import treeFromUrdfModel
# from urdf_parser_py.urdf import URDF

from pyquaternion import Quaternion as pyQuaternion

from moveit_msgs.msg import (
    RobotState,
    MoveItErrorCodes,
    TrajectoryConstraints,
    Constraints,
    WorkspaceParameters,
    JointConstraint,
    MotionPlanRequest,
    MotionPlanResponse,
    MoveGroupAction,
    MoveGroupGoal,
    MoveGroupResult,
    MoveGroupFeedback
)
from sensor_msgs.msg import JointState
from moveit_msgs.srv import (
    GetCartesianPath,
    GetCartesianPathRequest,
    GetCartesianPathResponse,
    GetMotionPlan
)
import actionlib
#from safenet import SafenetMonitor
from perls2.robots.real_robot_interface import RealRobotInterface
from perls2.robots.robot_interface import RobotInterface
from perls2.controllers.ee_imp import EEImpController
from perls2.controllers.ee_posture import EEPostureController
from perls2.controllers.joint_imp import JointImpController
from perls2.controllers.interpolator.linear_interpolator import LinearInterpolator
from perls2.controllers.interpolator.linear_ori_interpolator import LinearOriInterpolator
from perls2.controllers.robot_model.model import Model

from perls2.utils.yaml_config import YamlConfig
from scipy.spatial.transform import Rotation as R
import json
import socket

from perls2.ros_interfaces.redis_interface import RobotRedisInterface as RobotRedis
from perls2.ros_interfaces.redis_keys import *

LOOP_LATENCY = 0.000
LOOP_TIME = (1.0/500.0) - LOOP_LATENCY
class SawyerCtrlInterface(RobotInterface):
    """ Class definition for Sawyer Control Interface. 

    Interface creates and monitors RedisServer for new commands from RobotInterface. 
    Commands are interpreted and converted into set of joint torques sent to Sawyer via ROS.


    """
    def __init__(self, 
                 config='cfg/sawyer_ctrl_config.yaml', 
                 controlType='EEImpedance',
                 use_safenet=False,
                 use_moveit=False,
                 node_name='sawyer_interface'):
        """
        Initialize Sawyer Robot for control

        Args:
            use_safenet: bool
                whether to use the Safenet to constrain ee positions
                and velocities
            use_moveit: bool
                whether to use moveIt for motion planning
            node_name: str
                name of the node for ROS

        Attributes:
            ee_position
            ee_orientation


        """
        self.log_start_times = []
        self.log_end_times = []
        self.cmd_start_times = []
        self.cmd_end_times = []
        # Connect to redis client
        # use default port 6379 at local host.
        # TODO:  match these up in cfg file later.
        self.config = YamlConfig(config)
        # redis_kwargs = self.config['redis']
        # if 'localhost' not in redis_kwargs['host']:
        #     redis_kwargs['host'] = socket.gethostbyname(self.config['redis']['host'])

        # self.redisClient = redis.Redis(**redis_kwargs)

        # 
        self.redisClient = RobotRedis(**self.config['redis'])
        self.redisClient.flushall()   
        self.current_state = "SETUP"
        ## Timing
        self.startTime = time.time()
        self.endTime = time.time()
        self.action_set = False
        self.model = Model()
        
        if config is not None:
            world_name = self.config['world']['type']
            controller_config = self.config['controller'][world_name]
            if self.config['controller']['interpolator']['type'] == 'linear':
                interp_kwargs = {'max_dx': 0.5, 
                                 'ndim': 3, 
                                  'controller_freq': 500, 
                                  'policy_freq' : 20, 
                                  'ramp_ratio' :  0.2 }
                self.interpolator_pos = LinearInterpolator(**interp_kwargs)
                self.interpolator_ori = None#LinearOriInterpolator(**interp_kwargs)
                rospy.loginfo("Linear interpolator created with params {}".format(interp_kwargs))
            else:
                self.interpolator_pos = None
                self.interpolator_ori = None
        self.interpolator_goal_set = False

        start = time.time()

        rospy.loginfo('Initializing Sawyer robot interface...')

        try:
                self._head = iif.Head()
                self._display = iif.HeadDisplay()
                self._lights = iif.Lights()
                self._has_head = True
        except:
            rospy.logerr('No head found, head functionality disabled')
            self._has_head = False

        self._limb = iif.Limb(limb="right", synchronous_pub=False)
        self._joint_names = self._limb.joint_names()

        self._use_safenet = use_safenet
        if self._use_safenet:
            self.safenet = SafenetMonitor('right_hand')

        self.transform_matrix = None

        self.cmd = []

        try:
            self._gripper = iif.Gripper()
            self._has_gripper = True
        except:
            rospy.logerr('No gripper found, gripper control disabled')
            self._has_gripper = False

        self._robot_enable = iif.RobotEnable(True)

        self._params = iif.RobotParams()

        self.blocking = False
        # Initialize motion planning part
        self._use_moveit = use_moveit
        if use_moveit:
            moveit_commander.roscpp_initialize(sys.argv)
            self._moveit_robot = moveit_commander.RobotCommander()
            self._moveit_scene = moveit_commander.PlanningSceneInterface()
            self._moveit_group = moveit_commander.MoveGroupCommander("right_arm")
            self._moveit_group.allow_replanning(True)
            self._moveit_group.set_pose_reference_frame('world')
            # Allow some leeway in position(meters) and orientation (radians)
            self._moveit_group.set_goal_position_tolerance(0.005)
            self._moveit_group.set_goal_orientation_tolerance(0.05)
            print_msg = "The robot groups are: {0}".format( self._moveit_robot.get_group_names())
            rospy.loginfo(print_msg)
            print_msg = "Any planning will be performed relative to the {0} reference frame".format(
                self._moveit_group.get_planning_frame())
            rospy.loginfo(print_msg)
            print_msg = "The command group is '{0}'".format( self._moveit_group.get_name())
            rospy.loginfo(print_msg)
            print_msg = "The {0} group has active joints: {1}".format(self._moveit_group.get_name(),
                                                            self._moveit_group.get_active_joints())
            rospy.loginfo(print_msg)
            print_msg = "Its end effector link is: {0}".format(
                self._moveit_group.get_end_effector_link())
            rospy.loginfo(print_msg)
            self._cartesian_path_service = rospy.ServiceProxy('compute_cartesian_path',
                                                              GetCartesianPath)
            self._js_path_service = rospy.ServiceProxy('plan_kinematic_path', GetMotionPlan)
            self._js_path_action = actionlib.SimpleActionClient('move_group', MoveGroupAction)
            self._js_path_action.wait_for_server()


        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # set up internal pybullet simulation for jacobian and mass matrix calcs.
        self._clid = pb.connect(pb.DIRECT)
        pb.resetSimulation(physicsClientId=self._clid)

        # TODO: make this not hard coded
        sawyer_urdf_path = self.config['sawyer']['arm']['path']

        # self._pb_sawyer = pb.loadURDF(sawyer_urdf_path,
        #                               (0, 0, 0),
        #                               useFixedBase=True, 
        #                               physicsClientId=self._clid)
        self._pb_sawyer = pb.loadURDF(
            fileName=sawyer_urdf_path,
            basePosition=self.config['sawyer']['arm']['pose'],
            baseOrientation=pb.getQuaternionFromEuler(
                                    self.config['sawyer']['arm']['orn']),
            globalScaling=1.0,
            useFixedBase=self.config['sawyer']['arm']['is_static'],
            flags=pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | pb.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self._clid)
        # For pybullet dof
        self._motor_joint_positions = self.get_motor_joint_positions()
        try:
            ns = "ExternalTools/right/PositionKinematicsNode/IKService"
            self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
            rospy.wait_for_service(ns, 5.0)
            self._ik_service = True
        except:
            rospy.logerr('IKService from Intera timed out')
            self._ik_service = False

#        self._interaction_options = InteractionOptions()
#        if len(self._interaction_options._data.D_impedance) == 0:
#            self._interaction_options._data.D_impedance = [8, 8, 8, 2, 2, 2]

#        self._interaction_options_pub = \
#            rospy.Publisher('/robot/limb/right/interaction_control_command',
#                            InteractionControlCommand, queue_size = 1)
        rospy.loginfo('Sawyer initialization finished after {} seconds'.format(time.time() - start))

        # joint_state_topic = 'robot/joint_states'
        # _joint_state_sub = rospy.Subscriber(
        #     joint_state_topic,
        #     JointState,
        #     self._on_joint_states,
        #     queue_size=1,
        #     tcp_nodelay=True)

        # Set desired pose to initial
        curr_ee_pose = self.ee_pose
        self.neutral_joint_position = [0,-1.18,0.00,2.18,0.00,0.57,3.3161]

        self.prev_cmd = np.asarray(self.neutral_joint_position)
        self.current_cmd = self.prev_cmd

        self.reset_to_neutral()
        self.cmd_dict = {
            'move_ee_delta' : self.move_ee_delta,
            'set_ee_pose' : self.set_ee_pose,
            'set_joint_positions' : self.set_joint_positions
            }

        self.default_control_type = self.config['controller']['selected_type']
        self.default_params = self.config['controller']['Real'][self.default_control_type]
        self.redisClient.set(CONTROLLER_CONTROL_TYPE_KEY, self.default_control_type)
        self.redisClient.set(CONTROLLER_CONTROL_PARAMS_KEY, json.dumps(self.default_params))

        # Set initial values for redis db

        # self.redisClient.set(ROBOT_CMD_TYPE_KEY, 'set_joint_positions')

        # self.redisClient.set(CONTROLLER_CONTROL_TYPE_KEY, 'JointImpedance')
        # # self.redisClient.set('robot::qd', str(self.neutral_joint_position))
        # self.redisClient.set(CONTROLLER_GOAL_KEY, 
        #     json.dumps({'delta': None, 'set_qpos': self.neutral_joint_position}))

        # default_params = self.config['controller']['Real']['JointImpedance']
        # self.redisClient.set(CONTROLLER_CONTROL_PARAMS_KEY, json.dumps(default_params))
        

        # Set initial redis keys
        self._linear_jacobian = None
        self._angular_jacobian = None
        self._jacobian = None
        self._calc_mass_matrix()
        self._calc_jacobian() 
            
        self.update_redis()


        self.update_model()

        self.controlType = self.get_control_type()
        self.control_dict = self.get_controller_params()

        self.controller = self.make_controller_from_redis(self.controlType, self.control_dict)


       # self.redisClient.set('robot::desired_ee_pose', str(curr_ee_pose))
        # self.redisClient.set('robot::env_connected', 'False')
        # self.redisClient.set('run_controller', 'False')


        # self.redisClient.set('robot::ee_position', str(self.ee_position))

        # self.redisClient.set('robot::ee_orientation', str(self.ee_orientation))

        # #Set desired pose to current pose initially
        # self.redisClient.set('robot::desired_ee_pose', str(self.ee_pose))
        # self.redisClient.set('robot::tau_desired', str(self.tau))

        self.redisClient.set(ROBOT_CMD_TSTAMP_KEY, time.time())
        self.last_cmd_tstamp = self.get_cmd_tstamp()
        rospy.logdebug('Control Interface initialized')

        # TIMING TEST ONLY
        self.timelog_start = open('cmd_tstamp.txt', 'w')
        self.timelog_end = open('cmd_set_time.txt', 'w')

        self.controller_times = []
        self.loop_times = []
        self.cmd_end_time = []

        print("CURRENT EE_POSE {}".format(self.ee_pose))



    def make_controller_from_redis(self, control_type, controller_dict):
        print("Making controller {} with params: {}".format(control_type, controller_dict))
        self.controlType = control_type
        if control_type == "EEImpedance":
            return EEImpController(self.model, 
                     interpolator_pos=self.interpolator_pos, 
                     interpolator_ori=self.interpolator_ori, **controller_dict)
        elif control_type == "EEPosture":
            return EEPostureController(self.model, 
                     interpolator_pos=self.interpolator_pos, 
                     interpolator_ori=self.interpolator_ori, **controller_dict)
        elif control_type =="JointImpedance":
            return JointImpController(self.model, 
                    **controller_dict)



    def get_controller_params(self):
        return self.redisClient.get(CONTROLLER_CONTROL_PARAMS_KEY)

    def get_control_type(self):
        return self.redisClient.get(CONTROLLER_CONTROL_TYPE_KEY)

    def reset_to_neutral(self):
        """Blocking call for resetting the arm to neutral position
        """
        self.redisClient.set(ROBOT_RESET_COMPL_KEY, 'False')
        rospy.loginfo("Resetting to neutral")
        self.goto_q(self.neutral_joint_position,  max_joint_speed_ratio=0.2)

        # set to joint position control to avoid repeating reset.
        # self.redisClient.set('robot::cmd_type', 'joint_position')
        # self.redisClient.set('robot::qd', str(self.neutral_joint_position))
        self.redisClient.set(ROBOT_RESET_COMPL_KEY, 'True')
        rospy.loginfo("reset complete")

    @property
    def version(self):
        """
        List current versions of robot SDK, gripper, and robot
        :return: dictionary of version strings
        """
        return dict(SDKVersion=iif.settings.SDK_VERSION,
                    SDK2Gripper=iif.settings.VERSIONS_SDK2GRIPPER,
                    SDK2Robot=iif.settings.VERSIONS_SDK2ROBOT)

    @property
    def name(self):
        """
        Get the name of the robot
        :return: 'Sawyer'
        """
        return self._params.get_robot_name()

    @property
    def ee_position(self):
        """
        Get the position of the end-effector.
        :return: a list of floats for the position [x, y, z]
        """
        return list(self._limb.endpoint_pose()['position'])

    @property
    def ee_orientation(self):
        """
        Get the orientation of the end-effector.
        :return: a list of floats for the orientation quaternion [qx,
        qy, qz, qw]
        """
        return list(self._limb.endpoint_pose()['orientation'])

    @property
    def ee_pose(self):
        """
        Get the pose of the end effector.
        :return: a list of floats for the position and orientation [x,
        y, z, qx, qy, qz, qw]
        """
        return list(self.ee_position + self.ee_orientation)

    @property
    def ee_v(self):
        """
        Get the current linear velocity of end effector in the eef
        frame.
        :return: a list of floats for the linear velocity m/s [vx, vy,
        vz]
        """
        return list(self._limb.endpoint_velocity()['linear'])

    @property
    def ee_omega(self):
        """
        Get the current twist velocity of end effector in the eef frame.
        :return: a list of floats for the twist velocity [vx, vy, vz,
        wx, wy, wz]
        """
        return list(self._limb.endpoint_velocity()['angular'])

    @property
    def ee_twist(self):
        """
        Get the current twist velocity of end effector in the eef frame.
        :return: a list of floats for the twist velocity [vx, vy, vz,
        wx, wy, wz]
        """
        return list(self.ee_v + self.ee_omega)

    @property
    def ee_force(self):
        """
        Get the force (fx, fy, fz) on end effector
        :return: a list of floats for force N [fx, fy, fz]
        """
        return list(self._limb.endpoint_effort()['force'])

    @property
    def ee_torque(self):
        """
        Get the torque (mx, my, mz) on end effector
        :return: a list of floats for torque N*m [mx, my, mz]
        """
        return list(self._limb.endpoint_effort()['torque'])

    @property
    def ee_wrench(self):
        """
        Get the wrench on end effector
        :return: a list of floats for the wrench [fx, fy, fz, mx, my,
        mz]
        """
        return list(self.ee_force + self.ee_torque)

    @property
    def q(self):
        """
        Get the joint configuration of the robot arm.
        :return: a list of joint configurations in radian ordered by
        indices from small to large.
        Typically the order goes from base to end effector.
        """
        return self._limb.joint_ordered_angles()

    @property
    def dq(self):
        """
        Get the joint velocities of the robot arm.
        :return: a list of joint velocities in radian/s ordered by
        indices from small to large.
        Typically the order goes from base to end effector.
        """
        dq = self._limb.joint_velocities()
        return [dq[n] for n in self._joint_names]

    @property
    def tau(self):
        """
        Get the joint torques of the robot arm.
        :return: a list of joint torques in N*m ordered by indices from
        small to large.
        Typically the order goes from base to end effector.
        """
        tau = self._limb.joint_efforts()
        return [tau[n] for n in self._joint_names]

    @property 
    def num_joints(self):
        """ Number of joints according to pybullet.
        """
        return pb.getNumJoints(self._pb_sawyer, physicsClientId=self._clid)
    def get_motor_joint_indices(self):
        """ Go through urdf and get joint indices of "free" joints.
        """
        motor_joint_indices = []

        for joint_index in range(self.num_joints):

            info = pb.getJointInfo(
                bodyUniqueId=self._pb_sawyer,
                jointIndex=joint_index,
                physicsClientId=self._clid)

            if (info[2] != pb.JOINT_FIXED):
                motor_joint_indices.append(joint_index)

        return motor_joint_indices

    @property
    def num_free_joints(self):
        return len(self.get_motor_joint_indices())

    @property
    def motor_joint_positions(self):
        return self._motor_joint_positions

    def get_motor_joint_positions(self):
        """ returns the motor joint positions for "each DoF" according to pb.

        Note: fixed joints have 0 degrees of freedoms.
        """
        joint_states = pb.getJointStates(
        self._pb_sawyer, range(pb.getNumJoints(self._pb_sawyer,
                                                  physicsClientId=self._clid)),
        physicsClientId=self._clid)
        # Joint info specifies type of joint ("fixed" or not)
        joint_infos = [pb.getJointInfo(self._pb_sawyer, i,  physicsClientId=self._clid) for i in range(pb.getNumJoints(self._pb_sawyer, 
                                                                                                                                     physicsClientId=self._clid))]
        # Only get joint states of free joints
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[2] != pb.JOINT_FIXED]

        joint_positions = [state[0] for state in joint_states]

        return joint_positions

    @property
    def motor_joint_velocities(self):
        """ returns the motor joint positions for "each DoF" according to pb.

        Note: fixed joints have 0 degrees of freedoms.
        """
        joint_states = pb.getJointStates(
        self._pb_sawyer, range(pb.getNumJoints(self._pb_sawyer,
                                                  physicsClientId=self._clid)),
        physicsClientId=self._clid)
        # Joint info specifies type of joint ("fixed" or not)
        joint_infos = [pb.getJointInfo(self._pb_sawyer, i,  physicsClientId=self._clid) for i in range(pb.getNumJoints(self._pb_sawyer,physicsClientId=self._clid))]
        # Only get joint states of free joints
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[2] != pb.JOINT_FIXED]

        joint_velocities = [state[1] for state in joint_states]

        return joint_velocities

    def _calc_jacobian(self, q=None, dq=None, localPos=None):
        if q is None:
            q = self.q
        if dq is None:
            dq = self.dq 
        if localPos is None:
            localPos = [0, 0, 0]
        num_dof = len(self.motor_joint_positions)

        # append zeros to q to fit pybullet

        for extra_dof in range(num_dof - len(q)):
            q.append(0)
            dq.append(0)

        linear_jacobian, angular_jacobian = pb.calculateJacobian(
            bodyUniqueId=self._pb_sawyer,
            linkIndex=6,
            localPosition=localPos,
            objPositions=q,#[:num_dof],
            objVelocities=dq,#[:num_dof],
            objAccelerations=[0]*num_dof,
            physicsClientId=self._clid
            )

        self._linear_jacobian = np.reshape(
            linear_jacobian, (3, self.num_free_joints))

        self._angular_jacobian = np.reshape(
            angular_jacobian, (3, self.num_free_joints))

        self._jacobian = np.vstack(
            (self._linear_jacobian[:,:len(self.q)], self._angular_jacobian[:,:len(self.q)]))
        return linear_jacobian, angular_jacobian

    @property
    def J(self, q=None):
        """ Calculate the full jacobian using pb.
        """
        return self._jacobian

    @property
    def linear_jacobian(self):
        """The linear jacobian x_dot = J_t*q_dot
        """
        return self._linear_jacobian[:,:7]

    @property
    def angular_jacobian(self):
        """The linear jacobian x_dot = J_t*q_dot
        """    
        return self._angular_jacobian[:,:7]

    @property
    def mass_matrix(self):
        return self._mass_matrix

    def _calc_mass_matrix(self, q=None):
        if q is None:
            q = self.q
        mass_matrix = pb.calculateMassMatrix(self._pb_sawyer, 
            q, 
            physicsClientId=self._clid)
        self._mass_matrix =  np.array(mass_matrix)[:7,:7]
    
    @property 
    def torque_compensation(self):
        """ Sum of torques from gravity / coriolis at each joint.

        The low level controller for sawyer automatically compensates
        for gravity.
        """
        return np.zeros(7)

    @property
    def info(self):
        """
        Collect and return information about parameters of the robot
        :return: names and information
        """
        assembly_names = self._params.get_robot_assemblies()
        camera_info = self._params.get_camera_details()

        return assembly_names, camera_info

    # Controllers#######################################################
    def step(self, start):
        """Update the robot state and model, set torques from controller
        
        Note this is different from other robot interfaces, as the Sawyer
        low-level controller takes gravity compensation into account.
        """

        self.update_model()
     
        if self.action_set:     
            #t1 = time.time()

            torques = self.controller.run_controller() 
            #self.set_torques([0]*7)
        
            self.set_torques(torques)
 

            while (time.time() - start < LOOP_TIME):
                pass 
                #time.sleep(.00001)


        else:
            pass
            #print("ACTION NOT SET")

    def goto_q(self, qd, max_joint_speed_ratio=0.3, timeout=15.0,
               threshold=0.008726646):
        """
        Blocking call.
        Set joint position according to given values list. Note that the
        length of the list must match the number of joints.
        :param qd: A list of length DOF of desired joint configurations
        :param max_joint_speed_ratio: ratio of the maximum joint
        velocity that the motion should be executed at [0.3]
        :param timeout: seconds to wait for move to finish [15]
        :param threshold: position threshold in radians across each
        joint when move is considered successful [0.008726646]
        :return: None
        """
        rospy.logdebug("commanding joint angles")
        self.endTime = time.time()
        rospy.logdebug("calc time " + str(self.endTime- self.startTime))
        self.startTime = time.time()
        assert len(qd) == len(self._joint_names), \
            'Input number of position values must match the number of joints'

        command = {self._joint_names[i]: qd[i] for i in range(len(self._joint_names))}
        self._limb.set_joint_position_speed(max_joint_speed_ratio)
        self._limb.move_to_joint_positions(command, timeout, threshold)
        self.endTime = time.time()
        rospy.logdebug("intera blocking time " + str(self.endTime- self.startTime))

        rospy.logdebug('desired ee pose ' + str(self.prev_cmd))
        rospy.logdebug('current ee pose ' + str(self.ee_pose))


    def set_torques(self, desired_torques):
        """
        Set joint torques according to given values list. Note that the
        length of the list must match that of the joint indices.
        :param desired_torques: A list of length DOF of desired torques.
        :return: None
        """
        assert len(desired_torques) == len(self._joint_names), \
            'Input number of torque values must match the number of joints'
        #print(desired_torques)
        command = {self._joint_names[i]: desired_torques[i] for i in range(len(self._joint_names))}

        self._limb.set_joint_torques(command)

    def q_dq_ddq(self, value):
        """
        Set joint position, velocity and acceleration
        :param value: A list of lists of length DOF with desired
        position, velocity and acceleration for each joint.
        :return: None
        """
        assert len(value[0]) == len(self._joint_names), \
            'Input number of position values must match the number of joints'
        assert len(value[1]) == len(self._joint_names), \
            'Input number of velocity values must match the number of joints'
        assert len(value[2]) == len(self._joint_names), \
            'Input number of acceleration values must match the number of joints'

        self._limb.set_joint_trajectory(self._joint_names, value[0], value[1], value[2])


    def update_model(self):
        self._calc_jacobian()
        self._calc_mass_matrix()

        orn = R.from_quat(self.ee_orientation)
        self.model.update_states(ee_pos=np.asarray(self.ee_position),
                                 ee_ori= np.asarray(self.ee_orientation),
                                 ee_pos_vel=np.asarray(self.ee_v),
                                 ee_ori_vel=np.asarray(self.ee_omega),
                                 joint_pos=np.asarray(self.q[:7]),
                                 joint_vel=np.asarray(self.dq[:7]),
                                 joint_tau=np.asarray(self.tau),
                                 joint_dim=7, 
                                 torque_compensation=self.torque_compensation)                                 
                                 

        self.model.update_model(J_pos=self.linear_jacobian,
                                J_ori=self.angular_jacobian,
                                mass_matrix=self.mass_matrix)
    
    def update_redis(self):
        """ update db parameters for the robot on a regular loop

        """
        # Set initial redis keys
        robot_state = {
            ROBOT_STATE_TSTAMP_KEY : str(time.time()),
            ROBOT_STATE_EE_POS_KEY :  str(self.ee_position), 
            ROBOT_STATE_EE_POSE_KEY: str(self.ee_pose),
            ROBOT_STATE_EE_ORN_KEY: str(self.ee_orientation),
            ROBOT_STATE_EE_V_KEY: str(self.ee_v), 
            ROBOT_STATE_Q_KEY: str(self.q),
            ROBOT_STATE_DQ_KEY: str(self.dq),
            ROBOT_STATE_TAU_KEY: str(self.tau),
            ROBOT_MODEL_JACOBIAN_KEY: str(self.J),
            ROBOT_MODEL_L_JACOBIAN_KEY: str(self.linear_jacobian),
            ROBOT_MODEL_A_JACOBIAN_KEY: str(self.angular_jacobian),
            ROBOT_MODEL_MASS_MATRIX_KEY: str(self.mass_matrix)
        }

        self.redisClient.mset(robot_state)

    ## HELPER
    def get_free_joint_idx_dict(self):
        """ Create a dictionary linking the "free" (not fixed)
        joints in urdf to their corresponding indicies in the
        urdf file and pybullet indicies.

        Access indices from this dict using
        """

        free_joint_dict = {}
        num_joints_bullet = pb.getNumJoints(self._pb_sawyer)

        for i in range(num_joints_bullet):
            joint_info = pb.getJointInfo(self._pb_sawyer, i)

            joint_name = joint_info[1]
            joint_type = joint_info[2]

            # q_index the first position index in positional
            # q_index > -1 means it is not a fix joint
            # The joints of the arm are called right_j0-6
            joint_dict = {}

            if (joint_type != pb.JOINT_FIXED) and 'right_j' in joint_name:
                # to retrieve the index, remove prepending string
                q_index2 = int(joint_name.replace('right_j', ''))
                joint_dict['pb_index'] = i
                joint_dict['iif_index'] = q_index2

                free_joint_dict[joint_name] = joint_dict
                rospy.logdebug("free joint dict" +
                                str(free_joint_dict[joint_name]))
            else:
                rospy.logdebug("Not fixed")
                # reset the joint to the correct state
                #pb.resetJointState(self._pb_sawyer, i, self.q[q_index2])
        #rospy.logdebug("Free joint dict " +str(free_joint_dict))
        return free_joint_dict

    def show_image(self, image_path, rate=1.0):
        """
        Display given image on sawyer head display
        :param image_path: absolute path string of the image
        :param rate: refresh rate
        :return: None
        """
        self._display.display_image(image_path, display_rate=rate)

    @property
    def light(self):
        """
        Get the info (names) of all available lights
        :return: A dictionary where keys are light name strings, and
        values are their boolean on status
        """
        return {name: self._lights.get_light_state(name) for name in self._lights.list_all_lights()}

    @light.setter
    def light(self, name_on):
        """
        Set the status of given light
        :param name: string name of the light
        :param on: boolean True for on, False for off
        :return: True if light state is set, False if not
        """
        name_on = (name, on)
        self._lights.set_light_state(name, on)

    def set_max_speed(self, factor):
        """
        Set the factor of the max speed of the robot joints to move with
        :param factor: factor of the max speed to move the joints
        :return: None
        """
        self._limb.set_joint_position_speed(factor)

    def set_grasp_weight(self, weight):
        """
        Set the weight of object the gripper is grasping
        :param weight: float in kg
        :return: True if successful, False if failure
        """
        return self._gripper.set_object_weight(weight)


    def _msg_wrapper(self, ctype):

        if len(self.cmd) == len(self._joints):
            if ctype == 'velocity':
                self._joint_cmd_msg.mode = JointCommand.VELOCITY_MODE
                self._joint_cmd_msg.velocity = self.cmd

            elif ctype == 'torque':
                self._joint_cmd_msg.mode = JointCommand.TORQUE_MODE
                self._joint_cmd_msg.effort = self.cmd
            else:
                raise NotImplemented

            self._command_msg.header.stamp = rospy.Time.now()

    def _from_tip_to_base_of_gripper(self, tip_pose):
        """
        Transforms a pose from the tip to the base of the gripper
        PyBullet functions (e.g. inverse kin) are queried wrt the
        base of the gripper but Intera provides
        the pose of the tip of the gripper
        :param tip_pose: Pose (x, y, z, qx, qy, qz, qw) of the tip of the gripper
        :return: Pose (x, y, z, qx, qy, qz, qw) of the base of the gripper
        """
        # This is the static transformation from base to tip of the gripper as
        # defined in the URDF
        T_ee_base = np.eye(4)
        T_ee_base[0:3, 0:3] = np.array(
            pb.getMatrixFromQuaternion([0.000, 0.000, np.sqrt(2) / 2,
                np.sqrt(2) / 2])).reshape((3, 3))

        T_ee_base[0:3, 3] = [0.000, 0.000, 0.0245]
        #T_ee_base[0:3, 3] = [0.000, 0.000, np.linalg.norm(
        #                                                [-0.11,0.1053, 0.0245])]

        T_tip = np.eye(4)
        T_tip[0:3, 0:3] = np.array(
            pb.getMatrixFromQuaternion(tip_pose[3:7])).reshape((3, 3))
        T_tip[0:3, 3] = tip_pose[0:3]

        T_base = T_tip.dot(np.linalg.inv(T_ee_base))

        base_pose = 7 * [0]
        base_pose[0:3] = T_base[0:3, 3]

        # In pyQuaternion the elements are ordered as (w, x, y, z)
        base_pose[3:6] = pyQuaternion(matrix=T_base).elements[1:4]
        base_pose[6] = pyQuaternion(matrix=T_base).elements[0]
        return base_pose


    def _distance_between_position_msgs(self, pos1, pos2):
        """
        Computes the L2 distance between two ROS Point3 messages
        :param pos1: one 3D point msg
        :param pos2: another 3D point msg
        :return: distance between points
        """
        return np.sqrt((pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2 + (pos1.z-pos2.z)**2)

    def pose_endpoint_transform(self, pose, curr_endpoint='right_hand', des_endpoint='right_gripper_tip'):
        '''
        Takes a pose from one endpoint reference frame to another. For example, instead of moving the right_hand to a desired
        position, you can move the right_gripper_tip to that position.
        :param pose: list of the pose. [x,y,z,qx,qy,qz,qw]
        :param curr_endpoint: current endpoint of the Sawyer Arm. Default to 'right_hand'
        :param des_endpoint: where you would like the endpoint to be. Default to 'right_gripper_tip'
        return: list of the new pose. [x,y,z,qx,qy,qz,qw]
        '''
        if self.transform_matrix is None:
            listener = tf.TransformListener()
            start_time = rospy.Time.now().secs
            listener.waitForTransform(curr_endpoint, des_endpoint, rospy.Time(0), rospy.Duration(1.0))
            (trans1,rot1) = listener.lookupTransform(des_endpoint, curr_endpoint, rospy.Time(0))

            trans1_mat = tf.transformations.translation_matrix(trans1)
            rot1_mat   = tf.transformations.quaternion_matrix(rot1)
            self.transform_matrix = np.dot(trans1_mat, rot1_mat)

        trans2 = pose[0:3]
        rot2 = pose[3:]
        trans2_mat = tf.transformations.translation_matrix(trans2)
        rot2_mat = tf.transformations.quaternion_matrix(rot2)
        mat2 = np.dot(trans2_mat, rot2_mat)

        mat3 = np.dot(mat2, self.transform_matrix)
        trans3 = tf.transformations.translation_from_matrix(mat3)
        rot3 = tf.transformations.quaternion_from_matrix(mat3)

        # return pose
        return trans3.tolist()+rot3.tolist()
    #### REDIS / CONTROL INTERFACE SPECIFIC ATTRIBUTES AND FUNCTIONS #############

    @property
    def env_connected(self):
        """ Flag indicating if environment and RobotInterface are connected.
        """
        return self.redisClient.get(ROBOT_ENV_CONN_KEY)

    @property
    def cmd_type(self):
        """ Redis key identifying the type of command robot should execute
        """
        return self.redisClient.get(ROBOT_CMD_TYPE_KEY)
       
    @property
    def controller_goal(self):
        return self.redisClient.get(CONTROLLER_GOAL_KEY)

    @property 
    def controlType(self):
        return self._controlType
        #return self.redisClient.get(CONTROLLER_CONTROL_TYPE_KEY)
    
    @controlType.setter
    def controlType(self, new_type):
        self._controlType = new_type 

    def get_cmd_tstamp(self):
        return self.redisClient.get(ROBOT_CMD_TSTAMP_KEY)

    def process_cmd(self, cmd_type, controller_goal):
        # print("CMD TYPE {}".format(self.cmd_type))

        # todo hack for states.         
        
        if (cmd_type == b"set_ee_pose"):

            self.set_ee_pose(**controller_goal)
        elif (cmd_type == b'move_ee_delta'):
            self.move_ee_delta(**controller_goal)

        # elif(cmd_type == b'set_joint_positions'):
        #     print("joint position command received")
        #     #self.check_controller("JointImpedance")
        #     self.set_joint_positions(**controller_goal)
        # elif(cmd_type == b'set_joint_delta'):
        #     self.set_joint_delta(**controller_goal)
        # elif (cmd_type == b'torque'):
        #     raise NotImplementedError
        #     #self.tau = self.desired_torque
        elif(cmd_type == b'reset_to_neutral'):

            self.redisClient.set(ROBOT_RESET_COMPL_KEY, 'False')
            self.reset_to_neutral()
        # elif(cmd_type == b'ee_delta'):
        #     raise NotImplementedError
        #     #self.move_ee_delta(self.desired_state)
        elif cmd_type ==b'IDLE':
            # make sure action set if false
            self.action_set = False
            return
        elif cmd_type == b'CHANGE_CONTROLLER':
            self.controller = self.make_controller_from_redis(self.get_control_type(),
                self.get_controller_params())
        else:
            rospy.logwarn("Unknown command{}".format(cmd_type))
 
    def check_for_new_cmd(self):
        cmd_tstamp = self.get_cmd_tstamp()
        if self.last_cmd_tstamp is None or (self.last_cmd_tstamp!=cmd_tstamp):
            self.last_cmd_tstamp = cmd_tstamp
            return True
        else:
            return False

    def run(self):
        # if (self.env_connected == b'True'):
        #     self.controller = self.make_controller_from_redis(self.get_control_type(), self.get_controller_params())
        # TODO: Run controller once to initalize numba

        while True:
            start = time.time()
            self.log_start_times.append(start)
            if (self.env_connected == b'True'):
                if self.check_for_new_cmd():
                    self.process_cmd(self.cmd_type, self.controller_goal)
                self.step(start)
            else:
                break
        #self.log_start_times.append(time.time())
        #np.savez('dev/sawyer_ctrl_timing/0821_time_nuc/0821_ctrl_loop_sq.npz', start=np.array(self.log_start_times), end=np.array(self.log_end_times), allow_pickle=True)
        #np.savez('dev/sawyer_ctrl_timing/0821_time_nuc/0821_cmd_times.npz', start=np.array(self.cmd_start_times), end=np.array(self.cmd_end_times), allow_pickle=True)
        # np.savez('dev/sawyer_ctrl_timing/run_controller_times.npz', delay=np.array(self.controller_times), allow_pickle=True)
        # np.savez('dev/sawyer_ctrl_timing/sawye_ctrl_loop_times.npz', tstamps=np.array(self.loop_times), allow_pickle=True)
        # np.savez('dev/sawyer_ctrl_timing/cmd_end_times.npz', tstamps=np.array(self.cmd_end_time), allow_pickle=True)


    # def _on_joint_states(self, msg):
    #     pass
    #     # self.calc_mass_matrix()
    #     # self._calc_jacobian() 
    #     #self.update_redis()

### MAIN ###
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    # Create ros node
    rospy.init_node("sawyer_interface", log_level=rospy.DEBUG)
    #load_gazebo_models()
    ctrlInterface = SawyerCtrlInterface(use_safenet=False, use_moveit=False)
    # Control Loop
    threshold = 0.05
    rospy.loginfo("warming up redis...")
    rospy.sleep(10.0)
    rospy.loginfo("waiting for environment ... ")
    while (ctrlInterface.env_connected != b'True'):
        # do nothing until the environment is connected
        pass
    #if (ctrlInterface.redisClient.get('env_connected') == b'True'):
    rospy.loginfo('Environment connected... entering control loop')
    ctrlInterface.run()

    # Timing test: 
    # Print stats for run_controller step. 
"""
    print("Torque calculation: run_controller delay (s) \n")
    tdict =np.load('dev/sawyer_ctrl_timing/run_controller_times.npz')
    delay_times = tdict['delay']
    print("Num samples collected:\t{}\n".format(len(delay_times)))
    print("Mean delay:\t{}\n".format(np.mean(delay_times)))
    print("Max delay:\t{}\n".format(np.max(delay_times)))
    print("index of max:\t{}\n".format(np.argmax(delay_times)))
    print("First sample:\t{}\n".format(delay_times[0]))
    print("sample 2:\t{}\n".format(delay_times[1]))
    print("Last sample:\t{}\n".format(delay_times[-1]))
"""
    # while(ctrlInterface.env_connected == b'True'):
    #     rospy.logdebug('Environment connected')
    #     start = time.time()
    #     ctrlInterface.process_cmd()

    #     ctrlInterface.step()
    #     ctrlInterface.update()

    #     while ((time.time() - start) < 0.001):
    #         pass





