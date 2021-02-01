"""Implementation of ControlInterface for Sawyer. Class definition and execution.
Runs in a separate python2.7 environment on Sawyer Workstation
"""
from __future__ import division
from perls2.robots.robot_interface import RobotInterface
from perls2.controllers.ee_imp import EEImpController
from perls2.controllers.ee_posture import EEPostureController
from perls2.controllers.joint_imp import JointImpController
from perls2.controllers.joint_torque import JointTorqueController
from perls2.controllers.joint_vel import JointVelController
from perls2.controllers.interpolator.linear_interpolator import LinearInterpolator
from perls2.controllers.interpolator.linear_ori_interpolator import LinearOriInterpolator
from perls2.controllers.robot_model.model import Model
import time
import rospy
from intera_core_msgs.msg import JointCommand
try:
    import intera_interface as iif
    from intera_core_msgs.srv import (
        SolvePositionIK,
    )
except ImportError:
    print('intera_interface not imported, did you remember to run cd ~/ros_ws;./intera.sh?')
import numpy as np
import logging
# Pybullet used for calculating mass matrix, jacobians.
import pybullet as pb
from perls2.utils.yaml_config import YamlConfig
import json
from perls2.redis_interfaces.redis_interface import RobotRedisInterface as RobotRedis
from perls2.redis_interfaces.redis_keys import *
from perls2.redis_interfaces.redis_values import *
import perls2.controllers.utils.transform_utils as T

LOOP_LATENCY = 0.000
LOOP_TIME = (1.0 / 500.0) - LOOP_LATENCY

# Redis reads commands as bytes strings
# compatible with python 2.7
bSET_EE_POSE = bytes(SET_EE_POSE)
bMOVE_EE_DELTA = bytes(MOVE_EE_DELTA)
bSET_JOINT_DELTA = bytes(SET_JOINT_DELTA)
bSET_JOINT_POSITIONS = bytes(SET_JOINT_POSITIONS)
bSET_JOINT_VELOCITIES = bytes(SET_JOINT_VELOCITIES)
bSET_JOINT_TORQUES = bytes(SET_JOINT_TORQUES)
bIDLE = bytes(IDLE)
bCHANGE_CONTROLLER = bytes(CHANGE_CONTROLLER)
bRESET = bytes(RESET)


class SawyerCtrlInterface(RobotInterface):
    """ Class definition for Sawyer Control Interface.

    Interface creates and monitors RedisServer for new commands from RobotInterface.
    Commands are interpreted and converted into set of joint torques sent to Sawyer via ROS.

    Attributes:
        config (YamlConfig): YamlConfig expressing default sawyer_ctrl_parameters and redis connection info.

    """
    def __init__(self,
                 config='cfg/sawyer_ctrl_config.yaml',
                 controlType='EEImpedance',
                 use_safenet=False,
                 node_name='sawyer_interface'):
        """
        Initialize Sawyer Robot for control

        Args:
            use_safenet: bool
                whether to use the Safenet to constrain ee positions
                and velocities
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
        self.config = YamlConfig(config)
        self.redisClient = RobotRedis(**self.config['redis'])
        self.redisClient.flushall()

        # Timing
        self.startTime = time.time()
        self.endTime = time.time()
        self.action_set = False
        self.model = Model()
        self.ee_name = self.config['sawyer']['end_effector_name']
        if config is not None:
            if self.config['controller']['interpolator']['type'] == 'linear':
                interp_kwargs = {'max_dx': 0.005,
                                 'ndim': 3,
                                 'controller_freq': 500,
                                 'policy_freq': 20,
                                 'ramp_ratio': 0.2}
                self.interpolator_pos = LinearInterpolator(**interp_kwargs)
                self.interpolator_ori = LinearOriInterpolator(**interp_kwargs)
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

        # set up internal pybullet simulation for jacobian and mass matrix calcs.
        self._clid = pb.connect(pb.DIRECT)
        pb.resetSimulation(physicsClientId=self._clid)

        # TODO: make this not hard coded
        sawyer_urdf_path = self.config['sawyer']['arm']['path']

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
        self._joint_names = self.config['sawyer']['limb_joint_names']
        self.free_joint_dict = self.get_free_joint_dict()
        self.joint_dict = self.get_joint_dict()
        self._link_id_dict = self.get_link_dict()

        rospy.loginfo('Sawyer initialization finished after {} seconds'.format(time.time() - start))

        # Set desired pose to initial
        self.neutral_joint_position = self.config['sawyer']['neutral_joint_angles'] #[0, -1.18, 0.00, 2.18, 0.00, 0.57, 3.3161]

        self.prev_cmd = np.asarray(self.neutral_joint_position)
        self.current_cmd = self.prev_cmd

        self.reset_to_neutral()
        self.default_control_type = self.config['controller']['selected_type']
        self.default_params = self.config['controller']['Real'][self.default_control_type]
        self.redisClient.set(CONTROLLER_CONTROL_TYPE_KEY, self.default_control_type)
        self.redisClient.set(CONTROLLER_CONTROL_PARAMS_KEY, json.dumps(self.default_params))

        self.pb_ee_pos_log = []
        self.pb_ee_ori_log = []
        self.pb_ee_v_log = []
        self.pb_ee_w_log = []

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

        self.redisClient.set(ROBOT_CMD_TSTAMP_KEY, time.time())
        self.redisClient.set(ROBOT_SET_GRIPPER_CMD_KEY, 0.1)
        self.redisClient.set(ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY, time.time())
        self.last_cmd_tstamp = self.get_cmd_tstamp()
        self.last_gripper_cmd_tstamp = self.get_gripper_cmd_tstamp()
        self.prev_gripper_state = self.des_gripper_state
        rospy.logdebug('Control Interface initialized')

        # TIMING TEST ONLY
        self.timelog_start = open('cmd_tstamp.txt', 'w')
        self.timelog_end = open('cmd_set_time.txt', 'w')

        self.controller_times = []
        self.loop_times = []
        self.cmd_end_time = []


    def make_controller_from_redis(self, control_type, controller_dict):
        print("Making controller {} with params: {}".format(control_type, controller_dict))
        self.controlType = control_type
        if control_type == EE_IMPEDANCE:
            interp_kwargs = {'max_dx': 0.005,
                             'ndim': 3,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            self.interpolator_ori = LinearOriInterpolator(**interp_kwargs)
            return EEImpController(
                self.model,
                interpolator_pos=self.interpolator_pos,
                interpolator_ori=self.interpolator_ori, **controller_dict)
        elif control_type == EE_POSTURE:
            interp_kwargs = {'max_dx': 0.005,
                             'ndim': 3,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            self.interpolator_ori = LinearOriInterpolator(**interp_kwargs)
            return EEPostureController(
                self.model,
                interpolator_pos=self.interpolator_pos,
                interpolator_ori=self.interpolator_ori, **controller_dict)
        elif control_type == JOINT_IMPEDANCE:
            interp_kwargs = {'max_dx': 0.05,
                             'ndim': 7,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            return JointImpController(
                self.model,
                interpolator_qpos=self.interpolator_pos,
                **controller_dict)
        elif control_type == JOINT_TORQUE:
            interp_kwargs = {'max_dx': 0.05,
                             'ndim': 7,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            return JointTorqueController(
                self.model,
                interpolator=self.interpolator_pos,
                **controller_dict)
        elif control_type == JOINT_VELOCITY:
            interp_kwargs = {'max_dx': 0.05,
                             'ndim': 7,
                             'controller_freq': 500,
                             'policy_freq': 20,
                             'ramp_ratio': 0.2}
            self.interpolator_pos = LinearInterpolator(**interp_kwargs)
            return JointVelController(
                self.model,
                interpolator=self.interpolator_pos,
                **controller_dict)
        else:
            raise ValueError("Invalid control type.")

    def get_controller_params(self):
        return self.redisClient.get(CONTROLLER_CONTROL_PARAMS_KEY)

    def get_control_type(self):
        return self.redisClient.get(CONTROLLER_CONTROL_TYPE_KEY)

    def reset_to_neutral(self):
        """Blocking call for resetting the arm to neutral position
        """
        self.redisClient.set(ROBOT_RESET_COMPL_KEY, 'False')
        rospy.loginfo("Resetting to neutral")
        self.goto_q(self.neutral_joint_position, max_joint_speed_ratio=0.2)
        self.set_gripper_to_value(1.0)
        self.redisClient.set(ROBOT_RESET_COMPL_KEY, 'True')
        self.action_set = False
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

    def get_ee_v_world(self):
        """
        Returns ee_v in world frame, after applying transformations from eef ori.
        """
        return np.dot(np.array(self.ee_orientation), np.transpose(np.array(self.ee_v)))

    @property
    def ee_omega(self):
        """
        Get the current twist velocity of end effector in the eef frame.
        :return: a list of floats for the twist velocity [vx, vy, vz,
        wx, wy, wz]
        """
        return list(self._limb.endpoint_velocity()['angular'])

    def get_ee_omega_world(self):
        """
        Returns ee_v in world frame, after applying transformations from eef ori.
        """
        return np.dot(np.array(self.ee_orientation), np.transpose(np.array(self.ee_omega)))

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

    def _get_motor_joint_indices(self):
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

    def get_free_joint_dict(self):
        """Dictionary of free joints in urdf and corresponding indicies.
        """
        free_joint_dict = {}

        num_joints = pb.getNumJoints(
            self._pb_sawyer, physicsClientId=self._clid)

        for joint_id in range(num_joints):

            joint_ind, joint_name, joint_type, _, _, _, _, _, _, _, _, _, _, _, _, _, _ = (
                pb.getJointInfo(
                    bodyUniqueId=self._pb_sawyer,
                    jointIndex=joint_id,
                    physicsClientId=self._clid))
            if joint_type != pb.JOINT_FIXED:
                free_joint_dict.update(
                    {
                        joint_name: joint_ind
                    })

        return free_joint_dict

    def _pad_zeros_to_joint_values(self, des_joint_values):
        """Pads zeros to appropriate joint index positions for joints not listed in limb_joint_names
        """
        # Order the joints by their value from least to greatest.
        sorted_free_joints = sorted(self.free_joint_dict.items(), key=lambda x: x[1], reverse=False)
        padded_joint_values = [0] * len(sorted_free_joints)
        des_joint_index = 0
        for joint_ind in range(len(sorted_free_joints)):
            # If the joint name is a motor joint
            if sorted_free_joints[joint_ind][0] in self._joint_names:
                padded_joint_values[joint_ind] = des_joint_values[des_joint_index]
                des_joint_index += 1

        return padded_joint_values

    @property
    def num_free_joints(self):
        return len(self._get_motor_joint_indices())

    @property
    def motor_joint_positions(self):
        return self._motor_joint_positions

    def get_motor_joint_positions(self):
        """ returns the motor joint positions for "each DoF" according to pb.

        Note: fixed joints have 0 degrees of freedoms.
        """
        joint_states = pb.getJointStates(
            self._pb_sawyer,
            range(pb.getNumJoints(
                self._pb_sawyer,
                physicsClientId=self._clid)),
            physicsClientId=self._clid)
        # Joint info specifies type of joint ("fixed" or not)
        joint_infos = [pb.getJointInfo(self._pb_sawyer, i, physicsClientId=self._clid) for i in range(
            pb.getNumJoints(self._pb_sawyer, physicsClientId=self._clid))]
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
            self._pb_sawyer,
            range(pb.getNumJoints(
                self._pb_sawyer,
                physicsClientId=self._clid)),
            physicsClientId=self._clid)
        # Joint info specifies type of joint ("fixed" or not)
        joint_infos = [pb.getJointInfo(self._pb_sawyer, i, physicsClientId=self._clid) for i in range(pb.getNumJoints(self._pb_sawyer, physicsClientId=self._clid))]
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

        # append zeros to q to fit pybullet
        q = self._pad_zeros_to_joint_values(q)
        dq = self._pad_zeros_to_joint_values(dq)

        linear_jacobian, angular_jacobian = pb.calculateJacobian(
            bodyUniqueId=self._pb_sawyer,
            linkIndex=self._link_id_dict[self.ee_name],
            localPosition=localPos,
            objPositions=q,
            objVelocities=dq,
            objAccelerations=[0] * self.num_free_joints,
            physicsClientId=self._clid)

        # Save full linear jacoban as 3 x 10
        self._linear_jacobian = np.reshape(
            linear_jacobian, (3, self.num_free_joints))
        # save full angular jacobian as 3x10
        self._angular_jacobian = np.reshape(
            angular_jacobian, (3, self.num_free_joints))
        # Save full jacobian as 6x10
        self._jacobian = np.vstack(
            (self._linear_jacobian, self._angular_jacobian))
        return linear_jacobian, angular_jacobian

    @property
    def J(self, q=None):
        """ Full jacobian using pb as a 6x7 matrix
        """
        return self._jacobian[:, :7]

    @property
    def linear_jacobian(self):
        """The linear jacobian x_dot = J_t*q_dot using pb as a 3x7 matrix
        """
        return self._linear_jacobian[:, :7]

    @property
    def angular_jacobian(self):
        """The linear jacobian x_dot = J_t*q_dot
        """
        return self._angular_jacobian[:, :7]

    @property
    def mass_matrix(self):
        return self._mass_matrix

    def _calc_mass_matrix(self, q=None):
        if q is None:
            q = self.q
        mass_matrix = pb.calculateMassMatrix(
            self._pb_sawyer,
            q,
            physicsClientId=self._clid)

        self._mass_matrix = np.array(mass_matrix)[:7, :7]

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
            torques = self.controller.run_controller()
            # self.set_torques([0]*7)

            torques = np.clip(torques, -5.0, 5.0)
            #print("Torques {}".format(torques))

            self.set_torques(torques)

            while (time.time() - start < LOOP_TIME):
                pass

        else:
            pass

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
        rospy.logdebug("calc time " + str(self.endTime - self.startTime))
        self.startTime = time.time()
        assert len(qd) == len(self._joint_names), \
            'Input number of position values must match the number of joints'

        command = {self._joint_names[i]: qd[i] for i in range(len(self._joint_names))}
        self._limb.set_joint_position_speed(max_joint_speed_ratio)
        self._limb.move_to_joint_positions(command, timeout, threshold)
        self.endTime = time.time()
        rospy.logdebug("intera blocking time " + str(self.endTime - self.startTime))

    def set_torques(self, desired_torques):
        """
        Set joint torques according to given values list. Note that the
        length of the list must match that of the joint indices.
        :param desired_torques: A list of length DOF of desired torques.
        :return: None
        """
        assert len(desired_torques) == len(self._joint_names), \
            'Input number of torque values must match the number of joints'

        command = {self._joint_names[i]: desired_torques[i] for i in range(len(self._joint_names))}
        self._limb.set_joint_torques(command)

    def get_link_id_from_name(self, link_name):
        """Get link id from name

        Args:
            link_name (str): name of link in urdf file
        Returns:
            link_id (int): index of the link in urdf file.
             OR -1 if not found.
        """
        if link_name in self._link_id_dict:
            return self._link_id_dict.get(link_name)
        else:
            return -1

    def get_link_dict(self):
        """Create a dictionary between link id and link name
        Dictionary keys are link_name : link_id

        Notes: Each link is connnected by a joint to its parent, so
               num links = num joints
        """

        num_links = pb.getNumJoints(
            self._pb_sawyer, physicsClientId=self._clid)

        link_id_dict = {}

        for link_id in range(num_links):
            link_id_dict.update(
                {
                    self.get_link_name(
                        (self._pb_sawyer, link_id)).decode('utf-8'): link_id
                })

        return link_id_dict

    def get_link_name(self, link_uid):
        """Get the name of the link. (joint)

        Parameters
        ----------
        link_uid :
            A tuple of the body Unique ID and the link index.

        Returns
        -------

            The name of the link.

        """
        arm_id, link_ind = link_uid
        _, _, _, _, _, _, _, _, _, _, _, _, link_name, _, _, _, _ = pb.getJointInfo(
            bodyUniqueId=arm_id,
            jointIndex=link_ind,
            physicsClientId=self._clid)
        return link_name

    def get_joint_dict(self):
        """Create a dictionary between link id and link name
        Dictionary keys are link_name : link_id

        Notes: Each link is connnected by a joint to its parent, so
               num links = num joints
        """

        num_joints = pb.getNumJoints(
            self._pb_sawyer, physicsClientId=self._clid)

        joint_id_dict = {}

        for joint_id in range(num_joints):
            joint_id_dict.update(
                {
                    self.get_joint_name(
                        (self._pb_sawyer, joint_id)).decode('utf-8'): joint_id
                })

        return joint_id_dict

    def get_joint_name(self, link_uid):
        """Get the name of the joint

        Parameters
        ----------
        link_uid :
            A tuple of the body Unique ID and the link index.

        Returns
        -------

            The name of the link.

        """
        arm_id, joint_ind = link_uid
        _, joint_name, _, _, _, _, _, _, _, _, _, _, _, _, _, _, _ = pb.getJointInfo(
            bodyUniqueId=arm_id,
            jointIndex=joint_ind,
            physicsClientId=self._clid)
        return joint_name

    def update_model(self):
        self._calc_jacobian()
        self._calc_mass_matrix()

        # Reset pybullet model to joint State to get ee_pose and orientation.
        for joint_ind, joint_name in enumerate(self._joint_names):
            pb.resetJointState(
                bodyUniqueId=self._pb_sawyer,
                jointIndex=self.joint_dict[joint_name],
                targetValue=self.q[joint_ind],
                targetVelocity=self.dq[joint_ind],
                physicsClientId=self._clid)

        pb_ee_pos, pb_ee_ori, _, _, _, _, pb_ee_v, pb_ee_w = pb.getLinkState(
            self._pb_sawyer,
            self._link_id_dict[self.ee_name],
            computeLinkVelocity=1,
            computeForwardKinematics=1,
            physicsClientId=self._clid)

        # Get ee orientation as a matrix
        ee_ori_mat = T.quat2mat(self.ee_orientation)
        # Get end effector velocity in world frame
        ee_v_world = np.dot(ee_ori_mat, np.array(self.ee_v).transpose())

        ee_w_world = np.dot(ee_ori_mat, np.array(self.ee_omega).transpose())
        self.ee_omega_world = ee_w_world
        self.model.update_states(ee_pos=np.asarray(self.ee_position),
                                 ee_ori=np.asarray(self.ee_orientation),
                                 ee_pos_vel=np.asarray(ee_v_world),
                                 ee_ori_vel=np.asarray(ee_w_world),
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
            ROBOT_STATE_TSTAMP_KEY: str(time.time()),
            ROBOT_STATE_EE_POS_KEY: str(self.ee_position),
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
                rospy.logdebug("free joint dict" + str(free_joint_dict[joint_name]))
            else:
                rospy.logdebug("Not fixed")

        return free_joint_dict

    def set_max_speed(self, factor):
        """
        Set the factor of the max speed of the robot joints to move with
        :param factor: factor of the max speed to move the joints
        :return: None
        """
        self._limb.set_joint_position_speed(factor)

    def close_gripper(self):
        """
        Close the gripper of the robot
        :return: None
        """
        if self._has_gripper:
            self._gripper.close()

    def open_gripper(self):
        """
        Open the gripper of the robot
        :return: None
        """
        if self._has_gripper:
            self._gripper.open()

    def set_gripper_to_value(self, value):
        """
        Set the gripper grasping opening state
        :param openning: a float between 0 (closed) and 1 (open).
        :return: None
        """

        if self._has_gripper:
            scaled_pos = value * self._gripper.MAX_POSITION
            self._gripper.set_position(scaled_pos)

# ### REDIS / CONTROL INTERFACE SPECIFIC ATTRIBUTES AND FUNCTIONS #############

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
    def des_gripper_state(self):
        return float(self.redisClient.get(ROBOT_SET_GRIPPER_CMD_KEY))

    @property
    def controller_goal(self):
        return self.redisClient.get(CONTROLLER_GOAL_KEY)

    @property
    def controlType(self):
        return self._controlType

    @controlType.setter
    def controlType(self, new_type):
        self._controlType = new_type

    def get_gripper_cmd_tstamp(self):
        return self.redisClient.get(ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY)

    def get_cmd_tstamp(self):
        return self.redisClient.get(ROBOT_CMD_TSTAMP_KEY)

    def process_cmd(self, cmd_type):
        """ process command from redis

        Args:
            cmd_type (str): byte-array string from redis cmd key
        """
        if (cmd_type == bSET_EE_POSE):
            self.set_ee_pose(**self.controller_goal)
        elif (cmd_type == bMOVE_EE_DELTA):
            self.move_ee_delta(**self.controller_goal)
        elif(cmd_type == bSET_JOINT_DELTA):
            self.set_joint_delta(**self.controller_goal)
        elif (cmd_type == bSET_JOINT_POSITIONS):
            self.set_joint_positions(**self.controller_goal)
        elif (cmd_type == bSET_JOINT_TORQUES):
            self.set_joint_torques(**self.controller_goal)
        elif (cmd_type == bSET_JOINT_VELOCITIES):
            self.set_joint_velocities(**self.controller_goal)
        elif(cmd_type == bRESET):
            self.redisClient.set(ROBOT_RESET_COMPL_KEY, 'False')
            self.reset_to_neutral()
        elif (cmd_type == bIDLE):
            # make sure action set if false
            self.action_set = False
            return
        elif (cmd_type == bCHANGE_CONTROLLER):
            rospy.loginfo("CHANGE CONTROLLER COMMAND RECEIVED")
            self.controller = self.make_controller_from_redis(
                self.get_control_type(),
                self.get_controller_params())
        else:
            rospy.logwarn("Unknown command: {}".format(cmd_type))

    def check_for_new_gripper_cmd(self):
        gripper_cmd_tstamp = self.get_gripper_cmd_tstamp()
        if self.last_gripper_cmd_tstamp is None or (self.last_gripper_cmd_tstamp != gripper_cmd_tstamp):
            self.last_gripper_cmd_tstamp = gripper_cmd_tstamp
            return True
        else:
            return False

    def process_gripper_cmd(self):
        """Process the new gripper command by setting gripper state.

        Only send gripper command if current gripper open fraction is not
        equal to desired gripper open fraction.
        """
        if self.prev_gripper_state != self.des_gripper_state:
            self.set_gripper_to_value(self.des_gripper_state)
            self.prev_gripper_state = self.des_gripper_state
        else:
            pass


    def check_for_new_cmd(self):
        cmd_tstamp = self.get_cmd_tstamp()
        if self.last_cmd_tstamp is None or (self.last_cmd_tstamp != cmd_tstamp):
            self.last_cmd_tstamp = cmd_tstamp
            return True
        else:
            return False

    def run(self):
        if (self.env_connected == b'True'):
            self.controller = self.make_controller_from_redis(self.get_control_type(), self.get_controller_params())

        while True:
            start = time.time()
            self.log_start_times.append(start)
            if (self.env_connected == b'True'):
                if self.check_for_new_cmd():
                    self.process_cmd(self.cmd_type)
                if self.check_for_new_gripper_cmd():
                    self.process_gripper_cmd()
                self.step(start)
            else:
                break

### MAIN ###
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    # Create ros node
    rospy.init_node("sawyer_interface", log_level=rospy.DEBUG)

    ctrlInterface = SawyerCtrlInterface(use_safenet=False)

    # Control Loop
    rospy.loginfo("warming up redis...")
    rospy.sleep(10.0)
    rospy.loginfo("waiting for environment ... ")
    try:
        while not rospy.is_shutdown():
            if (ctrlInterface.env_connected != b'True'):
                pass
            else:
                break
    except KeyboardInterrupt:
        pass

    while (ctrlInterface.env_connected == b'True'):
        rospy.loginfo('Environment connected... entering control loop')
        ctrlInterface.run()

    rospy.loginfo("Env disconnected. shutting down.")





