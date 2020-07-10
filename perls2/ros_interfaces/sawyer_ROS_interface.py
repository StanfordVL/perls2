"""Implementation of ControlInterface for Sawyer
Runs in a separate python2.7 environment on Sawyer Workstation
"""

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

import pybullet as pb
import time
import PyKDL as KDL
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF

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
from safenet import SafenetMonitor

from perls2.worlds.bullet_world import BulletWorld

def bstr_to_ndarray(array_bstr):
    """Convert bytestring array to 1d array
    """
    return np.fromstring(array_bstr[1:-1], dtype=np.float, sep = ',')

class SawyerCtrlInterface(object):
    def __init__(self, 
                 config='cfg/sawyer_ctrl_config.yaml', 
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
        # Connect to redis client
        # use default port 6379 at local host.
        # TODO:  match these up in cfg file later.
        self.redisClient = redis.Redis()

        ## Timing
        self.startTime = time.time()
        self.endTime = time.time()

        # set up rospy node
        #if node_name  and rospy.get_name() != '/unnamed':
        #rospy.init_node(node_name)
        # assert rospy.get_name() != '/unnamed', \
        #     'You must init a ROS node! Call rospy.init_node(\'node_name\')'
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

        self._limb = iif.Limb()
        self._joint_names = self._limb.joint_names()

        self._use_safenet = use_safenet
        if self._use_safenet:
            self.safenet = SafenetMonitor('right_hand')

        self.transform_matrix = None

        # self._navigator = iif.Navigator()
        self.cmd = []

        # TODO: add replacement for Joint Commands
        #self._joint_cmd_msg = JointCommand()
        #self._joint_cmd_msg.names = self._joint_names

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

        self._clid = pb.connect(pb.DIRECT)
        
        pb.resetSimulation()
        # TODO: make this not hard coded
        sawyer_urdf_path = \
            rospack.get_path('perls_robot_interface_ros') + '/models/sawyer_pybullet.urdf'

        self._pb_sawyer = pb.loadURDF(
                                        sawyer_urdf_path,
                                        (0, 0, 0),
                                        useFixedBase=True)

        # Get dictionary of free (not fixed) joint indices of urdf
        #self._free_joint_idx_dict = self.get_free_joint_idx_dict()

        # create a URDF object from an xml file
        urdf_robot = URDF.from_xml_file(sawyer_urdf_path)

        # construct a KDL tree from an URDF object
        (parse_ok, kdl_tree) = treeFromUrdfModel(urdf_robot)

        # get a KDL kinematic chain from a KDL tree
        self._kdl_chain = kdl_tree.getChain('base', 'right_hand')

        try:
            ns = "ExternalTools/right/PositionKinematicsNode/IKService"
            self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
            rospy.wait_for_service(ns, 5.0)
            self._ik_service = True
        except:
            rospy.logerr('IKService from Intera timed out')
            self._ik_service = False

        self._interaction_options = InteractionOptions()
        if len(self._interaction_options._data.D_impedance) == 0:
            self._interaction_options._data.D_impedance = [8, 8, 8, 2, 2, 2]

        self._interaction_options_pub = \
            rospy.Publisher('/robot/limb/right/interaction_control_command',
                            InteractionControlCommand, queue_size = 1)

        rospy.loginfo('Sawyer initialization finished after {} seconds'.format(time.time() - start))

        # Set desired pose to initial
        curr_ee_pose = self.ee_pose
        self.neutral_joint_position = [0,-1.18,0.00,2.18,0.00,0.57,3.3161]

        self.prev_cmd = np.asarray(self.neutral_joint_position)
        self.current_cmd = self.prev_cmd

        self.reset_to_neutral()

        # Set initial values for redis db
        self.redisClient.set('robot::cmd_type', 'joint_position')
        self.redisClient.set('robot::qd', str(self.neutral_joint_position))


       #self.redisClient.set('robot::desired_ee_pose', str(curr_ee_pose))
        self.redisClient.set('robot::env_connected', 'False')
        # self.redisClient.set('run_controller', 'False')

        # Set initial redis keys
        self.redisClient.set('robot::ee_position', str(self.ee_position))
        self.redisClient.set('robot::ee_pose', str(self.ee_pose))
        self.redisClient.set('robot::ee_orientation', str(self.ee_orientation))

        #Set desired pose to current pose initially
        self.redisClient.set('robot::desired_ee_pose', str(self.ee_pose))
        self.redisClient.set('robot::tau_desired', str(self.tau))

        rospy.logdebug('Control Interface initialized')

        ##


    def reset_to_neutral(self):
        """Blocking call for resetting the arm to neutral position
        """
        self.redisClient.set('reset_complete', 'False')
        rospy.loginfo("Resetting to neutral")
        self.goto_q(self.neutral_joint_position,  max_joint_speed_ratio=0.2)

        # set to joint position control to avoid repeating reset.
        # self.redisClient.set('robot::cmd_type', 'joint_position')
        # self.redisClient.set('robot::qd', str(self.neutral_joint_position))
        self.redisClient.set('reset_complete', 'True')
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

    #@ee_position.setter
    def set_ee_position(self, position):
        """ set
         ee position
        """
        rospy.logdebug('set ee position')
        self.ee_pose = (list(position) + self.ee_orientation)

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

    @ee_pose.setter
    def ee_pose(self, xd, realtime = False):
        """
        Set the end effector pose.
        Use this function for poses that are close to the current pose.
        There are two options for the implementation of this function:
        1) Operational space control. NOT IN SAWYER
        2) IK and set q or goto_q (only if the goal is close) -> the
        motion "interpolates" in joint space!
        :param xd:

        Args:
            xd: list
                list of desired position, orientation values [x, y,
                z, qx, qy, qz, qw]
            realtime: whether to use realtime for pb simulation.

        Notes:
        Uses a pybullet simulation with matching urdf to calculate the
        corresponding joint angles for the desired pose via inverse
        kinematics.

        """
        rospy.logdebug("calculating joint angles for ee_pose")
        self.startTime = time.time()

        useRealTimeSimulation = 0
        # This only affects when we simulate (avoids having to step)
        pb.setRealTimeSimulation(useRealTimeSimulation)
        rospy.logdebug("Current joint configuration: %s", self.q)
        num_joints_bullet = pb.getNumJoints(self._pb_sawyer)
        for i in range(num_joints_bullet):
            joint_info = pb.getJointInfo(self._pb_sawyer, i)
            q_index = joint_info[3]
            # q_index > -1 means it is not a fix joint
            # The joints of the arm are called right_j0-6
            if q_index > -1 and 'right_j' in joint_info[1]:
                q_index2 = int(joint_info[1].replace('right_j', ''))
                pb.resetJointState(self._pb_sawyer, i, self.q[q_index2])

        xd_bullet = self._from_tip_to_base_of_gripper(xd)
        #xd_bullet = xd

        # We iterate several times (max max_iter) using the last
        # solution until the distance (only translation) to the desired
        #configuration is small enough (max max_dist)
        close_enough = False
        iter = 0
        max_iter = [20, 1][int(realtime)]
        max_dist = 0.001 # Meters
        sawyer_ee_index = 16  # Where is this magic number coming from?
        qd_full = None
        joint_dumping = [0.1] * num_joints_bullet
        while not close_enough and iter < max_iter:
            qd_full = pb.calculateInverseKinematics(self._pb_sawyer,
                                                    sawyer_ee_index,
                                                    xd_bullet[0:3],
                                                    # restPoses=self.q,
                                                    targetOrientation = xd_bullet[3:7],
                                                    jointDamping = joint_dumping)

            mod_qd_full = qd_full[0:1] + qd_full[2:] #qd_full contains head joint
            for i in range(num_joints_bullet):
                joint_info = pb.getJointInfo(self._pb_sawyer, i)
                q_index = joint_info[3]
                if q_index > -1 and 'right_j' in joint_info[1]:
                    q_index2 = int(joint_info[1].replace('right_j', ''))
                    pb.resetJointState(self._pb_sawyer,
                                       i,
                                       mod_qd_full[q_index2])
            link_new_state = pb.getLinkState(self._pb_sawyer,
                                             sawyer_ee_index)


            x_new = link_new_state[4]
            diff = [xd_bullet[0] - x_new[0],
                    xd_bullet[1] - x_new[1],
                    xd_bullet[2] - x_new[2]]  # Only position
            dist = np.sqrt(diff[0] * diff[0] +
                           diff[1] * diff[1] +
                           diff[2] * diff[2])
            close_enough = (dist < max_dist)
            iter += 1
        qd =[]
        qd_idx = 0
        # ignore head joint -> We need to know which one is the pan-head
        for i in range(num_joints_bullet):
            joint_info = pb.getJointInfo(self._pb_sawyer, i)
            q_index = joint_info[3]
            # q_index > -1 means it is not a fix joint
            # The joints of the arm are called right_j0-6
            if q_index > -1:
                if 'right_j' in joint_info[1]:
                    qd += [qd_full[qd_idx]]
                qd_idx += 1

        rospy.logdebug("Moving to the joint configuration %s to achieve"
                        + " the Cartesian goal", qd)

        # Depending on the amount of joint motion we will use one
        # function or another from the Intera SDK
        q_threshold = 100 #0.005  # 100
        if ((not self.blocking) and
            (np.linalg.norm(np.array(qd) - np.array(self.q)) < q_threshold)):
            rospy.logdebug("Calling direct motion to move the joints")
            self.q = qd
        else:
            rospy.logdebug("Calling goto to move the joints")
            self.goto_q(qd)


    def goto_ee_pose(self, xd, **kwargs):
        """
        Set the end effector pose.
        Use this funtion for larger distances to current pose.
        There are two options:
        1) Use moveit! to interpolate in cartesian the shortest path to
        xd and plan the sequence joint configurations to execute it ->
        -> the motion "interpolates" in Cartesian space even though
        follows a joint space trajectory
        2) Use the Intera SDK motion interface of the robot to first
        compute the joint configuration at the desired pose (IK service)
        and then plan and execute a trajectory in joint space towards it
        :param xd: list of desired position, orientation values [x, y,
        z, qx, qy, qz, qw]
        3) The kwargs parameter includes a flag to switch between pose
        interpolation and joint space interpolation. It also includes parameters
        to specify the arm's maximum linear and rotational speed and acceleration
        """
        if self._use_moveit:
            ee_pose = Pose()
            ee_pose.position.x = xd[0]
            ee_pose.position.y = xd[1]
            ee_pose.position.z = xd[2]
            ee_pose.orientation.x = xd[3]
            ee_pose.orientation.y = xd[4]
            ee_pose.orientation.z = xd[5]
            ee_pose.orientation.w = xd[6]
            self._moveit_group.set_pose_target(ee_pose)
            qd_trajectory_moveit = self._moveit_group.plan()

            if len(qd_trajectory_moveit.joint_trajectory.points) == 0:
                rospy.logerr('No viable plan to reach target pose.')
                return False
            else:
                self._moveit_group.execute(qd_trajectory_moveit)
                qd_trajectory = []
                for point in qd_trajectory_moveit.joint_trajectory.points:
                    js_waypoint = {'time': point.time_from_start,
                                   'pose': point.positions,
                                   'velocity': point.velocities,
                                   'acceleration': point.accelerations}
                    qd_trajectory += [js_waypoint]
                self.js_trajectory(qd_trajectory)
        else:
            ik_request = SolvePositionIKRequest()
            hdr = Header(stamp=rospy.Time.now(), frame_id='base')
            poses = {'right': PoseStamped(header=hdr,pose=Pose(position=Point(*xd[0:3]),
                                     orientation=Quaternion(*xd[3:7]),
                    ),
                ),
            }
            # Add desired pose for inverse kinematics
            ik_request.pose_stamp.append(poses["right"])
            # Request inverse kinematics from base to "right_hand" frame
            ik_request.tip_names.append('right_hand')
            try:
                resp = self._iksvc(ik_request)  # get IK response
                #print(resp)
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr("Service call failed: %s" % (e,))
                return False

            if resp.result_type[0] <= 0:
                rospy.logerr("IK response is not valid")
                return False
            else:

                wpt_opts = MotionWaypointOptions(label="0", n_dim=7, joint_tolerances=0.01,
                             max_linear_speed=kwargs.get("max_linear_speed", 0.3),
                             max_linear_accel=kwargs.get("max_linear_accel", 0.3),
                             max_rotational_speed=kwargs.get("max_rotational_speed", 0.75),
                             max_rotational_accel=kwargs.get("max_rotational_accel", 0.75),
                             max_joint_speed_ratio=1.0)
                start_waypoint = MotionWaypoint(limb=self._limb, options = wpt_opts.to_msg())
                traj_options = TrajectoryOptions()
                if kwargs['traj_type'] == 'joint_angles':
                    traj_options.interpolation_type = TrajectoryOptions.JOINT
                    start_waypoint.set_joint_angles(joint_angles=self.q) #Why is one of the waypoints our current location? Is this required?
                elif kwargs['traj_type'] == 'cartesian':

                    traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
                    poseStamped = PoseStamped(pose = Pose(position = Point(*self.ee_pose[0:3]), orientation = Quaternion(*self.ee_pose[3:7])))
                    start_waypoint.set_cartesian_pose(poseStamped, 'right_hand', self.q)
                else:
                    raise ValueError('kwargs traj_type not set.')


                qd_trajectory = MotionTrajectory(trajectory_options = traj_options, limb=self._limb)
                qd_trajectory.append_waypoint(start_waypoint.to_msg())

                end_waypoint = MotionWaypoint(limb=self._limb, options = wpt_opts.to_msg())
                if kwargs['traj_type'] == 'joint_angles':
                    end_waypoint.set_joint_angles(joint_angles=resp.joints[0].position)
                elif kwargs['traj_type'] == 'cartesian':
                    poseStamped = PoseStamped(pose=Pose(position=Point(*xd[0:3]), orientation=Quaternion(*xd[3:7])))
                    end_waypoint.set_cartesian_pose(poseStamped, 'right_hand', resp.joints[0].position)
                else:
                    raise ValueError('kwargs traj_type not set.')

                qd_trajectory.append_waypoint(end_waypoint.to_msg())

                strr = qd_trajectory.to_msg()
                result = qd_trajectory.send_trajectory()
                if result is None:
                    rospy.logerr('Trajectory FAILED to send')
                    return False

                if result.result:
                    rospy.loginfo('Motion controller successfully finished the trajectory!')
                else:
                    rospy.logerr('Motion controller failed to complete the trajectory with error '
                                 + str(result.errorId))
                    return False

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
    def ddq(self):
        """
        Get the joint accelerations of the robot arm.
        :return: a list of joint accelerations in radian/s^2, which
        is ordered by indices from small to large.
        Typically the order goes from base to end effector.
        """
        return NotImplemented

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
    def J(self, q=None):
        """
        Estimate and return the kinematic Jacobian at q or at the
        current configuration.
        :return: Jacobian matrix (2D numpy array)
        """

        q_kdl_array = KDL.JntArray(7)

        if q is None:
            q = self.q

        num_dof = len(self.q)

        for i in range(0, num_dof):
            q_kdl_array[i] = q[i]

        jacKDL = KDL.ChainJntToJacSolver(self._kdl_chain)
        jacobian = KDL.Jacobian(7)
        jacKDL.JntToJac(q_kdl_array, jacobian)
        jacobianMat = np.matrix(np.zeros([6, num_dof]))
        for i in range(0, 6):
            for j in range(0, num_dof):
                jacobianMat[i, j] = jacobian[i, j]
        return jacobianMat

    @property
    def J(self, q=None):
        if q is None:
            q = self.q
    @property
    def info(self):
        """
        Collect and return information about parameters of the robot
        :return: names and information
        """
        assembly_names = self._params.get_robot_assemblies()
        camera_info = self._params.get_camera_details()

        return assembly_names, camera_info

    def configure(self, configs):
        """
        Configure the parameters of the robot
        :param configs: configuration
        :return: None
        """
        return NotImplemented

    def get_link_name(self, uid, lid):
        """
        Get the name string of given link
        :param uid: integer body unique id
        :param lid: integer link id of body
        :return: name string of link on body
        """
        return NotImplemented

    def get_link_state(self, lid):
        """
        Get the state of given link on given body
        :param uid: integer body unique id
        :param lid: integer link id on body
        :return: a tuple of
        link world position (list of floats [x, y, z]),
        link world orientation (list of floats [qx, qy, qz, qw]),
        local position offset of CoM frame,
        local orientation offset of CoM frame,
        world position of asset file link frame,
        world orientation of asset file link frame,
        link world linear velocity,
        link world angular velocity.
        """
        return NotImplemented

    def get_joint_info(self, jid):
        """
        Get the information of body joint
        :param uid: integer unique body id
        :param jid: integer joint id on body
        :return: a tuple of
        joint index,
        joint name string in asset file,
        joint type string ('fixed', 'revolute',
        'prismatic', 'spherical', 'planar', 'gear',
        'point2point'),
        the first position index in the positional
        state variables for this body,
        the first velocity index in the velocity
        state variables for this body,
        joint damping value,
        joint friction coefficient,
        joint lower limit,
        joint upper limit,
        maximum force joint can sustain,
        maximum velocity joint can sustain,
        name string of associated link.
        """
        return NotImplemented
    # Controllers#######################################################
    @q.setter
    def q(self, qd):
        """
        Set joint position according to given values list. Note
        that the length of the list must match the number of joints.
        Using the set_position function (no underlying controller)
        :param qd: A list of length DOF of desired joint configurations
        """
        rospy.logdebug('setting q')
        assert len(qd) == len(self._joint_names), \
            'Input number of position values must match the number of joints'

        command = {self._joint_names[i]: qd[i] for i in range(len(self._joint_names))}
        self._limb.set_joint_position_speed(0.1)
        self._limb.set_joint_positions(command)

        rospy.logdebug('desired ee pose ' + str(self.prev_cmd))
        rospy.logdebug('current ee pose ' + str(self.ee_pose))

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

    @dq.setter
    def dq(self, dqd):
        """
        Set joint velocities according to given values list. Note that
        the length of the list must match that of the joint indices.
        :param dqd: A list of length DOF of desired joint velocities.
        """
        assert len(dqd) == len(self._joint_names), \
            'Input number of velocity values must match the number of joints'

        command = {self._joint_names[i]: dqd[i] for i in range(len(self._joint_names))}
        self._limb.set_joint_velocities(command)

    @tau.setter
    def tau(self, taud):
        """
        Set joint torques according to given values list. Note that the
        length of the list must match that of the joint indices.
        :param taud: A list of length DOF of desired torques.
        :return: None
        """
        assert len(taud) == len(self._joint_names), \
            'Input number of torque values must match the number of joints'

        command = {self._joint_names[i]: taud[i] for i in range(len(self._joint_names))}

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





    # GazeboSimCtrl #######################################################


    def update(self):
        """ update db parameters for the robot on a regular loop
        """
         # Set initial redis keys
        self.redisClient.set('robot::ee_position', str(self.ee_position))
        self.redisClient.set('robot::ee_pose', str(self.ee_pose))
        self.redisClient.set('robot::ee_orientation', str(self.ee_orientation))
        self.redisClient.set('robot::ee_v', str(self.ee_v))
        self.redisClient.set('robot::ee_omega', str(self.ee_omega))
        self.redisClient.set('robot::q', str(self.q))
        self.redisClient.set('robot::dq', str(self.dq))
        self.redisClient.set('robot::tau', str(self.tau))

        #self.redisClient.set('tip_pose',
        #    str(self._from_tip_to_base_of_gripper(self.ee_pose)))


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

    def js_trajectory(self, js_trajectory):
            """
            Plan and execute a trajectory in joint space following a set of
            waypoints
            :param js_trajectory: list of dictionaries. Each dictionary is a
            waypoint and contains 'time', joint space 'pose' [, 'velocity',
            'acceleration', operational space 'K_impedance', 'D_impedance',
            'force', 'impedance_flag']
            :return: None
            """
            num_waypoints = len(js_trajectory)
            dynamic_trajectory = False
            if any(dynamic_key in js_trajectory[0].keys() \
                for dynamic_key in ['K_impedance', 'D_impedance', 'force']):
                rospy.loginfo("Dynamic properties defined for the trajectory")
                dynamic_trajectory = True
                self._interaction_options = InteractionOptions()
                self._interaction_options.set_interaction_control_active(True)
                self._interaction_options.set_max_impedance(6*[False])
                # 'False' if the impedance/force is defined in base frame
                # 'True' if the impedance/force is defined in ee frame
                self._interaction_options.set_in_endpoint_frame(True)
                rospy.loginfo("Activate interaction control")
                rospy.sleep(1.0)
            else:
                # In case it was active before, we deactivate interaction control by sending the
                # appropriate message and waiting for the reception
                self._interaction_options.set_interaction_control_active(False)
                self._interaction_options_pub.publish(self._interaction_options.to_msg())
                rospy.loginfo("Deactivate interaction control")
                rospy.sleep(1.0)

            v_a_provided = False
            if any(v_a_key in js_trajectory[0].keys() for v_a_key in ['velocity', 'acceleration']):
                if len(js_trajectory[0]['velocity']) == len(self.q) \
                    and len(js_trajectory[0]['acceleration']) == len(self.q):
                    rospy.loginfo("Velocity and/or acceleration provided for the trajectory")
                    v_a_provided = True

            if not v_a_provided:  #  Plan to have smooth motions
                rospy.loginfo("Velocity and/or acceleration NOT provided for the trajectory")
                rospy.logerr("TODO: interpolate the joint poses to obtain smooth velocities " +
                             " and accelerations given the constraints of the robot")

            rospy.logdebug("Number of points in js trajectory: " + str((len(js_trajectory))))

            for index, waypoint in enumerate(js_trajectory):
                if v_a_provided:
                    self.q_dq_ddq([waypoint['pose'],
                                   waypoint['velocity'],
                                   waypoint['acceleration']])
                else:
                    self.goto_q(waypoint['pose'])

                if dynamic_trajectory:
                    self._interaction_options.set_K_impedance(waypoint['K_impedance'])
                    self._interaction_options._data.D_impedance = \
                        copy.deepcopy((waypoint['D_impedance']))
                    self._interaction_options.set_force_command(waypoint['force'])
                    IMPEDANCE_MODE = 1
                    FORCE_MODE = 2
                    IMPEDANCE_WITH_FORCE_LIMIT_MODE = 3
                    FORCE_WITH_MOTION_LIMIT_MODE = 4
                    interaction_cm = \
                        [[FORCE_WITH_MOTION_LIMIT_MODE, IMPEDANCE_MODE][int(impedance_flag)]
                        for impedance_flag in waypoint['impedance_flag']]
                    self._interaction_options.set_interaction_control_mode(interaction_cm)
                    self._interaction_options_pub.publish(self._interaction_options.to_msg())

                if index+1 < len(js_trajectory):
                    rospy.sleep(js_trajectory[index+1]['time']-waypoint['time'])


    def os_trajectory(self, os_trajectory):
        """
        Plan and execute a trajectory in operational space following a
        set of waypoints
        :param os_trajectory: list of dictionaries. Each dictionary is a
        waypoint and contains 'time', joint space 'pose' [, 'velocity',
        'acceleration', operational space 'K_impedance', 'D_impedance',
        'force', 'impedance_flag']
        :return: None
        """
        num_waypoints = len(os_trajectory)
        dynamic_trajectory = False
        if any(dynamic_key in os_trajectory[0].keys() \
            for dynamic_key in ['K_impedance', 'D_impedance', 'force']):
            dynamic_trajectory = True

        if any(v_a_key in os_trajectory[0].keys() for v_a_key in ['velocity', 'acceleration']):
            rospy.logerr("Velocity and/or acceleration cannot be provided for an os trajectory!")
            return False

        js_trajectory = self._plan_os_trajectory(os_trajectory, dynamic_trajectory)

        if len(js_trajectory) == 0:
            rospy.logerr("No points in the joint space trajectory plan!")
            return False
        else:
            self.js_trajectory(js_trajectory)

    def goto_ee_pose_and_grasp(self, x, y, z, qx = 0, qy = 1, qz = 0,
                               qw = 0, hover = 0.4, dive=0.05):
        """
        Move to given pose, advance the gripper in +z and grasp
        :param x: refer to <goto_ee_pose::xd>
        :param y: refer to <goto_ee_pose::xd>
        :param z: refer to <goto_ee_pose::xd>
        :param qx: refer to <goto_ee_pose::xd>
        :param qy: refer to <goto_ee_pose::xd>
        :param qz: refer to <goto_ee_pose::xd>
        :param qw: refer to <goto_ee_pose::xd>
        :param hover: the distance above object before grasping
        :param dive: the distance to go down before slowing down motion
        :return: None
        """
        self.open_gripper()

        self.set_max_speed(0.15)
        des_pose = [x,y,z+hover,qx,qy,qz,qw]
        des_pose = self.pose_endpoint_transform(des_pose)
        self.goto_ee_pose(des_pose, traj_type='cartesian')

        rospy.sleep(0.7)

        des_pose = [x,y,z+dive,qx,qy,qz,qw]
        # send fingertips to pose specified instead of j6
        des_pose = self.pose_endpoint_transform(des_pose)
        self.goto_ee_pose(des_pose, traj_type='cartesian')

        self.set_max_speed(0.05)
        des_pose = [x,y,z,qx,qy,qz,qw]
        # send fingertips to pose specified instead of j6
        des_pose = self.pose_endpoint_transform(des_pose)
        self.goto_ee_pose(des_pose, traj_type='cartesian')

        rospy.sleep(0.8)

        self.close_gripper()

    def goto_ee_pose_and_grasp_and_lift(self, x, y, z, qx = 0, qy = 1,
                                        qz = 0, qw = 0, hover = 0.4,
                                        dive=0.05, lift_height=0.3,
                                        drop=True):
        """
        Move to given position, grasp the object, and lift up the
        object.
        :param x: refer to <goto_ee_pose::xd>
        :param y: refer to <goto_ee_pose::xd>
        :param z: refer to <goto_ee_pose::xd>
        :param qx: refer to <goto_ee_pose::xd>
        :param qy: refer to <goto_ee_pose::xd>
        :param qz: refer to <goto_ee_pose::xd>
        :param qw: refer to <goto_ee_pose::xd>
        :param hover: the distance above object before grasping
        :param dive: the distance to go down before slowing down motion
        :param lift_height: the height to lift the object
        :param drop: boolean whether drop the object after lift
        :return: None
        """
        self.goto_ee_pose_and_grasp(x,y,z,qx,qy,qz,qw,hover=hover,dive=dive)

        des_pose = [x,y,z-lift_height,qx,qy,qz,qw]
        # send fingertips to pose specified instead of j6
        des_pose = self.pose_endpoint_transform(des_pose)
        self.goto_ee_pose(des_pose, traj_type='cartesian')

        time.sleep(.8)

        if drop:
            self.open_gripper()
            time.sleep(.5)

    # Sawyer specific###################################################
    @property
    def head_pan(self):
        """
        Get current pan angle of the head
        :return: float radian
        """
        return self._head.pan()

    @head_pan.setter
    def head_pan(self, angle_speed_rel):
        """
        Pan at given speed to desired angle
        :param angle: float radian
        :param speed: float speed, 0 - 1
        :param rel: if True, pan wrt base frame
        :return: None
        """
        angle, speed, rel = angle_speed_rel
        self._head.set_pan(angle, speed=speed, active_cancellation=rel)

    def activate_interaction_mode(self):
        """
        Activates the interaction mode of the Sawyer for force/impedance
        control.
        It also resets the impedance/force values to the default
        :return: None
        """
        rospy.loginfo("Dynamic properties defined for the trajectory")
        dynamic_trajectory = True
        self._interaction_options = InteractionOptions()
        self._interaction_options.set_interaction_control_active(True)
        self._interaction_options.set_max_impedance(6*[False])
        # 'False' if the impedance/force is defined in base frame
        # 'True' if the impedance/force is defined in ee frame
        self._interaction_options.set_in_endpoint_frame(True)
        rospy.loginfo("Activate interaction control")
        rospy.sleep(1.0)

    def deactivate_interaction_mode(self):
        """
        Deactivates the interaction mode of the Sawyer for
        force/impedance control
        :return: None
        """
        self._interaction_options.set_interaction_control_active(False)
        self._interaction_options_pub.publish(
            self._interaction_options.to_msg())
        rospy.loginfo("Deactivate interaction control")
        rospy.sleep(1.0)

    def set_timeout(self, time):
        """
        Set the timeout in seconds for the joint controller
        :param timeout: float timeout in seconds
        """
        self._limb.set_command_timeout(time)

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

    # Auxiliary functions###############################################

    def _plan_os_trajectory(self, os_trajectory, dynamic_trajectory):
        """
        Calls the operational space planner and interpolates the dynamic
        parameters for each point of the trajectory
        :param os_trajectory: operational space trajectory
        :param dynamic_trajectory: associated dynamic parameters (force
        and impedance control) at each os waypoint
        :return: True if successful, False if failure
        """
        js_trajectory = []
        pose_curr = Pose()
        xx_curr = self.ee_pose
        pose_curr.position.x = xx_curr[0]
        pose_curr.position.y = xx_curr[1]
        pose_curr.position.z = xx_curr[2]
        pose_curr.orientation.x = xx_curr[3]
        pose_curr.orientation.y = xx_curr[4]
        pose_curr.orientation.z = xx_curr[5]
        pose_curr.orientation.w = xx_curr[6]
        pose_next = Pose()
        interaction_options_curr = self._interaction_options
        interaction_options_next = InteractionOptions()
        last_q = None

        eef_step = 0.001 #  Step for the interpolation in Cartesian space (only positions!)

        # pieces TRUE:
        #   compute the trajectory between each pair of OS waypoints, interpolate dynamics and
        #   merge the result
        #   + Exact count of trajectory points between pairs of OS waypoints
        #   - Velocity and acceleration go to zero at each waypoint (not smooth)
        # pieces FALSE:
        #   compute the trajectory for the entire set OS waypoints, interpolate dynamics using an
        #   estimation of the number of points in the trajectory
        #   + Smooth trajectory
        #   - Approximate number of points between pairs of waypoints
        pieces = False
        if pieces:
            # Starting with the current pose, take the next pose,
            # compute the path, interpolate the dyn params
            for waypoint in os_trajectory:
                waypoints = []
                xx_next = waypoint['pose']
                pose_next.position.x = xx_next[0]
                pose_next.position.y = xx_next[1]
                pose_next.position.z = xx_next[2]
                pose_next.orientation.x = xx_next[3]
                pose_next.orientation.y = xx_next[4]
                pose_next.orientation.z = xx_next[5]
                pose_next.orientation.w = xx_next[6]
                waypoints.append(copy.deepcopy(pose_curr))
                waypoints.append(copy.deepcopy(pose_next))
                # Plan the Cartesian path connecting the waypoints
                maxtries = 100
                attempts = 0
                fraction = 0
                plan = None
                print_msg = "Planning trajectory between {} and {}".format(waypoints[0],
                                                                           waypoints[1])
                rospy.loginfo(print_msg)
                while fraction < 1.0 and attempts < maxtries:
                    (plan, fraction) = self._call_os_planner (
                                            waypoints,  # waypoint poses
                                            eef_step,   # eef_step
                                            0.0,        # jump_threshold
                                            False,      # avoid_collisions
                                            last_q)     # start state for the planning (joints)

                    # Increment the number of attempts
                    attempts += 1

                str_msg = "Num points between two points: {}".format(
                    len(plan.joint_trajectory.points))
                rospy.loginfo(str_msg)

                if dynamic_trajectory:
                    plan_len = len(plan.joint_trajectory)

                    # Set the interaction values (impedance, forces, selection flag) for the next
                    # waypoint
                    interaction_options_next.set_K_impedance(waypoint['K_impedance'])
                    interaction_options_next._data.D_impedance = \
                        copy.deepcopy((waypoint['D_impedance']))
                    interaction_options_next.set_force_command(waypoint['force'])
                    IMPEDANCE_MODE = 1
                    FORCE_MODE = 2
                    IMPEDANCE_WITH_FORCE_LIMIT_MODE = 3
                    FORCE_WITH_MOTION_LIMIT_MODE = 4
                    interaction_cm = [[FORCE_WITH_MOTION_LIMIT_MODE,
                                      IMPEDANCE_WITH_FORCE_LIMIT_MODE]
                                      [int(impedance_flag)] for impedance_flag \
                                      in js_trajectory['impedance_flag']]
                    interaction_options_next.set_interaction_control_mode(interaction_cm)

                    # Interpolate between the interaction values of the current waypoint and the
                    # next waypoint
                    for point, k, d, f in zip(plan.joint_trajectory.points,
                                              self._array_linspace(
                                                interaction_options_curr._data.K_impedance,
                                                interaction_options_next._data.K_impedance,
                                                plan_len),
                                              self._array_linspace(
                                                interaction_options_curr._data.D_impedance,
                                                interaction_options_next._data.D_impedance,
                                                plan_len),
                                              self._array_linspace(
                                                interaction_options_curr._data.force_command,
                                                interaction_options_next._data.force_command,
                                                plan_len)):
                        js_waypoint = {'time': point.time_from_start,
                                       'pose': point.positions,
                                       'velocity': point.velocities,
                                       'acceleration': point.accelerations,
                                       'K_impedance': k,
                                       'D_impedance': d,
                                       'force': f}
                        js_trajectory += [js_waypoint]
                        interaction_options_curr = copy.deepcopy(interaction_options_next)
                else:
                    for point in plan.joint_trajectory.points :
                        js_waypoint = {'time': point.time_from_start,
                                       'pose': point.positions,
                                       #'velocity': point.velocities,
                                       #'acceleration': point.accelerations
                                       }
                        js_trajectory += [js_waypoint]
                pose_curr = copy.deepcopy(pose_next)
                last_q = plan.joint_trajectory.points[-1].positions

        else:
            waypoints = []
            waypoints.append(copy.deepcopy(pose_curr))

            expected_steps = []

            # Starting with the current pose, take the next pose, compute the path,
            # interpolate the dyn params
            for waypoint in os_trajectory:

                xx_next = waypoint['pose']
                pose_next.position.x, pose_next.position.y, pose_next.position.z = xx_next[0:3]
                pose_next.orientation.x = xx_next[3]
                pose_next.orientation.y = xx_next[4]
                pose_next.orientation.z = xx_next[5]
                pose_next.orientation.w = xx_next[6]

                # Approximation of the number of trajectory points between a pair of waypoints
                expected_steps += \
                    [np.max(
                        [int(self._distance_between_position_msgs(pose_curr.position,
                                                                  pose_next.position)/eef_step + 1),
                        3])]

                waypoints.append(copy.deepcopy(pose_next))
                pose_curr = copy.deepcopy(pose_next)

            # Plan the Cartesian path connecting the waypoints
            maxtries = 100
            attempts = 0
            fraction = 0

            plan = None
            print_msg = "Planning trajectory between "
            for wp in waypoints:
                print_msg += "{}".format(wp)

            rospy.loginfo(print_msg)

            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = self._call_os_planner (waypoints,   # waypoint poses
                                                          eef_step,    # eef_step
                                                          0.0,         # jump_threshold
                                                          False,       # avoid_collisions
                                                          last_q)

                # Increment the number of attempts
                attempts += 1

            rospy.loginfo("Num points: " + str(len(plan.joint_trajectory.points)))
            rospy.loginfo("Predicted number of points: " + str(np.sum(expected_steps)))

            # Hack! Because the number of steps is just an approximation, we fix it by
            #removing/adding some from the last interval
            # This will be fine if the interpolation uses small steps (around 1mm) because the
            # impedance parameters won't change much then
            expected_steps[-1] -= np.sum(expected_steps) - len(plan.joint_trajectory.points)

            rospy.loginfo("Adjusted number of points: " + str(np.sum(expected_steps)))

            # Because we don't know hte exact number of points between waypoints in the final
            # trajectory we approximate them using the step size and the distance between waypoints
            current_idx = 0
            if dynamic_trajectory:
                for wp_idx, waypoint in enumerate(os_trajectory):
                    # Set the interaction values (impedance, forces, selection flag) for the next
                    # waypoint
                    interaction_options_next.set_K_impedance(waypoint['K_impedance'])
                    interaction_options_next._data.D_impedance = \
                        copy.deepcopy((waypoint['D_impedance']))
                    interaction_options_next.set_force_command(waypoint['force'])
                    IMPEDANCE_MODE = 1
                    FORCE_MODE = 2
                    IMPEDANCE_WITH_FORCE_LIMIT_MODE = 3
                    FORCE_WITH_MOTION_LIMIT_MODE = 4
                    interaction_cm = \
                        [[FORCE_WITH_MOTION_LIMIT_MODE, IMPEDANCE_WITH_FORCE_LIMIT_MODE]
                        [int(impedance_flag)] for impedance_flag in waypoint['impedance_flag']]
                    interaction_options_next.set_interaction_control_mode(interaction_cm)

                    # Interpolate between the interaction values of the current waypoint and the
                    # next waypoint
                    for point, k, d, f in zip(plan.joint_trajectory.points[current_idx:current_idx +
                                                                            expected_steps[wp_idx]],
                                              self._array_linspace(
                                                interaction_options_curr._data.K_impedance,
                                                interaction_options_next._data.K_impedance,
                                                expected_steps[wp_idx]),
                                              self._array_linspace(
                                                interaction_options_curr._data.D_impedance,
                                                interaction_options_next._data.D_impedance,
                                                expected_steps[wp_idx]),
                                              self._array_linspace(
                                                interaction_options_curr._data.force_command,
                                                interaction_options_next._data.force_command,
                                                expected_steps[wp_idx])):

                        js_waypoint = {'time': point.time_from_start,
                                       'pose': point.positions,
                                       'velocity': point.velocities,
                                       'acceleration': point.accelerations,
                                       'K_impedance': k,
                                       'D_impedance': d,
                                       'force': f,
                                       'impedance_flag': waypoint['impedance_flag']}
                        js_trajectory += [js_waypoint]
                        interaction_options_curr = copy.deepcopy(interaction_options_next)

                    current_idx = np.sum(expected_steps[0:wp_idx+1]) - 1
            else:
                for point in plan.joint_trajectory.points :
                    js_waypoint = {'time': point.time_from_start,
                                   'pose': point.positions,
                                   'velocity': point.velocities,
                                   'acceleration': point.accelerations}
                    js_trajectory += [js_waypoint]

        return js_trajectory

    def _call_os_planner(self, waypoints, step, jump_threshold, avoid_collisions, last_q):
        """
        Calls the operations space motion planner
        :param waypoints: the trajectory waypoints (operational space configurations)
        :param step: the interpolation step in operational space (meters)
        :param jump_threshold: the allowed jump in joint space for consecutive solutions
        :param avoid_collisions: if we try to avoid collisions
        :param last_q: last joint configuration of the robot
        :return: Pair of OS path and fraction of the trajectory that can be completed
        """
        req2 = GetCartesianPathRequest()
        req = dict()

        req["group_name"] = 'right_arm'
        header = Header()
        header.frame_id = 'world'
        req["link_name"] = 'right_hand'
        req["header.stamp"] = rospy.Time.now()
        req["waypoints"] = waypoints
        req["max_step"] = step
        req["jump_threshold"] = jump_threshold
        req["avoid_collisions"] = avoid_collisions
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.q_names
        if last_q == None:
            joint_state.position = self.q
        else:
            joint_state.position = last_q
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        req["start_state"] = moveit_robot_state

        res = self._cartesian_path_service(header,
                                           req["start_state"],
                                           req["group_name"],
                                           req["link_name"],
                                           req["waypoints"],
                                           req["max_step"],
                                           req["jump_threshold"],
                                           req["avoid_collisions"],
                                           req2.path_constraints)

        if res:
          error_code = res.error_code
          if res.error_code.val == MoveItErrorCodes.SUCCESS:
            return (res.solution, res.fraction)
          else:
            return (res.solution,-1.0)
        else:
          error_code.val =  MoveItErrorCodes.FAILURE
          return (res.solution,-1.0)

    def _call_js_planner(self, waypoints, last_q=None):
        """
        Calls the joint space motion planner
        :param waypoints: the trajectory waypoints (joint space configurations)
        :param last_q: last joint configuration of the robot
        :return: JS path
        """
        req_msg = MotionPlanRequest()

        req = dict()
        req_msg.group_name = 'right_arm'
        header = Header()
        header.frame_id = 'world'
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.q_names
        if last_q == None:
            joint_state.position = self.q
        else:
            joint_state.position = last_q
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state
        req_msg.start_state = moveit_robot_state

        req_msg.num_planning_attempts = 1000
        req_msg.max_velocity_scaling_factor = 1
        req_msg.max_acceleration_scaling_factor = 1
        req_msg.allowed_planning_time = 30
        #req_msg.planner_id = "OMPL"
        wsp = WorkspaceParameters()
        wsp.header = Header()
        wsp.header.stamp = rospy.Time.now()
        # TODO: set the min_corner and max_corner in the wsp
        # (setting up the volume (a box) in which the robot is allowed to move.
        # This is useful only when planning for mobile parts of
        # the robot as well.)
        wsp.min_corner.x, wsp.min_corner.y, wsp.min_corner.z = -10, -10, -10
        wsp.max_corner.x, wsp.max_corner.y, wsp.max_corner.z = 10, 10, 10

        req_msg.workspace_parameters = wsp
        req_msg.path_constraints = Constraints()

        #  Here we add the goal (last element of the trajectory)
        goal_constraints = []
        goal_constraint = Constraints()
        goal_constraint.name = "js_goal"
        goal_constraint.joint_constraints = []
        tolerance = 0.0001
        for q_name, qd in zip(self.q_names, waypoints[-1]):
            js_goal_constraint = JointConstraint()
            js_goal_constraint.joint_name = q_name
            js_goal_constraint.position = qd
            js_goal_constraint.tolerance_above = tolerance
            js_goal_constraint.tolerance_below = tolerance
            js_goal_constraint.weight = 1
            goal_constraint.joint_constraints += [js_goal_constraint]
        goal_constraints += [goal_constraint]
        req_msg.goal_constraints = goal_constraints

        #  Here we add the via points
        traj_constraints = TrajectoryConstraints()
        for wp_idx, waypoint in enumerate(waypoints[:-1]):
            traj_constraint = Constraints()
            traj_constraint.name = "qd_" + str(wp_idx)
            traj_constraint.joint_constraints = []
            for q_name, qd in zip(self.q_names, waypoint):
                js_goal_constraint = JointConstraint()
                js_goal_constraint.joint_name = q_name
                js_goal_constraint.position = qd
                js_goal_constraint.tolerance_above = tolerance
                js_goal_constraint.tolerance_below = tolerance
                js_goal_constraint.weight = 1
                traj_constraint.joint_constraints += [js_goal_constraint]
            traj_constraints.constraints += [traj_constraint]
        req_msg.trajectory_constraints = traj_constraints

        res = self._js_path_service(req_msg)

        return res

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


    def _array_linspace(self, orig, dest, num_points):
        """
        Creates a number of equally spaced vectors between the vectors origin and destination
        :param orig: origin vector
        :param dest: destination vector
        :param num_points: number of equally spaced vectors
        :return: list of lists with the equally spaced vectors
        """
        mmstep = (np.array(dest) - np.array(orig))/float(num_points - 1)
        ret = []
        for ctr in range(num_points):
            ret += [orig + ctr*mmstep]
        return ret

    def print_pose_message(self, pose_msg):
        """
        Prints a ROS Pose message
        :param pose_msg: the pose message to print
        :return: None
        """
        print_msg = "%f, %f, %f, %f, %f, %f, %f" %(pose_msg.position.x, pose_msg.position.y,
                                                   pose_msg.position.z, pose_msg.orientation.x,
                                                   pose_msg.orientation.y, pose_msg.orientation.z,
                                                   pose_msg.orientation.w)
        rospy.loginfo(print_msg)


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
        return self.redisClient.get('robot::env_connected')

    @property
    def cmd_type(self):
        """ Redis key identifying the type of command robot should execute
        """
        return self.redisClient.get('robot::cmd_type')

    @property
    def desired_ee_pose(self):
        return bstr_to_ndarray(self.redisClient.get('robot::desired_ee_pose'))

    @property
    def qd(self):
        return bstr_to_ndarray(self.redisClient.get('robot::qd'))

    @property
    def desired_torque(self):
        return bstr_to_ndarray(self.redisClient.get('robot::tau_desired'))
    

    def process_cmd(self):
        if (self.cmd_type == b'ee_pose'):
            rospy.loginfo('des_pose ' + str(self.desired_ee_pose))
            #rospy.loginfo('prev Cmd ' + str(ctrlInterface.prev_cmd))
            #if not np.array_equal(ctrlInterface.prev_cmd,desired_pose):
            # TODO: get rid of all prev_cmd for debugging
            self.prev_cmd = self.desired_ee_pose
            self.ee_pose = self.desired_ee_pose
        elif(self.cmd_type == b'joint_position'):
            self.q = self.qd
        elif (self.cmd_type == b'torque'):
            self.tau = self.desired_torque
        elif(self.cmd_type == b'reset_to_neutral'):
            self.reset_to_neutral()
        else:
            rospy.logwarn('Unknown command')


### MAIN ###
if __name__ == "__main__":
    # Create ros node
    rospy.init_node("sawyer_interface", log_level=rospy.INFO)
    #load_gazebo_models()
    ctrlInterface = SawyerCtrlInterface(use_safenet=False, use_moveit=False)
    # Control Loop
    threshold = 0.05
    rospy.loginfo("waiting for environment ... ")
    while (ctrlInterface.env_connected != b'True'):
        # do nothing until the environment is connected
        pass

    #if (ctrlInterface.redisClient.get('env_connected') == b'True'):


    rospy.loginfo('Running control loop')

    while(ctrlInterface.env_connected == b'True'):
        rospy.logdebug('Environment connected')
        start = time.time()
        ctrlInterface.process_cmd()
        ctrlInterface.update()

        while ((time.time() - start) < 0.001):
            pass





