"""
Abstract class defining the interface to the robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions
from tq_control.controllers.ee_imp import EEImpController
from tq_control.robot_model.manual_model import ManualModel
#from tq_control.interpolator.reflexxes_interpolator import ReflexxesInterpolator
import numpy as np

@six.add_metaclass(abc.ABCMeta)
class RobotInterface(object):
    """Abstract interface to be implemented for each real and simulated
    robot.

    Attributes:
        control_type (str): Type of controller for robot to use
            e.g. IK, OSC, Joint Velocity
        model (ManualModel): a model of the robot state as defined by tq_control.
        controller (Controller): tq_control object that takes robot states and compute torques.

    """

    def __init__(self,
                 controlType='EEImp'):
        """
        Initialize variables

        Args:
            control_type (str): Type of controller for robot to use
                e.g. IK, OSC, Joint Velocity
            model (ManualModel): a model of the robot state as defined by tq_control.
            controller (Controller): tq_control object that takes robot states and compute torques.

        :TODO:
            * controller type not currently supported
        """
        self.controlType = controlType
        print(controlType)
        self.model = ManualModel()
        self.update()
        if self.controlType == 'EEImp':
            self.controller = EEImpController(self.model,
                kp=200, damping=1,
                interpolator_pos =None,
                interpolator_ori=None,
                control_freq=self.config['sim_params']['steps_per_action'])
        self.action_set = False

    def update(self):
        self.model.update_states(ee_pos=np.asarray(self.ee_position),
                                 ee_ori=np.asarray(self.ee_orientation),
                                 ee_pos_vel=np.asarray(self.ee_v),
                                 ee_ori_vel=np.asarray(self.ee_w),
                                 joint_pos=np.asarray(self.motor_joint_positions),
                                 joint_vel=np.asarray(self.motor_joint_velocities))

        self.model.update_model(J_pos=self.linear_jacobian,
                                J_ori=self.angular_jacobian,
                                mass_matrix=self.mass_matrix)

    def step(self):
        """Update the robot state and model, set torques from controller
        """
        self.update()
        if self.action_set:
            torques = self.controller.run_controller() + self.N_q
            self.set_torques(torques)


    def move_ee_delta(self, delta):
        self.controller.set_goal(delta)


    @abc.abstractmethod
    def create(config):
        """Factory for creating robot interfaces based on config type
        """
        raise NotImplementedError

    @property
    def base_pose(self):
        return self.pose

    @property
    @abc.abstractmethod
    def version(self):
        """dict of current versions of robot SDK, gripper, and robot
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def name(self):
        """str of the name of the robot
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def ee_position(self):
        """list of three floats [x, y, z] of the position of the
        end-effector.
        """
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def ee_orientation(self):
        """list of four floats [qx, qy, qz, qw] of the orientation
        quaternion of the end-effector.
        """
        raise NotImplementedError


    @property
    @abc.abstractmethod
    def ee_pose(self):
        """list of seven floats [x, y, z, qx, qy, qz, qw] of the 6D pose
        of the end effector.
        """
        raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def ee_v(self):
    #     """List of three floats [vx, vy, vz] of the current linear
    #     velocity of end effector in the ee frame.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def ee_omega(self):
    #     """List of three floats [wx, wy, wz] of the current angular
    #     velocity of end effector in the ee frame.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def ee_twist(self):
    #     """List of six floats [vx, vy, vz, wx, wy, wz] of the current
    #     twist (linear and angular) velocity of end effector in the ee
    #     frame.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def ee_force(self):
    #     """List of three floats [fx, fy, fz] of the current force
    #     applied by the end effector in ee frame.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def ee_torque(self):
    #     """List of three floats [mx, my, mz] of the current torque
    #     applied by the end effector in ee frame.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def ee_wrench(self):
    #     """List of six floats [fx, fy, fz, mx, my, mz] of the current
    #     wrench applied by the end effector in ee frame.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def q_names(self):
    #     """List of the names of the joints ordered by indices typically
    #     growing from base to end effector.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def q(self):
    #     """List of floats of the joint configuration of the robot arm.
    #     Typically the order goes from base to end effector.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def dq(self):
    #     """List of floats of the joint velocities of the robot arm.
    #     Typically the order goes from base to end effector.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def ddq(self):
    #     """List of floats of the joint accelerations of the robot arm.
    #     Typically the order goes from base to end effector.
    #     """
    #     raise NotImplementedError

    # @property
    # @abc.abstractmethod
    # def tau(self):
    #     """List of floats of the joint torques of the robot arm.
    #     Typically the order goes from base to end effector.
    #     """
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def J(self, q=None):
    #     """Compute and return a 2D numpy array of the Kinematic Jacobian
    #     at the robot configuration q (current configuration if q is
    #     None).

    #     Parameters
    #     ----------
    #     q
    #         robot configuration where the Jacobian is computed (Default
    #          = None)

    #     Returns
    #     -------
    #     2D numpy array
    #         Kinematic Jacobian matrix at the given robot configuration
    #     """
    #     raise NotImplementedError

    # @property
    # def info(self):
    #     """Additional parameters of the robot
    #     """
    #     raise NotImplementedError

    # @info.setter
    # def info(self, new_params):
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def get_link_name(self, uid, lid):
    #     """Get the name string of given link

    #     Parameters
    #     ----------
    #     uid
    #         integer body unique id
    #     lid
    #         integer link id of body

    #     Returns
    #     -------
    #     str
    #         name string of link on body
    #     """
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def get_link_state(self, lid):
    #     """Get the state of given link on given body

    #     Parameters
    #     ----------
    #     uid
    #         integer body unique id
    #     lid
    #         integer link id on body

    #     Returns
    #     -------
    #     tuple
    #         link world position (list of floats [x, y, z]),
    #         link world orientation (list of floats [qx, qy, qz, qw]),
    #         local position offset of CoM frame,
    #         local orientation offset of CoM frame,
    #         world position of asset file link frame,
    #         world orientation of asset file link frame,
    #         link world linear velocity,
    #         link world angular velocity.

    #     """
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def get_joint_info(self, jid):
    #     """Get the information of body joint

    #     Parameters
    #     ----------
    #     uid
    #         integer unique body id
    #     jid
    #         integer joint id on body

    #     Returns
    #     -------
    #     tuple
    #         joint index,
    #         joint name string in asset file,
    #         joint type string ('fixed', 'revolute', 'prismatic',
    #         'spherical', 'planar', 'gear', 'point2point'),
    #         the first position index in the positional state variables
    #         for this body,
    #         the first velocity index in the velocity state variables
    #         for this body,
    #         joint damping value,
    #         joint friction coefficient,
    #         joint lower limit,
    #         joint upper limit,
    #         maximum force joint can sustain,
    #         maximum velocity joint can sustain,
    #         name string of associated link.

    #     """
    #     raise NotImplementedError

    # @property
    # def gripper_force(self):
    #     """Force in the gripper when closing.
    #     """
    #     raise NotImplementedError

    # @gripper_force.setter
    # def gripper_force(self, force):
    #     raise NotImplementedError

    # def close_gripper(self):
    #     """Close the gripper of the robot
    #     """
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def open_gripper(self):
    #     """Open the gripper of the robot
    #     """
    #     raise NotImplementedError

    # @property
    # def gripper_opening(self):
    #     """float: Current opening distance of the gripper"""
    #     raise NotImplementedError

    # @gripper_opening.setter
    # def gripper_opening(self, opening):
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def pause(self):
    #     """Pause the action and disable all motors.
    #     Call <start> to resume.
    #     """
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def start(self):
    #     """Start the robot
    #     """
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def reset(self, reboot=False):
    #     """Reset the robot and move to rest pose.
    #     Parameters
    #     ----------
    #     reboot
    #          (Default value = False)
    #     """
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def stop(self):
    #     """Act as E-stop. Must reset to clear
    #     the stopped state
    #     """
    #     raise NotImplementedError

    # @staticmethod
    # def shutdown():
    #     """Safely shut down the robot controller
    #     """
    #     raise NotImplementedError

    # # Controller interfaces ############################################

    # @q.setter
    # @abc.abstractmethod
    # def q(self, qd):
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def goto_q(self, qd, max_joint_speed_ratio=0.3, timeout=15.0,
    #            threshold=0.008726646):
    #     """Blocking call.
    #     Set joint position according to given values list. Note that the
    #     length of the list must match the number of joints.

    #     Parameters
    #     ----------
    #     qd
    #         A list of length DOF of desired joint configurations
    #     max_joint_speed_ratio
    #         ratio of the maximum joint velocity that the motion should
    #         be executed at (Default value = 0.3)
    #     timeout
    #         seconds to wait for move to finish (Default value = 15.0)
    #     threshold
    #         position threshold in radians across each joint when move is
    #         considered successful (Default value = 0.008726646)
    #     """
    #     raise NotImplementedError

    # @dq.setter
    # @abc.abstractmethod
    # def dq(self, dqd):
    #     raise NotImplementedError

    # @tau.setter
    # @abc.abstractmethod
    # def tau(self, taud):
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def q_dq_ddq(self, value):
    #     """Set joint position, velocity and acceleration

    #     Parameters
    #     ----------
    #     value
    #         A list of lists of length DOF with desired position,
    #         velocity and acceleration for each joint.
    #     """
    #     raise NotImplementedError

    # @ee_pose.setter
    # @abc.abstractmethod
    # def ee_pose(self, xd):
    #     """Set the end effector pose.
    #     Use this function for poses that are close to the current pose.
    #     There are two options for the implementation of this function:
    #     1) Operational space control.
    #     2) IK and set q or goto_q (only if the goal is close) -> the
    #     motion "interpolates" in joint space!

    #     Parameters
    #     ----------
    #     xd
    #         list of desired position, orientation values [x, y, z, qx,
    #         qy, qz, qw]
    #     """
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def goto_ee_pose(self, xd):
    #     """Set the end effector pose.
    #     Use this funtion for larger distances to current pose.
    #     The function interpolates in cartesian the shortest path to
    #     xd and plans the sequence joint configurations to execute it ->
    #     -> the motion "interpolates" in Cartesian space even though
    #     follows a joint space trajectory

    #     Parameters
    #     ----------
    #     xd
    #         list of desired position, orientation values [x, y, z, qx,
    #         qy, qz, qw]
    #     """
    #     raise NotImplementedError

    # @ee_position.setter
    # @abc.abstractmethod
    # def ee_position(self, position):
    #     raise NotImplementedError

    # @ee_twist.setter
    # @abc.abstractmethod
    # def ee_twist(self, twist):
    #     raise NotImplementedError

    # @ee_v.setter
    # @abc.abstractmethod
    # def ee_v(self, linear_vel):
    #     raise NotImplementedError

    # @ee_omega.setter
    # @abc.abstractmethod
    # def ee_omega(self, angular_vel):
    #     raise NotImplementedError

    # # Trajectory interfaces#############################################

    # @abc.abstractmethod
    # def js_trajectory(self, js_trajectory):
    #     """Plan and execute a trajectory in joint space following a set
    #     of waypoints

    #     Parameters
    #     ----------
    #     js_trajectory
    #         list of dictionaries. Each dictionary is a waypoint and
    #         contains 'time', joint space 'pose' [, 'velocity',
    #         'acceleration', operational space 'K_impedance',
    #         'D_impedance', 'force', 'impedance_flag']
    #     """
    #     raise NotImplementedError

    # @abc.abstractmethod
    # def os_trajectory(self, os_trajectory):
    #     """Plan and execute a trajectory in operational space following
    #     a set of waypoints

    #     Parameters
    #     ----------
    #     os_trajectory
    #         list of dictionaries. Each dictionary is a waypoint and
    #         contains 'time', joint space 'pose' [, 'velocity',
    #         'acceleration', operational space 'K_impedance',
    #         'D_impedance', 'force', 'impedance_flag']
    #     """
    #     raise NotImplementedError

    # # More "complex" behaviors #########################################

    # def goto_ee_pose_and_grasp(self, x, y, z, qx=0, qy=1, qz=0, qw=0,
    #                            hover=0.4, dive=0.05):
    #     """Move to given pose of the gripper at a hover distance in the
    #     direction of the palm, advance until a dive distance to the
    #     given pose, reduce velocity, advance to the given pose and grasp

    #     Parameters
    #     ----------
    #     x
    #         refer to <goto_ee_pose::x>
    #     y
    #         refer to <goto_ee_pose::y>
    #     z
    #         refer to <goto_ee_pose::z>
    #     qx :
    #         refer to <goto_ee_pose::xd> (Default value = 0)
    #     qy :
    #         refer to <goto_ee_pose::xd> (Default value = 1)
    #     qz :
    #         refer to <goto_ee_pose::xd> (Default value = 0)
    #     qw :
    #         refer to <goto_ee_pose::xd> (Default value = 0)
    #     hover
    #         the distance above object before grasping (Default value =
    #         0.4)
    #     dive
    #         the distance to the object before slowing down motion
    #         (Default value = 0.05)
    #     """
    #     raise NotImplementedError

    # def goto_ee_pose_and_grasp_and_lift(self, x, y, z, qx=0, qy=1,
    #                                     qz=0, qw=0, hover=0.4,
    #                                     dive=0.05, lift_height=0.3,
    #                                     drop=True):
    #     """Move to given position, grasp the object, and lift up the
    #     object.

    #     Parameters
    #     ----------
    #     x :
    #         refer to <goto_ee_pose::xd>
    #     y :
    #         refer to <goto_ee_pose::xd>
    #     z :
    #         refer to <goto_ee_pose::xd>
    #     qx :
    #         refer to <goto_ee_pose::xd> (Default value = 0)
    #     qy :
    #         refer to <goto_ee_pose::xd> (Default value = 1)
    #     qz :
    #         refer to <goto_ee_pose::xd> (Default value = 0)
    #     qw :
    #         refer to <goto_ee_pose::xd> (Default value = 0)
    #     hover :
    #         the distance above object before grasping (Default value =
    #         0.4)
    #     dive :
    #         the distance to the object before slowing down motion
    #         (Default value = 0.05)
    #     lift_height :
    #         the height to lift the object (Default value = 0.3)
    #     drop :
    #         boolean whether drop the object after lift (Default value =
    #         True)
    #     """
    #     raise NotImplementedError
