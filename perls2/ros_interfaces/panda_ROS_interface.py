""" Implementation of ROS Control Interface for Franka PAnda
"""
import rospy
import redis
import sys, time, copy
from threading import Thread, Event
import tf

import franka_gripper
from franka_gripper.msg import GraspActionGoal, MoveActionGoal, StopActionGoal, HomingActionGoal
from franka_control.msg import ErrorRecoveryActionGoal
from franka_control.srv import SetFullCollisionBehavior, SetFullCollisionBehaviorRequest
from franka_example_controllers.msg import MultiControllerCommand


class PandaROSInterface(object):
    """ Class definition for PandaROSInterface.
    Main purpose of the interface is to communicate between franka_ros and perls2.
    This is acheived by publishing robot state to redis and scraping redis for commands.

    Attributes:
        redis_client (RedisClient): redis client. Connected with specific port to Panda
        command_type (str): Type of command to execute
    """
    def __init__(self, use_gripper=True):
        """ Initialize ROS Interface.
        """
        # Initialize ros node
        rospy.init_node('panda_ros_interface')
        self.redisClient = redis.Redis()
        # Lock thread for synchronized access to internal state
        self._joint_lock = threading.Lock()

        # Create following publishers/ subscribers
        #  - Control Command Publisher
        #  - Robot State Subscriber
        #  - Joint State Subscriber
        self._robot_state_sub = rospy.Subscriber(
            '/franka_state_controller/franka_states',
            FrankaState, self._on_robot_state, queue_size=1, tcp_nodelay=True)

        self._joint_state_sub = rospy.Subscriber(
            '/joint_states',
            JointState, self._on_joint_state, queue_size=1, tcp_nodelay=True)

       self.set_collision_behavior()

    def set_collision_behavior(joint_contact_factor=0.5,
                               joint_collision_factor=2.0,
                               cartesian_contact_factor=0.5,
                               cartesian_collision_factor=2.0):
        """
        Set default collision behavior for franka.
        Args:
            joint_contact_factor (float): factor for joint-level contact threshold
            joint_collision_factor (float): factor for joint-level collision threshold (triggers reflex)
            cartesian_contact_factor (float): factor for cartesian-level contact threshold
            cartesian_collision_factor (float): factor for cartesian-level collision threshold (triggers reflex)

        """
        # service for setting collision behaviors
        rospy.wait_for_service('/franka_control/set_full_collision_behavior')
        collision_srv = rospy.ServiceProxy('/franka_control/set_full_collision_behavior', SetFullCollisionBehavior)

        # Prepare request for service
        req = SetFullCollisionBehaviorRequest()
        req.lower_torque_thresholds_acceleration = np.array([20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]) * joint_contact_factor
        req.upper_torque_thresholds_acceleration = np.array([20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]) * joint_collision_factor
        req.lower_torque_thresholds_nominal = np.array([20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]) * joint_contact_factor
        req.upper_torque_thresholds_nominal = np.array([20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]) * joint_collision_factor
        req.lower_force_thresholds_acceleration = np.array([20.0, 20.0, 20.0, 25.0, 25.0, 25.0]) * cartesian_contact_factor
        req.upper_force_thresholds_acceleration = np.array([20.0, 20.0, 20.0, 25.0, 25.0, 25.0]) * cartesian_collision_factor
        req.lower_force_thresholds_nominal = np.array([20.0, 20.0, 20.0, 25.0, 25.0, 25.0]) * joint_contact_factor
        req.upper_force_thresholds_nominal = np.array([20.0, 20.0, 20.0, 25.0, 25.0, 25.0]) * joint_collision_factor

        # Send service request
        resp = collision_srv.call(req)
        if not ret.success:
          rospy.logwarn(resp.err)
          raise ValueError('cannot set collision behavior')
        else:
            rospy.loginfo("Collision behavior set")

    def _on_robot_state(self, msg):
        self._cartesian_contact = np.array(msg.cartesian_contact)
        self._cartesian_collision = np.array(msg.cartesian_collision)

        self._joint_contact = np.array(msg.joint_contact)
        self._joint_collision = np.array(msg.joint_collision)

        self._ee_velocity = np.array(msg.EE_VEL)

        # get current ee pose
        ee_t = np.array(msg.O_T_EE).reshape((4, 4)).transpose()
        ee_pos = ee_t[:3, 3]
        ee_orn = tf.transformations.quaternion_from_matrix(ee_t)
        self._ee_pose = (ee_pos, ee_orn)

        if self.has_collided():
            print('collision')

        self.state = msg


if __name__ == '__main__':
    robot = PandaROSInterface()
    rospy.spin()