Robot Interfaces
=================
Interface for obtaining robot state and sending robot control commands.
RobotInterfaces are specific to the world they inhabit, as well as the
type of robot. They contain Controllers for converting desired goal states
to a set of joint torques.

RobotInterface
--------------

.. autoclass:: perls2.robots.robot_interface.RobotInterface

    .. automethod:: change_controller
    .. automethod:: update
    .. automethod:: update_model
    .. automethod:: make_controller
    .. automethod:: step
    .. automethod:: set_controller_goal
    .. automethod:: move_ee_delta
    .. automethod:: set_ee_pose
    .. automethod:: set_joint_positions
    .. automethod:: set_joint_delta
    .. automethod:: set_joint_velocities
    .. automethod:: set_joint_torques
    .. autoproperty:: name
    .. autoproperty:: ee_pose
    .. autoproperty:: ee_position
    .. autoproperty:: ee_orientation
    .. autoproperty:: q
    .. autoproperty:: dq

BulletRobotInterface
--------------------
Extends RobotInterface functionality for BulletWorlds to be used
wity PyBullet. Connects to pybullet physics engine to obtain
robot state and dynamics parameters as well as execute torque control.

..autoclass:: perls2.robots.robot_interface.BulletRobotInterface

    ..automethod::