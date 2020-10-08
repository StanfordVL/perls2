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
    .. automethod:: open_gripper
    .. automethod:: close_gripper
    .. automethod:: set_gripper_to_value
    .. autoproperty:: name
    .. autoproperty:: ee_pose
    .. autoproperty:: ee_position
    .. autoproperty:: ee_orientation
    .. autoproperty:: ee_v
    .. autoproperty:: ee_w
    .. autoproperty:: ee_twist
    .. autoproperty:: q
    .. autoproperty:: dq
    .. autoproperty:: jacobian
    .. autoproperty:: linear_jacobian
    .. autoproperty:: angular_jacobian
    .. autoproperty:: mass_matrix

BulletRobotInterface
--------------------
Extends RobotInterface functionality for BulletWorlds to be used
wity PyBullet. Connects to pybullet physics engine to obtain
robot state and dynamics parameters as well as execute torque control.

.. autoclass:: perls2.robots.bullet_robot_interface.BulletRobotInterface
    :members:


BulletSawyerInterface
---------------------
Extends BulletRobotInterface for functionality specific to Rethink Sawyer Arms.

.. autoclass:: perls2.robots.bullet_sawyer_interface.BulletSawyerInterface
    :members:


BulletPandaInterface
--------------------
Extends BulletRobotInterface for functionality specific to Franka Panda arms.

.. autoclass:: perls2.robots.bullet_panda_interface.BulletPandaInterface

RealRobotInterface
------------------
Extends RobotInterface for Real Robots.

.. autoclass:: perls2.robots.real_robot_interface.RealRobotInterface

RealSawyerInterface
-------------------
Extends RealRobotInterface for REthink SawyerArms

.. autoclass:: perls2.robots.real_sawyer_interface.RealSawyerInterface

RealPandaInterface
------------------
Coming soon.