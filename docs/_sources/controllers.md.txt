# Perls2 Controllers

Controllers are used to a robot's ability to perform a task. For perls2, their inputs consist of a desired robot state and a current robot state. The outputs are control commands for the robot to acheive the desired state.

For research purposes, robotic control is the action-space of robotic learning. The controller libraries exist to unify controllers across different experiments and robot configurations. They also create transparency for ease of debugging.

All controllers in this use joint torque as an output control command. Because of the nature of torque control, control commands should be sent to the robot at a high frequency (>100 Hz, preferably 500-1000Hz).

## Controllers:
Controller types are set in the project config yaml file with the following key
`controller:
  selected_type: 'EEImpedance'
  EEImpedance:
    kp: 200 # P Gain for Impedance Control
    damping: .8 # Damping factor [0,1]
  JointVelocity:
    kp: 0.5
  JointImpedance:
    kp: 50
    damping: 1
  interpolator:
      type: 'linear'
      order: 1`

The `selected_type` determines which of the following controllers to use.

*`EEImpedance` (ee_imp.py): an end effector impedance controller that takes a desired end effector state in cartesian space. Typically used for interaction and contact-rich applications.
    *Interpolators may be defined for end effector position and orientation in the controller definition. However, only end effector position interpolation is recommended at this time.

*`JointImpedance` (joint_imp.py): a joint-space impedance controller that takes a desired joint position and velocity as an input in joint space. Typically used for trajectory-following and singularity avoidance.

*`JointVelocity` (joint_vel.py): a joint-velocity controller that takes a desired joint velocity in joint space. Typically used for visual servoing, teleoperation.

*`JointTorque` (joint_torque.py): a joint-torque controller that directly feeds joint torques through to the robot. Typically used for end-to-end learning or custom controllers applications.

*`Internal` (): signal to perls2 to use native controllers. Limited to joint position controller and end effector position control via IK.

## Interpolators:
Interpolators are used as controllers are only tuned/effective for small displacements from the current state. They provide intermediate desired robot states that the controller than uses to produce control commands. The controllers in perls2 are tuned to a maximum displacement of 0.1 for end effector position and orientation;  joint position, velocity and torque.

The available interpolators are:
*`linear`: provides a sequence of way points for the controller to reach as a linear function between the current robot state and desired state.
*`None`: controller uses no interpolation (not recommended unless

## Robot Model
The Model class (robot_model/model.py) captures the current state of the robot that the controller uses to output a control command. They are updated by RobotInterface objects.


