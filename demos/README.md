# Controller Demos

The Demos module provides scripts to demonstrate and validate perls2 controller performance.
Each demo creates a perls2.env and can be used to test perls2 controllers for several
different trajectories and robot interface commands. They can be configured in the command line and
can also be used to run on custom configuration files. This is helpful when debugging controller
behavior vs policy behavior.

Demos are organized into the two action spaces: OpSpace (Operational Space Control) and Joint Space


## Quickstart:
The following demos may be run out-of-the-box to verify perls2 controller performance.

The default config file for all demos is found in `perls2/demos/demo_control_cfg.yaml`.
You may modify the robot and world types in this config file as needed.


OpSpace Demos:
    * Hold EE Pose: Maintain initial end-effector pose using EEImpedance or EEPosture controller.
        Provides indication of cartesian stiffness.
        `python run_osc_fixed_demo.py`
    * Line (x/y/z): Move end-effector in a straight line using EEImpedance or EEPosture controller, while
        maintaining orientation.
        `python run_osc_line_demo.py`
    * Square : Move end-effector in a square shape in the XY plane, while maintaining orientation.
        `python run_osc_square_demo.py`
    * Rotation (x/y/z): Rotate end-effector about initial ee-frame axis a fixed amount while maintaining position.
        `python run_osc_rot_demo.py`


JointSpace Demos:
    * Gravity Compensation : set robot to gravity compensation mode using JointTorque controller.
        `python run_gc_demo.py`
    * Sequential Joint : Individually rotate each joint using JointImpedance controller. Resets robot
        after each joint test.
        `python run_jointimp_demo.py`


### Command-line specifications
Demonstrations may be customized with several command-line arguments. This is helpful in tuning gains,
or verifying a controller or robot is working properly.

You can specify the following command-line arguments for all demos. These arguments have different meanings based on the type of demo, so it's best to check documentation for the specific demo type.

For example, you can't run a JointSpace Demo with an EEImpedance controller.

You can see the valid command line args for a demo with:
`python run_<demo_name>.py -h`

The parameters general to all demos are listed here:

    --world     (str): (Optional) World to use for demo (Bullet or Real). If not defined, spec from config yaml file is used.
    --ctrl_type (str): Controller type for the demo.
    --demo_type (str): Type of demo to perform.
    --test_fn (str): RobotInterface function environment will use to perform actions. Specific to controller type.
    --delta_val (float): Step size between consecutive steps in demo. Best to keep small (~0.001)
    --num_steps (int) : Number of steps to perform in demonstration.

The following optional command line arguments are also available:

    --plot_pos (flag): Plot actualrobot states vs goal joint state at each step using matplotlib
    --plot_error (flag): Plot error at each step using matplotlib
    --save_fig (flag): save position and error plots as png.
    --config_file (str): config file to use for controller gains and parameters.

## Joint Space Demos

Joint space demos provide a demonstration of controller behavior for the following controllers:
- Joint Impedance
- Joint Velocity
- Joint Torque

### Run a joint space demo.

To run a joint space demo, use the run_joint_space_demo.py script. This demo rotates an individiually
specified joint using the controller and robot interface command specified.

### Command-line specifications for Joint Space Demos
You can specify the following command-line arguments:

    --ctrl_type (str): Controller type for the demo. Choose from: "JointImpedance", "JointVelocity" or "JointTorque"
    --demo_type (str): Type of demo to perform. Currently only SequentialJoint available.
    --test_fn (str): RobotInterface function environment will use to perform actions. Specific to controller type.
        For JointImpedance controllers you may choose from "set_joint_positions" or "set_joint_delta"
        For JointVelocity controllers you may choose from "set_joint_velocities"
        For JointTorque controllers you may choose "set_joint_torques"
    --joint_num (int): Joint number (0-6) to perform demo with. 0 is base joint, 6 is end-effector to gripper joint.
    --delta_val (float): Step size [rad] between consecutive steps in demo. Best to keep small (~0.001)

The following optional command line arguments are also available:

    --plot_pos (flag): Plot joint states with goal joint state at each step using matplotlib
    --plot_error (flag): Plot error at each step using matplotlib
    --save_fig (flag): save position and error plots as png.
    --config_file (str):

