# perls2 CtrlInterfaces

The perls2.ctrl_interfaces module provides ctrl_interfaces for real robot control.

## Description:
perls2 uses two machines to perform real robot control: a workstation where the perls2 environment is hosted, and a machine running a RT Kernel (typically a NUC). The NUC runs the ctrl_interface specific to the robot, along with other additional processes or drivers specific to the robot.

The ctrl_interface provides a link between the workstation and the driver for controlling the robot. The workstation sends robot commands and controller parameters at the policy frequency (typically 20Hz) to the CtrlInterface via redis. The robot's driver provides the ctrl_interface with the robot's current state by updating redis. At the control frequency (typically 500-1000Hz), the CtrlInterface uses the robot command, controller specifications and robot state to calculate the torques required to acheive the desired robot state. These torques are sent to the driver using the Redis Interface.

## Config files:
Don't modify the config file specific to the Ctrl Interface. Instead, modify the controller config on your workstation  specific to the perls2 environment. The perls2.RealRobotInterface will pass these on when it connects to the redis-server.

## Running Ctrl Interface for the Franka Panda

To run the PandaCtrlInterface:
1. ssh into the NUC
2. Start the redis-server.
    `redis_server my_redis_config.conf`
3. Start the franka-panda-iprl driver. (you'll get an error otherwise)
    `./bin/franka-panda-driver`
4. source the perls2 env
    `source ~/p2env/bin/activate`
5. Run the Ctrl Interface.
    `cd ~/perls2`
    `python perls2/ctrl_interfaces/panda_ctrl_interface.py`

6. Wait till you see the message:
    "Waiting for perls2.RealRobotInterface to connect to redis."

7. You may now run your perls2 environment on the workstation. Make sure your config file is set to 'Real'!


## Exiting the driver:
The driver will exit if the workstation interface disconnects from the redis-server i.e.
`robot_interface.disconnect()` is called. You can also exit the driver with Ctrl + C in the terminal.

## Changing controllers:
It is possible to change the controller after the driver and workstation have begun running, but only after resetting the robot. This is because creating the new controller disrupts control loop timing.
