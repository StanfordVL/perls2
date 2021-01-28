# Using the Franka Panda

## Description

Using the Franka Panda arm with torque control requires a consistent control loop frequency for sending torque commands to the arm. In order to acheive this, a separate machine with an RT Kernel is connected to the Franka's FCI. This machine, typically an Intel NUC, is dedicated to running only the processes necessary for torque control. A separate machine, referred to as the workstation, is used to host the perls2 environment for the experiment.

perls2 provides scripts and wrappers for libfranka to perform torque control on the NUC. A Redis server is used for interprocess communication for perls2 and the libfranka driver, as well as for communication between the NUC and the workstation. See the README in perls2/redis_interfaces for more information.

## Important notes about using Franka Panda Arms.

1. **Have the E-STOP in hand.**

    Performing torque control with robots can pose a risk to both the user and the robot. It is important to take precautions and keep the E-STOP in hand when performing robot experiments.

2. **Flush redis before starting the driver.**

    The redis-server contains many flags used to operate the robot arm. It's best to regularly flush the redis server with the FLUSHALL redis-cli command before starting
    the driver. This prevents any stale commands from triggering the PandaCtrlInterface to start running.

3. **Use redis-cli**

    It's a good idea to get familiar with some basic commands in redis-cli. This allows you to get a human-readable print out of the robot's state and commands being sent.

4. **Confirm resets on the driver.**

    When the driver receives a reset command, it requires the user confirm by pressing Enter before it resets the robot. This is because the joint space controller performing the reset does not perform collision checking, and makes sure you have a hand on the E-STOP.



## Instructions for using the Franka Panda Arms
After following instructions to setup the NUC and workstation, the Franka Arms may be operated entirely from the workstation.

1. Turn on the FCI control box. The base lights of the Franka should be Yellow indicating the joints are locked.

2. Open a browser and go the Franka Desk App (usually `https://<fci-ip>` or [https://172.16.0.2](https://172.16.0.2)). Unlock the Franka Arm. You should hear a series of clicks.

3. Confirm that the Franka is unlocked by verifying the base lights are White (IDLE) mode. You should be able to move the robot by pinching the buttons above the base.


Now set up the following terminals:


#### Terminal 1: redis-server
1. ssh into the NUC

2. Run the redis server with your perls2 configuration.

    `redis-server redis.conf`

#### Terminal 2: redis-cli
1. (Don't ssh into the NUC, this process should run on the workstation)

2. Start the redis-client using the host ip for the NUC and the port set in the redis.conf file. This should also be in the perls2 config when you set up the Workstaiton.

    `redis-cli -h <NUC Local IP> -p <port>`

3. Authenticate to the redis-server with the password from the redis_passfile.txt on
the workstation.

    `AUTH <your-very-long-password>`


#### Terminal 3: franka-panda-iprl driver
1. Unlock the robot activity stop so that the base lights are Blue

2. ssh into the NUC

3. In the redis-cli terminal, flush the redis-server.

    `> flushall`

4. Run the franka-panda-iprl driver with the perls2 config

    `cd ~/franka-panda-iprl/bin`

    `./franka-panda-driver ~/perls2/cfg/franka-panda.yaml`

#### Terminal 4: perls2.PandaCtrlInterface
1. ssh into the NUC

2. source the perls2 virtual environment

3. Run the Panda Ctrl Interface

    `cd ~/perls2`

    `python perls2/ctrl_interfaces/panda_ctrl_interface.py`

#### Terminal 5: Your perls2 script
You're almost there! This is where you run your perls2 environment to perform your robot experiment.

1. Source the perls2 virtual environment.

2. Run your perls2 script. For example, run the gravity-compensation demo:

    `cd ~/perls2; python demos/run_gc_demo.py`




