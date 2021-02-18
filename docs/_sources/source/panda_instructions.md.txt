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

1. Turn on the Franka Master Control box. The base lights of the Franka should be Yellow indicating the joints are locked.

2. Open a browser and go the Franka Desk App (usually `https://<fci-ip>` or [https://172.16.0.2](https://172.16.0.2)). Unlock the Franka Arm. You should hear a series of clicks.

3. Confirm that the Franka is unlocked by verifying the base lights are White (IDLE) mode. You should be able to move the robot by pinching the buttons above the base.

4. Pull the user-stop so that the lights at the base of the Franka are blue. 

5. Open a terminal and ssh into the control pc (NUC).
      ```bash
      ssh [control-pc-username]@[control-pc-ip]
      ```
6. Move to the perls2 repo directory, and run the `start_panda_control.sh` with the path to the `perls2_local_control_pc` as an argument. If you don't supply an path, the script will use the folder found in the `local` directory in the perls2 repo.

```bash
cd ~/perls2
./scripts/start_panda_control.sh ~/perls2_local_control_pc
```

7. Open another terminal and source your perls2 environment. You can now run your perls2 script.

8. Run your perls2 script. For example, run the gravity-compensation demo:

    ```bash
    source ~/p2env/bin/activate
    cd ~/perls2; 
    python demos/run_gc_demo.py --world=Real
    ```


