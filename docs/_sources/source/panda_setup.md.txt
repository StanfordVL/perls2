# Franka Panda Setup

## Overview
This page contains instructions for setting up the Franka Panda arm with perls2.

## perls2 + Franka Panda Infrastructure

![panda_infra](../images/perls2_franka_simple.png)

perls2 uses three machines to communicate with the Franka Panda. These machines are connected in a local network. They consist of the: 

1. **FCI**: This is the main control box that comes with the Franka Panda. 

3. **Workstation (WS)**: The workstation hosting the perls2 environment and running any scripts to perform  the robot experiment.

2. **Intel NUC (NUC)** or other computer with RT-PREEMPT Kernel: 

	Using the Franka Panda arm with torque control requires a consistent control loop frequency for sending torque commands to the arm. In order to acheive this, a separate machine with an RT Kernel sends torques to the FCI. This machine, typically an Intel NUC, is dedicated to running only the processes necessary for torque control. 

	The following processes run on the NUC: 

	1. **redis-server**: An in-memory database used by perls2 for interprocess communication. Hosted on the NUC and communicated with via tcp. Allows the `perls2.PandaCtrlInterface` to bridge the `perls2.Env` on the WS with `libfranka`. See the [Redis Help](redis.md) for more on Redis and perls2.
	
	2. **franka-panda-iprl driver**: A redis wrapper for the `libfranka` driver. Sets redis keys for robot state, and gets torque commands computed by perls2 controllers from redis.
	
	3. **perls2 PandaCtrlInterface**: Process that implements the robot controller. 
		* Robot commands and controller params are obtained from the WS. 
		* Robot state is obtained from the franka-panda-iprl driver. 
		* The `PandaCtrlInterface` uses these to 	calculate the torques required to acheive desired
		robot state, and sets these to redis for the driver to send to the FCI. 


## Set up infrastructure
1. Follow the [instructions here](panda_nuc_setup.md) to setup the NUC

2. Follow the [instructions here](panda_ws_setup.md) to setup the Workstation.

3. Run a demo by following the [instructions here](panda_instructions.md) to use the Franka Panda with perls2. The Gravity Compensation demo is a good place to start.

    ```
    python demos/run_gc_demo.py --world=Real
    ```
You can try [other demos](demos.md) with the command line argument --Real as above.
