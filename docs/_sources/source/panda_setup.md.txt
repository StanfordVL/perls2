# Franka Panda Setup

## Overview
This page contains instructions for setting up the Franka Panda arm with perls2.

## perls2 + Franka Panda Infrastructure

Using the Franka Panda arm with torque control requires a consistent control loop frequency for sending torque commands to the arm. In order to acheive this, a separate machine with an RT Kernel is connected to the Franka's FCI. This machine, typically an Intel NUC, is dedicated to running only the processes necessary for torque control. A separate machine, referred to as the workstation, is used to host the perls2 environment for the experiment.

perls2 provides scripts and wrappers for libfranka to perform torque control on the NUC. A Redis server is used for interprocess communication for perls2 and the libfranka driver, as well as for communication between the NUC and the workstation. See the README in perls2/redis_interfaces for more information.

## Set up infrastructure
1. Follow the [instructions here](panda_nuc_setup.md) to setup the NUC

2. Follow the [instructions here](panda_ws_setup.md) to setup the Workstation.

3. Run a demo by following the [instructions here](panda_instructions.md) to use the Franka Panda with perls2. The Gravity Compensation demo is a good place to start.

    ```
    python demos/run_gc_demo.py --Real
    ```
You can try [other demos](demos.md) with the command line argument --Real as above.











