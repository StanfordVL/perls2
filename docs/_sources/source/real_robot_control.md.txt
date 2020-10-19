# Getting started on real robots.

One of the main goals of perls2 is to make it easier to work with real robots. Currently SVL has two types of robotic arms, the REthink Sawyer Arm and the Franka Emika Panda.


## Torque control.

Torque control for real robots requires fast control frequencies for stability. Because of this, we use a separate computer other than the work station for sending these torque commands to the robot. This computer a (NUC) has nothing else running, which reduces latency for sending these commands.

For more information on this see the infrastructure descriptions.(Coming soon)

## Using the REthink Sawyer.

To use the REthink Sawyer arms, we have to use ROS-kinetic. Unfortunately ROS-kinetic only works with python 2.7, which has been outdated for some time. To overcome this limitation, we use Redis, an in-memory database to communicate between the process running on the workstation and the process sending torque commands the sawyer that runs on the NUC.

(Instructions coming soon.)
