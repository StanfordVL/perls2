# Bash Scripts for Real World Experiments

## Overview
This folder contains bash scripts that are helpful for running experiments in the real world. While perls2 makes experimental setup agnostic to simulated vs real worlds, there are some extra steps to perform before running real world experiments. These primarily have to do with the software requirements for the Sawyer, and communicating with ROS. These scripts typically use tmux to run multiple processes in one terminal window, and to allow them to continue running if the terminal window is closed.

Scripts for the following are provided:

* Real robot control : NUC
* Real robot control: WS
* Kinect Camera : WS

## Real Robot Control - NUC:

### Description:
This script runs the necessary processes for real sawyer control on the NUC. The following windows are created:

* Window 1) Run redis server.
			Secured via password in .conf file. The password should match the password.txt file in your config.
* Window 2) Run ros_redis_interface.py
			Updates redis database with robot state.
* Window 3) Run sawyer_ctrl_interface.py
 			Gets commands from redis and converts them into ROS messages.

### How to use:

```bash
ssh nuc-ctrl # substitute with real NUC name.
cd ~/perls2/scripts/
./nuc_control.sh
```

### How to kill:
```bash
ssh nuc-ctrl # substitute with real NUC name.
cd ~/perls2/scripts/
./kill_nuc_control.sh
```

## Real Robot Control - WS:

### Description:
This script runs the necessary processe for real sawyer control on the Work station. The following windows are created:
* Window 1) Run sawyer_joint_pub_rate
 			Sets rate sawyer publishes joint states at to 500 Hz instead of 100 Hz
* Window 2) Reset robot window. Allows for resetting robot if e-stop has been pressed.
			# simply run the command entered in the window.


### How to use:

```bash
ssh nuc-ctrl # substitute with real NUC name.
cd ~/perls2/scripts/
./ws_control.sh
```

### How to kill:
```bash
ssh nuc-ctrl # substitute with real NUC name.
cd ~/perls2/scripts/
./kill_ws_control.sh
```

## Kinect Camera - WS

### Description
This script runs on the Workstation and runs the kinect2_bridge and perls2 KinectROSInterface in order to obtain images from the Xbox Kinect.

* Window 1) Run kinect2bridge ros node
* Window 2) Run perls2 KinectROSInterface

### How to use
```bash
cd ~/perls2/scripts/
./ws_kinect.sh
```
Press enter in the second window to run the Kinect ROS Interface after the ros node in window 1 has completed initialization and is launched.
