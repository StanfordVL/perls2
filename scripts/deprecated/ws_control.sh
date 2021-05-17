#!/bin/sh

# Script to run processes on WS necessary for real sawyer control.
# Note:
# 	This is written specifically for running on r2d2-ws Workstation
# 	to communicate with c3po-ctrl. Needs to be modified for other
# 	robots / workstations.

# Bash script to run on NUC for real sawyer control.
# Window 1) Run sawyer_joint_pub_rate
# 			Sets rate sawyer publishes joint states at to 500 Hz instead of 100 Hz
# Window 2) Reset robot window. Allows for resetting robot if e-stop has been pressed.
			# simply run the command entered in the window.

session="ws_control"

tmux start-server
tmux new-session -d -s $session

# 1) Set up all the panels
tmux selectp -t 0
tmux splitw -h -p 50
tmux selectp -t 0
tmux splitw -v -p 50


# 2) Run sawyer_joint_pub_rate.py
tmux selectp -t 0
# Connect to ROS / intera
tmux send-keys "intera" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "source ~/penv/bin/activate" C-m
# Run the ros redis interface
tmux send-keys "cd ~/perls2" C-m
tmux send-keys "python perls2/ros_interfaces/sawyer_joint_pub_rate.py" C-m

# 4) Set up reset robot window.
tmux selectp -t 1
tmux send-keys "intera" C-m
# Load command to reset robot. Press enter to run.
tmux send-keys "rosrun intera_interface enable_robot.py -e"

tmux attach-session -t $session
