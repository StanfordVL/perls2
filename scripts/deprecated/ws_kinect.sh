#!/bin/sh

# Script to run processes on WS necessary for grabbing images from Xbox Kinect
# Note:

# Bash script to run on WS for Xbox Kinect
# Window 1) Run kinect2_bridge
# 			Publishes kinect images to ros.
# Window 2) run kinect_ROS_interface
# 			Takes published ros messages and updates redis database (hosted on WS)

session="ws_kinect"

tmux start-server
tmux new-session -d -s $session

# 1) Set up all the panels
tmux selectp -t 0
tmux splitw -h -p 50

# 2) Start up the ros node for kinect2bridge
tmux selectp -t 0
# Connect to ROS / intera
tmux send-keys "intera" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "source ~/penv/bin/activate" C-m
# Run the ros redis interface
tmux send-keys "cd ~/ros_ws" C-m
tmux send-keys "roslaunch kinect2_bridge kinect2_bridge.launch" C-m

tmux send-keys "sleep 10" C-m

# 3) Start up Kinect ROS Interface
tmux selectp -t 1
tmux send-keys "intera" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "source ~/penv/bin/activate" C-m
# Run the ros redis interface
tmux send-keys "cd ~/perls2" C-m
tmux send-keys "python perls2/ros_interfaces/kinect_ROS_interface.py"
tmux attach-session -t $session
