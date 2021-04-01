#!/bin/sh

# Template script to run ROS Cameras on workstation.
# This script will set up tmux-session with 4 windows.
# Fill in variables to customize to workspace. 

# Window 1: roscore
# Window 2: redis-server
# Window 3: camera nodes via roslaunch
# Window 4: perls2 RosRedisPublisher in python2.7 environment.

# To kill: 
# (within tmux -session): [Ctrl + B :] kill-session

# To kill outside: 
# tmux kill-session -t ros_cameras
# To kill redis (sometimes it keeps running)
# sudo killall redis-server

session="ros_cameras"
##############################################################################
# Fill the following out specific to your development environment. 

# ROS workspace directory
ROS_DIR="~/ros_ws"
SOURCE_ROS_CMD="source /opt/ros/kinetic/setup.sh"
#roslaunch command for cameras
LAUNCH_CMD="roslaunch ~/RobotTeleop/RobotTeleop/rs_multiple_devices.launch --wait"

# Source python 2.7 environment with perls2
SOURCE_P27_CMD="source ~/p27env/bin/activate"

# Directory for perls2 install
PERLS2_DIR="~/perls2"

# Launch redis-server (conf optional)
REDIS_CONF=""
START_REDIS_CMD="redis-server"
###################################################################################

tmux start-server
tmux new-session -d -s $session

tmux selectp -t 0
tmux splitw -h -p 50
tmux selectp -t 0
tmux splitw -v -p 50
tmux selectp -t 2
tmux split-w -v -p 50

# 1) Start ros
tmux selectp -t 0
tmux send-keys "$SOURCE_ROS_CMD" C-m
tmux send-keys "roscore" C-m

# 2) Start redis-server
tmux selectp -t 1
tmux send-keys "cd ~" C-m
tmux send-keys "$START_REDIS_CMD" C-m


# 2) Start up Camera Ros Nodes
tmux selectp -t 2
# Connect to ROS / intera
tmux send-keys "$SOURCE_ROS_CMD" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "$SOURCE_P27_CMD" C-m
# Run the roslaunch command
tmux send-keys "cd $ROS_DIR" C-m
tmux send-keys "$LAUNCH_CMD" C-m
tmux send-keys $ C-m

# 3) Start RosRedisPublisher
tmux selectp -t 3
# Connect to ROS / intera
tmux send-keys "$SOURCE_ROS_CMD" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "$SOURCE_P27_CMD" C-m
# Run the ros redis interface
tmux send-keys "cd ${PERLS2_DIR}" C-m
tmux send-keys "python perls2/ros_interfaces/ros_redis_pub.py" C-m

tmux attach-session -t $session
