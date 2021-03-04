#!/bin/bash

# Variables used by perls2 to navigate local development environment.
# Replace values of variables with commands specific to your set up.
# Necessary to make use of convenience bash scripts for starting 
# robots, cameras etc. 

########################################################
# Command to source virtualenv environment with  python 3.6 perls2 installed
export SOURCE_ENV_CMD="source ~/p2env/bin/activate"

# Command to source virtualenv environment with python2.7 perls2 installed
export SOURCE_P27_CMD="source ~/p27env/bin/activate"

# ROS WS directory
export ROS_DIR="$HOME/ros_ws"
# Command to source ROS
export SOURCE_ROS_CMD="source /opt/ros/kinetic/setup.sh"

# Roslaunch command for RealSense Cameras
LAUNCH_RS_CAMERAS_CMD="roslaunch ~/RobotTeleop/RobotTeleop/rs_multiple_devices.launch"
# Roslaunch command for Kinect Cameras
LAUNCH_KINECT_CMD="roslaunch kinect2_bridge kinect2_bridge.launch"
# Directory for perls2 repo
export PERLS2_DIR="$HOME/perls2"

########################################################
# Only edit below if you want to use a different redis conf
# Path to redis conf file.
export REDIS_CONF="redis.conf"
