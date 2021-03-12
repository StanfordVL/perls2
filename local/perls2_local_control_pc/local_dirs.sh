#!/bin/bash
# Command to source virtualenv environment with perls2 installed
export SOURCE_ENV_CMD="source ~/p2env/bin/activate"
# Directory for perls2 repo
export PERLS2_DIR="$HOME/perls2"

############# Franka Panda ############################
# Directory for franka-panda-iprl repo
export DRIVER_DIR="$HOME/franka-panda-iprl"

############## Rethink Sawyer ##########################
# ROS WS directory
export ROS_DIR="$HOME/ros_ws"

# Command to source ROS
export SOURCE_ROS_CMD="source /opt/ros/kinetic/setup.bash"
export SOURCE_INTERA_CMD="cd ${ROS_DIR}; ./intera.sh"

# Source python 2.7 environment
export SOURCE_P27ENV_CMD="source ~/p27env/bin/activate"

########################################################
# Only edit below if you want to use a different redis conf or store your password in a directory outside perls2_local_control_pc
# Path to redis conf file.
export REDIS_CONF="redis.conf"
# Path to redis pass file
export REDIS_PASS="redis_passfile.txt"  