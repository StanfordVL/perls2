#!/bin/bash

# Start any ROS camera on this workstaiton. 
# roslaunch command for camera provided as second positional argument.
# 
# Usage 
# -----
#    ./start_ros_camera.sh "roslaunch camera_pkg camera.launch"
#    ./start_ros_camera.sh "roslaunch camera.launch"
#    ./start_ros_camera.sh ~/perls2_local_ws "roslaunch camera_pkg camera.launch"
# 
# This script will set up tmux-session with 4 windows.
# Window 1: roscore
# Window 2: redis-server
# Window 3: camera nodes via roslaunch
# Window 4: perls2 RosRedisPublisher in python2.7 environment.
#
# To kill:
# -------- 
# (within tmux -session): 
#   [Ctrl + B :] kill-session
# (outside tmux session):
#   tmux kill-session -t ros_cameras
# Kill redis (sometimes it keeps running)
#   sudo killall redis-server

usage="$(basename "$0") [perls2_local_ws path] [\"roslaunch cmd\"] [-h] -- Start ROS Cameras
where:
    -h show this help text

    ./start_ros_camera.sh \"roslaunch camera_pkg camera.launch\"
    ./start_ros_camera.sh \"roslaunch camera.launch\"
    ./start_ros_camera.sh ~/perls2_local_ws \"roslaunch camera_pkg camera.launch\"
    "

# Assign positional arguments.
# 1 arg: roslaunch_cmd
# 2 args: perls2_local_path roslaunch_cmd
if [ "$#" -eq 1 ] 
then
      LAUNCH_ROS_CAMERA_CMD=$1
      p2_dir=$HOME
      perls2_local_dir="${p2_dir}/perls2_local_ws"
      echo "No local directory specified ... using $perls2_local_dir"  
elif [ "$#" -eq 2 ]
  then
    perls2_local_dir=$1
    LAUNCH_ROS_CAMERA_CMD=$2
else 
  echo "Invalid args. Try ./start_ros_cameras -h to for help."
fi
  
# Check launch command is valid.
if [[ "$LAUNCH_ROS_CAMERA_CMD" != "roslaunch"* ]]
then
  echo "roslaunch command arg must be valid command i.e. \"roslaunch camera_pkg camera.launch\""
  exit
else
  echo "launching camera using $LAUNCH_ROS_CAMERA_CMD"
fi

while getopts ':h' option; do
    case "${option}" in
        h) echo "$usage"
           exit
           ;;
        :) printf "missing argument for -%s\n" "$OPTARG" >&2
           echo "$usage" >&2
           exit 1
           ;;
       \?) printf "illegal option: -%s\n" "$OPTARG" >&2
           echo "$usage" >&2
           exit 1
           ;;
    esac
done
shift $((OPTIND -1))

session="ros_cameras"
tmux start-server
tmux new-session -d -s $session
##############################################################################
# Load local environment variables

LOCAL_PERLS2_VARS="${perls2_local_dir}/local_dirs.sh"
echo "Sourcing local machine config from $LOCAL_PERLS2_VARS"
if [ ! -f $LOCAL_PERLS2_VARS ]; then
    echo "Local config folder not found...exiting"
    exit
fi
source "$LOCAL_PERLS2_VARS"

START_REDIS_CMD="redis-server $REDIS_CONF"

###################################################################################

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
sleep 1

# 2) Start redis-server
tmux selectp -t 1
tmux send-keys "killall redis-server" C-m
sleep 1
tmux send-keys "cd $perls2_local_dir" C-m
tmux send-keys "$START_REDIS_CMD" C-m
sleep 1

# 2) Start up Camera Ros Nodes
tmux selectp -t 2
# Connect to ROS / intera
tmux send-keys "$SOURCE_ROS_CMD" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "$SOURCE_P27_CMD" C-m
# Run the roslaunch command
tmux send-keys "cd $ROS_DIR" C-m
tmux send-keys "$LAUNCH_ROS_CAMERA_CMD --wait" C-m
tmux send-keys $ C-m

# 3) Start RosRedisPublisher
tmux selectp -t 3
# Connect to ROS / intera
tmux send-keys "$SOURCE_ROS_CMD" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "$SOURCE_P27_CMD" C-m
# Run the ros redis interface
tmux send-keys "cd ${PERLS2_DIR}" C-m
tmux send-keys "python perls2/ros_interfaces/ros_redis_pub.py ${perls2_local_dir}/ros_sensors.yaml" C-m
tmux attach-session -t $session