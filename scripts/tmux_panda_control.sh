#!/bin/bash

# Template script to run ROS Cameras on workstation.
# This script will set up tmux-session with 4 windows.
# Fill in variables to customize to workspace. 

# Window 1: roscore
# Window 2: redis-server
# Window 3: camera nodes via roslaunch
# Window 4: perls2 RosRedisPublisher in python2.7 environment.

# To kill: 
# tmux kill-session -t ros_cameras
# To kill redis (sometimes it keeps running)
# sudo killall redis-server
LOCAL_PERLS2_VARS=$1
echo "Sourcing local machine config from $LOCAL_PERLS2_VARS"
source "$LOCAL_PERLS2_VARS"
session="panda_control"
##############################################################################
# Fill the following out specific to your development environment. 

# # Source virtualenv environment with perls2 installed
# SOURCE_ENV_CMD="source ~/p2env/bin/activate"

# # Directory for perls2 repo
# PERLS2_DIR="~/perls2"
# # Directory for franka-panda-iprl repo
# DRIVER_DIR="~/franka-panda-iprl"
# # Path to redis conf file.
# REDIS_CONF="~/goddard_redis.conf"

################ Do not modify below this line ###############################
DRIVER_CFG="${PERLS2_DIR}/cfg/franka-panda.yaml"
START_DRIVER_CMD="./franka_panda_driver $DRIVER_CFG"
START_REDIS_CMD="redis-server $REDIS_CONF"
START_PANDACTRL_CMD="python perls2/ctrl_interfaces/panda_ctrl_interface.py"

echo $SOURCE_ENV_CMD
echo $PERLS2_DIR
echo $DRIVER_DIR
echo $REDIS_CONF

# Convenience defines.
REDIS_PANE_NUM=0
DRIVER_PANE_NUM=1
PERLS2_CTRL_PANE_NUM=2
KILL_PANE_NUM=3

# Helper to make append C-m to any tmux send keys command.
send_tmux_cmd () {
	tmux send-keys "$1" C-m
}

start_redis_server () {
	# 1) Start redis-server
	tmux selectp -t $REDIS_PANE_NUM
	# kill any left over redis-server processes.
	send_tmux_cmd "killall redis-server"
	send_tmux_cmd "cd ~"
	send_tmux_cmd "$START_REDIS_CMD"

}

start_panda_driver () {
	tmux selectp -t $DRIVER_PANE_NUM
	send_tmux_cmd "cd ${DRIVER_DIR}/bin"
	send_tmux_cmd "$START_DRIVER_CMD"	
}

start_panda_ctrl () {
	tmux selectp -t $PERLS2_CTRL_PANE_NUM
	send_tmux_cmd "$SOURCE_ENV_CMD"
	send_tmux_cmd "cd ${PERLS2_DIR}"
	send_tmux_cmd "$START_PANDACTRL_CMD"
}

setup_kill () {
	tmux selectp -t $KILL_PANE_NUM
	send_tmux_cmd "echo \"Press Enter with this command Ctrl + C all panes in this tmux-session.\""
	send_tmux_cmd "cd ${PERLS2_DIR}"
	tmux send-keys "./scripts/kill_tmux_session.sh ${session}"
}

_tmux_ctrl_c_all_panes_() {
    tmux send-keys -t $REDIS_PANE_NUM C-c C-m
    tmux send-keys -t $DRIVER_PANE_NUM C-c C-m
    tmux send-keys -t $PERLS2_CTRL_PANE_NUM C-c C-m
}


###################################################################################
# kill any old sessions. 
tmux start-server
tmux kill-session -t $session
tmux new-session -d -s $session

tmux selectp -t 0
tmux splitw -h -p 50
tmux selectp -t 0
tmux splitw -v -p 50
tmux selectp -t 2
tmux split-w -v -p 50

# 1) Start redis-server
start_redis_server 

# 2) Start franka-panda driver
start_panda_driver 

# 3) Start perls2 Panda Ctrl Interface
start_panda_ctrl

# 4) Set up kill window. 
setup_kill

tmux attach-session -t $session
