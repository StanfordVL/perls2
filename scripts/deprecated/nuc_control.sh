#!/bin/sh

# Note: 
# 	This is written specifically for running on c3po-ctrl NUC
# 	to communicate with r2d2-ws. Needs to be modified for other
# 	robots / workstations.

# Bash script to run on NUC for real sawyer control. 
# Window 1) Run redis server. 
#			Secured via password in .conf file
# Window 2) Run ros_redis_interface.py
#			Updates redis database with robot state.
# Window 3) Run sawyer_ctrl_interface.py
# 			Gets commands from redis and converts them into ROS messages.

session="nuc_control"

tmux start-server
tmux new-session -d -s $session

# 1) Set up all the panels
tmux selectp -t 0
tmux splitw -h -p 50
tmux selectp -t 0
tmux splitw -v -p 50
# tmux selectp -t 2
# tmux selectp -t 0
# tmux split-w -v -p 50

# 2) Start up the redis-server
tmux selectp -t 0
tmux send-keys "cd ~" C-m
tmux send-keys 'redis-server c3po_r2d2_redis.conf' C-m

# 3) Start up the Ros Redis Interface
tmux selectp -t 1
# Connect to ROS / intera
tmux send-keys "intera" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "source ~/penv/bin/activate" C-m
# Run the ros redis interface
tmux send-keys "cd ~/perls2" C-m
tmux send-keys "python perls2/ros_interfaces/ros_redis_interface.py" C-m

# 4) Start up Sawyer Ctrl Interface
tmux selectp -t 2
tmux send-keys "intera" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "source ~/penv/bin/activate" C-m
tmux send-keys "cd ~/perls2" C-m
tmux send-keys "python perls2/ros_interfaces/sawyer_ctrl_interface.py" C-m

tmux attach-session -t $session
