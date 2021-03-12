#!/bin/bash

# Note:
# 	This is written specifically for running on c3po-ctrl NUC
# 	to communicate with r2d2-ws. Needs to be modified for other
# 	robots / workstations.
#!/bin/bash

usage="$(basename "$0") [path to perls2_control_pc directory] [-h] -- Start control PC from workstation
where:
    -h show this help text

    ./start_sawyer_control.sh
    ./start_sawyer_control.sh ~/perls2_local_control_pc
    "
if [ "$#" -eq 2 ]
then
    perls2_local_dir=$1
else
    echo "No local directory specified ... using ${HOME}/perls2_local_control_pc"
    p2_dir=$HOME
    perls2_local_dir="${HOME}/perls2_local_control_pc"
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



LOCAL_PERLS2_VARS="${perls2_local_dir}/local_dirs.sh"
echo "Sourcing local machine config from $LOCAL_PERLS2_VARS"
if [ ! -f $LOCAL_PERLS2_VARS ]; then
    echo "Local config folder not found...exiting"
    exit
fi
source "$LOCAL_PERLS2_VARS"

DRIVER_CFG="${perls2_local_dir}/franka-panda.yaml"
PANDACTRL_CFG="${perls2_local_dir}/panda_ctrl_config.yaml"
REDIS_PASSFILE="${perls2_local_dir}/${REDIS_PASS}"

START_DRIVER_CMD="./franka_panda_driver $DRIVER_CFG"
START_REDIS_CMD="redis-server ${perls2_local_dir}/$REDIS_CONF"

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
tmux selectp -t 2
tmux split-w -v -p 50

# tmux selectp -t 2
# tmux selectp -t 0
# tmux split-w -v -p 50

# 2) Start up the redis-server
tmux selectp -t 0
tmux send-keys "cd ~" C-m
tmux send-keys "${START_REDIS_CMD}" C-m

# 3) Start up the Ros Redis Interface
tmux selectp -t 1
# Connect to ROS / intera
tmux send-keys "${SOURCE_INTERA_CMD}" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "${SOURCE_P27ENV_CMD}" C-m

# Run the ros redis interface
tmux send-keys "cd ${PERLS2_DIR}" C-m
tmux send-keys "python perls2/ros_interfaces/ros_redis_interface.py" C-m


# 2) Run sawyer_joint_pub_rate.py
tmux selectp -t 2
# Connect to ROS / intera
tmux send-keys "${SOURCE_INTERA_CMD}" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "${SOURCE_P27ENV_CMD}" C-m
# Run sawyer joint rate publisher node.
tmux send-keys "cd ${PERLS2_DIR}" C-m
tmux send-keys "python perls2/ros_interfaces/sawyer_joint_pub_rate.py" C-m

# 4) Start up Sawyer Ctrl Interface
tmux selectp -t 3
tmux send-keys "${SOURCE_INTERA_CMD}" C-m
# Source perls python 2.7 virtual environment
tmux send-keys "${SOURCE_P27ENV_CMD}" C-m
tmux send-keys "cd ${PERLS2_DIR}" C-m
tmux send-keys "python perls2/ros_interfaces/sawyer_ctrl_interface.py" C-m

tmux attach-session -t $session
