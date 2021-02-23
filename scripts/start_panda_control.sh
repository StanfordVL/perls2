#!/bin/bash

usage="$(basename "$0") [path to perls2_control_pc directory] [-h] -- Start control PC from workstation
where:
    -h show this help text

    ./start_panda_control.sh
    ./start_panda_control.sh ~/perls2_local_control_pc
    "
if [ "$#" -eq 2 ] 
then
    perls2_local_dir=$1
else
    echo "No local directory specified ... using ${PWD}/perls2_local_control_pc"
    p2_dir=$PWD
    perls2_local_dir="${p2_dir}/local/perls2_local_control_pc"  
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

####################################################################

CURPID=$$
# Kill all the child process id for a  given pid
# Store the current Process ID, we don't want to kill the current executing process id
_kill_process_subchildren() {

    ppid=$1
    echo "killing subchildren of process $ppid"
    arraycounter=1

    FORLOOP=FALSE
    # Get all the child process id
    for i in `ps -ef| awk '$3 == '$ppid' { print $2 }'`
    do
            if [ $i -ne $CURPID ] ; then
                    procid[$arraycounter]=$i
                    arraycounter=`expr $arraycounter + 1`
                    ppid=$i
                    FORLOOP=TRUE
            fi
    done
    if [ "$FORLOOP" = "FALSE" ] ; then
       arraycounter=`expr $arraycounter - 1`
       ## We want to kill child process id first and then parent id's
       while [ $arraycounter -ne 0 ]
        do
         kill -2 "${procid[$arraycounter]}" >/dev/tty
         arraycounter=`expr $arraycounter - 1`
        done
        
    fi
}


cleanup () {
    echo "Shutting down Panda Control" >/dev/tty
    kill -2 "$PANDACTRL_PID" >/dev/tty
    sleep 3
    # _kill_process_subchildren $PANDACTRL_PID
    kill -2 "$DRIVER_PID" >/dev/tty
    # _kill_process_subchildren $DRIVER_PID
    # _kill_process_subchildren $REDIS_PID
    # _kill_process_subchildren $REDIS_PID
    exit
}
####################################################################
trap cleanup SIGINT

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
START_PANDACTRL_CMD="python perls2/ctrl_interfaces/panda_ctrl_interface.py --config=${PANDACTRL_CFG}"

# Start redis-server
killall "redis-server"
sleep 2
echo "Starting redis-server"
eval "${START_REDIS_CMD} &"
REDIS_PID=$!
sleep 2
echo "#######################################"
# # Start franka-panda-iprl driver
if [ ! -d ${DRIVER_DIR} ]
then
  echo "Driver directory not found. ${DRIVER_DIR}"
  echo "Check DRIVER_DIR in ${perls2_local_dir}/local_dirs.sh"
  exit
else
  echo "Starting franka-panda-iprl driver ..."
  echo "Loading driver config from $DRIVER_CFG"
  cd ${DRIVER_DIR}"/bin/"
  eval "$START_DRIVER_CMD &"
  DRIVER_PID=$! 
  sleep 2
fi

echo "#######################################"
echo "Starting perls2.PandaCtrlInterface..."
# Start Panda Ctrl Interface
cd ~
eval "$SOURCE_ENV_CMD" 
cd "${PERLS2_DIR}"
eval "$START_PANDACTRL_CMD &"
PANDACTRL_PID=$!

# # idle waiting for abort from user
read -r -d '' _ </dev/tty
