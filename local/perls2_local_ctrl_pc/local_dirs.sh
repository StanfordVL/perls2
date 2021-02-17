#!/bin/bash
# Command to source virtualenv environment with perls2 installed
export SOURCE_ENV_CMD="source ~/p2env/bin/activate"
# Directory for perls2 repo
export PERLS2_DIR="$HOME/perls2"
# Directory for franka-panda-iprl repo
export DRIVER_DIR="$HOME/franka-panda-iprl"
########################################################
# Only edit below if you want to use a different redis conf or store your password in a directory outside perls2_local_control_pc
# Path to redis conf file.
export REDIS_CONF="redis.conf"
# Path to redis pass file
export REDIS_PASS="redis_passfile.txt"  