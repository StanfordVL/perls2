#!/bin/bash
LOCAL_PERLS2_VARS=$1
source "$LOCAL_PERLS2_VARS"
START_REDIS_CMD="redis-server $REDIS_CONF"

killall redis-server
redis-server $REDIS_CONF