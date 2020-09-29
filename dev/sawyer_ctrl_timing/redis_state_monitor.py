"""Script for monitoring redis state changes. 

Logs the time from the local machine when an update to 
the redis robot state occurs. 
"""
import redis
import argparse

redisClient = redis.Redis()

MONITOR_KEY = "robot::state::tstamp"
UPDATE_TSTAMP_LOG = "dev/sawyer_ctrl_timing/state_update_tstamps.txt"

last_tstamp = redisClient.get(MONITOR_KEY)
logfile = open(UPDATE_TSTAMP_LOG, 'w')

while True: 
    if redisClient.get(MONITOR_KEY) != last_tstamp
        last_tstamp = redisClient.get(MONITOR_KEY)
        logfile.writelines(str(time.time()))        
