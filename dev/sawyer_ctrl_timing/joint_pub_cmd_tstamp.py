"""Script to monitor Sawyer Torque commands and measure timing. 

Logs timestamp via python time library when new command published to Sawyer. 

This is used to determine how long it takes for robot interface to issue torque command 
asynchronously. 


"""
import rospy
from intera_core_msgs.msg import (
    JointCommand,
    EndpointState,
    EndpointStates,
    CollisionDetectionState,
)
import time
import numpy as np

def callback(data):
    global tstamp_list
    global tstamp_nsecs_list
    if data.mode == 3: # Torque mdoe
        tstamp_list.append(float(data.header.stamp.secs) + np.multiply(1e-9, data.header.stamp.nsecs))

def listener():

    rospy.init_node("sawyer_tq_cmd_monitor", anonymous=True)

    rospy.Subscriber("/robot/limb/right/joint_command",  JointCommand, callback)
    rospy.spin()

tstamp_list =  []
#f = open("dev/sawyer_ctrl_timing/tq_cmd_tstamps.txt", 'w')
listener()
np.savez('dev/sawyer_ctrl_timing/joint_command_pub_tstamp.npz', end=np.array(tstamp_list),allow_pickle=True) 

# Calculate delta to get frequency
dt = np.diff(np.array(tstamp_list))
freq = np.divide(1.0, dt)
print("Mean frequency:\t{}\n".format(np.mean(freq)))
print("Max frequency:\t{}\n".format(np.max(freq)))
print("Min frequency:\t{}\n".format(np.min(freq)))