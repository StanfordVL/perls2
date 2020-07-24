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

def callback(data):
    global f
    current_time = time.time()
    f.writelines(current_time)
    print("cmd received at: {}".format(str(current_time)))

def listener():

    rospy.init_node("sawyer_tq_cmd_monitor", anonymous=True)

    rospy.Subscriber("/robot/limb/right/joint_command",  JointCommand, callback)
    rospy.spin()



f = open("dev/sawyer_ctrl_timing/tq_cmd_tstamps.txt", 'w')
listener()
f.close()