""" Ros node for publishing joint state publish rate to Sawyer 

By default, the sawyer only publishes at rate of 100Hz, which is unsuitable for torque control.
The max frequency at which the Sawyer can publish its joint state is 800Hz, which is suitable for torque control.  

This script expects the sawyer to be on and running. The channel
    '/robot/joint_state_publish_rate' should already exist.

"""

import rospy
from std_msgs.msg import UInt16
import argparse

parser = argparse.ArgumentParser(description="Rate to publish joint state from Sawyer")
parser.add_argument('--publish_rate', metavar='f', type=int, default=500, help="rate at which sawyer should publish joint state")
args = parser.parse_args()


JOINT_STATE_PUBLISH_RATE = args.publish_rate


rospy.logdebug("Creating joint state publish rate publisher")
pub = rospy.Publisher('/robot/joint_state_publish_rate', UInt16, queue_size=5)
rospy.init_node("sawyer_joint_pub_rate", anonymous=True)
rate = rospy.Rate(10)

rospy.logdebug("Publishing to joint state publish rate topic...")
rospy.logdebug("")
while not rospy.is_shutdown():
    pub.publish(JOINT_STATE_PUBLISH_RATE)
    rate.sleep()