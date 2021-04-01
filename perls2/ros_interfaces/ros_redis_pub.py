"""Class definition for Ros Redis Publisher
"""
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from perls2.redis_interfaces.redis_interface import RedisInterface
import perls2.redis_interfaces.redis_keys as R
import perls2.utils.redis_utils as RU
from perls2.utils.yaml_config import YamlConfig
from types import MethodType


def get_name_from_topic(topic_str):
    return topic_str[1:].split("/")[0]


class RosRedisPublisher(object):
    """Class to publish rostopics to redis on callback.
    """
    def __init__(self, 
                 config_file='cfg/ros_sensors.yaml', 
                 invert=True):
        """Initialize publisher
        """
        self.config = YamlConfig(config_file)
        # Override default invert flag with config setting.
        if 'invert' in self.config:
            self.invert = self.config['invert']
        else: 
            self.invert = invert
        redis_kwargs = self.config['workstation_redis']
        self.redisClient = RedisInterface(**redis_kwargs)

        self.rgb_topics = self.config['rgb_topics']
        self.depth_topics = self.config['depth_topics']
        self.rgb_callbacks = {}
        self.rgb_subscribers = {}
        self.depth_callbacks = {}
        self.msg_callbacks = {}
        self.run()

    def run(self):
        """Run main loop to publish images to redis-server
        """
        rospy.init_node("redis_publisher_node", log_level=rospy.DEBUG)
        self.make_subscribers()
        rospy.spin()

    def depth_callback(self, data, name):
        depth = CvBridge().imgmsg_to_cv2(data, '16UC1')
        encoded_depth = RU.convert_frame_to_encoded_bytes(depth)

        DEPTH_KEY = name + R.ROS_DEPTH_SUFF
        self.redisClient.set(DEPTH_KEY, encoded_depth)

    def rgb_callback(self, data, name):
        rgb = CvBridge().imgmsg_to_cv2(data, 'bgr8')
        rgb_timestamp_ms = np.asarray((data.header.stamp.secs * 1000) +
                                            (data.header.stamp.nsecs / 1000000))
        if self.invert:
            rgb = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        
        encoded_rgb = RU.convert_frame_to_encoded_bytes(rgb)
        RGB_KEY = name + R.ROS_RGB_SUFF
        RGB_TSTAMP_KEY = name + R.ROS_RGB_TSTAMP_SUFF

        self.redisClient.set(RGB_KEY, encoded_rgb)
        self.redisClient.set(
            RGB_TSTAMP_KEY, str(rgb_timestamp_ms))
      

    def make_rgb_subscribers(self):
        for topic in self.rgb_topics:
            name = get_name_from_topic(topic) 
            rospy.logdebug("Making {} rgb subscriber".format(name))
            self.rgb_subscribers[name] = self.make_rgb_subscriber(name, topic)

    def make_rgb_subscriber(self, name, topic):
        rgb_subscriber = rospy.Subscriber(
            topic,
            Image,
            self.rgb_callback,
            name
        )
        return rgb_subscriber
    
    def make_depth_subscribers(self):
        for topic in self.depth_topics:
            name = get_name_from_topic(topic) 
            rospy.logdebug("Making {} depth subscriber".format(name))
            self.rgb_subscribers[name] = self.make_depth_subscriber(name, topic)

    def make_depth_subscriber(self, name, topic):
        depth_subscriber = rospy.Subscriber(
            topic,
            Image,
            self.depth_callback,
            name
        )
        return depth_subscriber

    def make_subscribers(self):
        self.make_rgb_subscribers()
        self.make_depth_subscribers()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Publish ROS msgs to redis')
    parser.add_argument('config', metavar='config_filepath', default='local/perls2_local_ws/ros_sensors.yaml')
    args = parser.parse_args()


    pub = RosRedisPublisher(args.config)
