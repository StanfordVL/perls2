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

class RosRedisPubFactory(object):
    """Generate Ros Redis publisher based on config
    """
    def __init__(self, 
                 config):
        self.config = YamlConfig(config)
        self.pub = RosRedisPublisher(host="127.0.0.1", port=6379)
        self.rgb_topics = self.config['rgb_topics'] 

    def make_rgb_callbacks(self):
        """Make callback functions for rgb imagesfrom config
        """
        for topic in self.rgb_topics:
            # name device is always first in topic.
            name = get_name_from_topic(topic)
            setattr(self.pub, '{}_rgb_cb'.format(name), self.make_rgb_callback(name))
    
    def make_rgb_subscribers(self):
        for topic in self.rgb_topics:
            name = get_name_from_topic(topic) 
            self.rgb_subscribers[name] = self.make_rgb_subscriber(name, topic)

class RosRedisPublisher(object):
    """Class to publish rostopics to redis on callback.
    """
    def __init__(self, 
                 host,
                 port, 
                 password=None,
                 config_file='cfg/ros_sensors.yaml'):
        """Initialize publisher
        """
        self.redisClient = RedisInterface(host=host, port=port, password=password)
        self.config = YamlConfig(config_file)
        self.rgb_topics = self.config['rgb_topics']
        self.depth_topics = self.config['depth_topics']
        self.rgb_callbacks = {}
        self.rgb_subscribers = {}
        self.depth_callbacks = {}
        self.msg_callbacks = {}
    
    def run(self):
        """Run main loop to publish images to redis-server
        """
        rospy.init_node("redis_publisher_node", log_level=rospy.DEBUG)
        self.make_from_config('cfg/ros_sensors.yaml')
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
        
        encoded_rgb = RU.convert_frame_to_encoded_bytes(rgb)
        RGB_KEY = name + R.ROS_RGB_SUFF
        RGB_TSTAMP_KEY = name + R.ROS_RGB_TSTAMP_SUFF

        self.redisClient.set(RGB_KEY, encoded_rgb)
        self.redisClient.set(
            RGB_TSTAMP_KEY, str(rgb_timestamp_ms))

        # return rgb_callback         

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

    def make_from_config(self, config_file):
        self.config = YamlConfig(config_file)
        # self.make_rgb_callbacks()
        self.make_rgb_subscribers()
        self.make_depth_subscribers()


if __name__ == '__main__':
    pub = RosRedisPublisher("127.0.0.1", port=6379)
    
    pub.make_from_config('cfg/ros_sensors.yaml')

    pub.run()