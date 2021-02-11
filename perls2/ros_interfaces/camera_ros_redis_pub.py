"""Class definition for Generic ROS Camera
"""
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from perls2.redis_interfaces.redis_interface import RedisInterface
import perls2.redis_interfaces.redis_keys as R
import perls2.utils.redis_utils as RU
import struct
import logging
logging.basicConfig(level=logging.INFO)

ENV_CHECK_COUNT = 100   # how often we check if workstation is connected.

class CameraRosRedisPublisher(object):
    """Class for publish images from selected ROS topics to Redis. 
    """
    def __init__(self, 
                 host,
                 port,
                 password=None,
                 rgb_topic=None, 
                 depth_topic=None, 
                 name='camera1',
                 config=None):
        """Initialize ROS camera interface.
        """
        if config is None:
            self.RGB_TOPIC = rgb_topic
            self.DEPTH_TOPIC = depth_topic
        
        if self.RGB_TOPIC is None and self.DEPTH_TOPIC is None:
            raise ValueError("at least one topic must be defined")
        
        self.has_rgb = self.RGB_TOPIC is not None
        self.has_depth = self.DEPTH_TOPIC is not None

        # Get flat list of topics
        topics = [t for l in rospy.get_published_topics() for t in l]

        if self.RGB_TOPIC not in topics: 
            raise ValueError("Topic {} not found".format(self.RGB_TOPIC))

        # Initialize current frame
        self._rgb = None
        self._rgb_tstamp_ms = None

        self._rgb_receiver = rospy.Subscriber(
            self.RGB_TOPIC,
            Image,
            self._rgb_callback
        )

        # Set env connection flag to false. 
        self.redisClient.set('{}::env_connected'.format(name), 'False')
        self.ENV_CONN_KEY = name + R.ROS_ENV_CONN_SUFF
        self.RGB_KEY = name + R.ROS_RGB_SUFF
        self.RGB_TSTAMP_KEY = name + R.ROS_RGB_TSTAMP_SUFF
        self.loop_count = 0         # Keep track of number of loops for querying redis.

    def wait_to_receive_rgb(self):
        """ Wait to receive the newest rgb frame
        Blocking.
        """
        rospy.wait_for_message(self._KINECT_COLOR_TOPIC, Image)

    def _rgb_callback(self, rgb):
        self._rgb = CvBridge().imgmsg_to_cv2(rgb, 'bgr8')
        self._rgb_timestamp_ms = np.asarray((rgb.header.stamp.secs * 1000) +
                                            (rgb.header.stamp.nsecs / 1000000))
        
        encoded_rgb = RU.convert_frame_to_encoded_bytes(self._rgb)
        self.redisClient.set(self.RGB_KEY, encoded_rgb)
        self.redisClient.set(
            self.RGB_TSTAMP_KEY, str(self._rgb_timestamp_ms))      

    def is_env_connected(self):
        return self.redisClient.get(self.ENV_CONN_KEY).decode() == R.TRUE
    
    def check_env_connection(self, loop_count):
        """Check if perls2.Env Real Camera Interface is connected. 
        """
        if (loop_count % ENV_CHECK_COUNT == 0):
            return self.is_env_connected()
        else:
            return True

    def wait_for_env_connect(self):
        """Blocking code that waits for perls2.RobotInterface to connect to redis.

        Allows for ctrl+c key board interrupts
        """
        logging.info("Waiting for perls2.RealCameraInterface to connect to redis.")
        try:
            while True:
                if not self.is_env_connected():
                    pass
                else:
                    break
        except KeyboardInterrupt:
            logging.error("Keyboard interrupt received.")

        if self.redisClient.is_env_connected():
            logging.info("perls2.RealRobotInterface connected.")


    def wait_to_enforce_freq(self, start, freq):
        """Wait to enforce frequency of control loop.
        """
        loop_time_s = 1.0 / freq
        while ((time.time() - start) < loop_time_s):
            pass

    def run(self):
        """Run main loop to publish images to redis-server
        """
        rospy.init_node(self.name + '_node', log_level=rospy.DEBUG)
        rospy.spin()
        # try:
        #     while True:
        #         start = time.time()
        #         if (self.check_env_connection(self.loop_count)):
        #             rospy.spin()
        #         else:
        #             break
        
        # except KeyboardInterrupt:
        #     pass
        # else:
        #     pass
        # finally:
        #     pass

if __name__ == '__main__':
    import time
    
    camera = CameraRosRedisPublisher(host="127.0.0.1", port=6379, password=None, 
        rgb_topic='/sr300/color/image_raw', name='sr300')

    camera.run()