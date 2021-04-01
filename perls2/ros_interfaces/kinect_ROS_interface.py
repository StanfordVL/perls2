""" Sensor Interface for Kinect Camera
"""
#from perls2.sensors.camera_interface import CameraInterface

import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from abc import ABCMeta, abstractmethod
import logging
logging.basicConfig(level=logging.DEBUG)
import redis
import time
import struct
from perls2.redis_interfaces.redis_keys import *


def convert_frame_to_encoded_bytes(frame):
    """Convert rgb, depth or ir frame to bytes array with encoded dim
    """
    height = np.shape(frame)[0]
    width = np.shape(frame)[1]

    # Encode the shape of the picture into the bytes array
    frame_np = np.array(frame).astype('uint8')
    frame_bytes = frame_np.tobytes()
    frame_shape = struct.pack('>II', height, width)
    encoded_frame = frame_shape + frame_bytes
    return encoded_frame


def convert_frame_to_bytes_timestamp(frame, timestamp):
    """Convert rgb, depth or ir frame to bytes array with encoded dim
    """
    height = np.shape(frame)[0]
    width = np.shape(frame)[1]
    # Encode the shape of the picture into the bytes array
    frame_np = np.array(frame).astype('uint8')
    frame_bytes = frame_np.tobytes()
    timestamp
    frame_shape = struct.pack('>IIf', height, width, timestamp)
    encoded_frame = frame_shape + frame_bytes
    return encoded_frame


class KinectROSInterface():
    """ Kinect camera class that gets images via ROS.

    Attributes:
        _depth : numpy array
            depth image
        _rgb : numpy array
            rgb image
        _ir : numpy array
            ir image
        _KINECT_DEPTH_TOPIC: str
            rostopic for depth stream
        _KINECT_COLOR_TOPIC: str
            rostopic for color stream
        _KINECT_IR_TOPIC: str
            rostopic for ir stream


    Notes:
        Images None initialy, need to call capture_images before accessing
    """
    DEF_INTR_RGB_HD = np.array([
        [1.0450585754139581e+03, 0., 9.2509741958808945e+02],
        [0., 1.0460057005089166e+03, 5.3081782987073052e+02],
        [0., 0., 1.]], dtype=np.float32)

    DEF_DIST_RGB_HD = np.array([
        1.8025470248423700e-02, -4.0380385825573024e-02,
        -6.1365440651701009e-03, -1.4119705487162354e-03,
        9.5413324012517888e-04], dtype=np.float32)

    __metaclass__ = ABCMeta

    SD_IMG_SIZE = (424, 512)
    HD_IMG_SIZE = (1080, 1920)
    QHD_IMG_SIZE = (540, 960)

    def __init__(self, res_mode="sd"):
        """ Constructor """
        self._depth = None
        self._ir = None
        self._rgb = None
        self._model = None

        assert(res_mode in ['hd', 'sd', 'qhd'])
        self._KINECT_DEPTH_TOPIC = "/kinect2/%s/image_depth_rect" % res_mode
        self._KINECT_IR_TOPIC = "/kinect2/sd/image_ir_rect"
        self._KINECT_COLOR_TOPIC = "/kinect2/%s/image_color_rect" % res_mode

        # Get flat list of topics
        topics = [t for l in rospy.get_published_topics() for t in l]
        if self._KINECT_DEPTH_TOPIC not in topics:
            raise ValueError("Topic %s not found" %
                             self._KINECT_DEPTH_TOPIC)
        if self._KINECT_COLOR_TOPIC not in topics:
            raise ValueError("Topic %s not found" %
                             self._KINECT_COLOR_TOPIC)
        if self._KINECT_IR_TOPIC not in topics:
            raise ValueError("Topic %s not found" %
                             self._KINECT_IR_TOPIC)
        self._depth_receiver = rospy.Subscriber(
            self._KINECT_DEPTH_TOPIC,
            Image,
            self._depth_callback
        )
        self._rgb_receiver = rospy.Subscriber(
            self._KINECT_COLOR_TOPIC,
            Image,
            self._rgb_callback
        )
        self._ir_receiver = rospy.Subscriber(
            self._KINECT_IR_TOPIC,
            Image,
            self._ir_callback
        )

        # Set up redis Client
        self.redisClient = redis.Redis()
        self.redisClient.set('env_connected', 'False')

        self.invert = False

    @property
    def rgb_frame(self):
        return self._rgb

    @property
    def depth_frame(self):
        return self._depth

    @property
    def ir_frame(self):
        return self._ir

    def has_frames(self):
        """ Check if frames have been acquired """
        return self._rgb is not None and self._depth is not None

    def display(self, waitkey=0):
        """ Show the frames via OpenCV. Blocks til key pressed. """

        # Just for testing out the redis part. TODO switch this out

        rgb_encoded = self.redisClient.get(KINECT2_RGB_KEY)
        h, w = struct.unpack('>II', rgb_encoded[:8])
        image_np = np.frombuffer(
            rgb_encoded, dtype=np.uint8, offset=8).reshape(h, w, 3)
        # depth_frame = self.redisClient.get('camera:depth_frame')
        # ir_frame = self.redisClient.get('camera::ir_frame')
        rgb_frame = image_np

        cv2.imshow('rgb', rgb_frame)
        # cv2.imshow('d', depth_frame)
        cv2.waitKey(waitkey)
        cv2.destroyAllWindows()

    def capture_frames(self):
        """ Capture the newest frame """
        self.wait_to_receive()

        if self.invert:
            rgb_frame = cv2.cvtColor(self.rgb_frame, cv2.COLOR_RGB2BGR)
        else:
            rgb_frame = self.rgb_frame
        encoded_rgb = convert_frame_to_encoded_bytes(rgb_frame)
        encoded_depth = convert_frame_to_encoded_bytes(self.depth_frame)
        encoded_ir = convert_frame_to_encoded_bytes(self.ir_frame)
        # set the key in redis
        self.redisClient.set(KINECT2_RGB_KEY, encoded_rgb)
        self.redisClient.set(
            KINECT2_RGB_TSTAMP_KEY, str(self._rgb_timestamp_ms))
        self.redisClient.set(KINECT2_DEPTH_KEY, encoded_depth)
        self.redisClient.set(KINECT2_IR_KEY, encoded_ir)

        return (self.rgb_frame, self.depth_frame, self.ir_frame)

    def capture_rgb(self):
        """ Capture the newest rgb frame """
        self.wait_to_receive_rgb()
        rospy.logdebug("capture_rgb: rgb received")
        encoded_rgb = convert_frame_to_encoded_bytes(self.rgb_frame)
        self.redisClient.set(KINECT2_RGB_KEY, encoded_rgb)
        self.redisClient.set(
            KINECT2_RGB_TSTAMP_KEY, str(self._rgb_timestamp_ms))

    def wait_to_receive(self):
        """ Wait to receive the newest frame """
        rospy.wait_for_message(self._KINECT_DEPTH_TOPIC, Image)
        rospy.wait_for_message(self._KINECT_IR_TOPIC, Image)
        rospy.wait_for_message(self._KINECT_COLOR_TOPIC, Image)

    def wait_to_receive_rgb(self):
        """ Wait to receive the newest rgb frame"""
        rospy.wait_for_message(self._KINECT_COLOR_TOPIC, Image)

    def unregister(self):
        """ Unregister the subscribers """
        self._depth_receiver.unregister()
        self._rgb_receiver.unregister()
        self._ir_receiver.unregister()

    def _depth_callback(self, depth):
        self._depth = CvBridge().imgmsg_to_cv2(depth, '16UC1')

    def _rgb_callback(self, rgb):
        self._rgb = CvBridge().imgmsg_to_cv2(rgb, 'bgr8')
        self._rgb_timestamp_ms = np.asarray((rgb.header.stamp.secs * 1000) +
                                            (rgb.header.stamp.nsecs / 1000000))

    def _ir_callback(self, ir):
        self._ir = CvBridge().imgmsg_to_cv2(ir, '16UC1')

    @property
    def camera_model(self):
        # TODO
        pass

    def callback_view(self, callback):
        """
        Create an OpenCV window with the passed in callback

        Args:
            callback: function to call on click. Should accept OpenCV args.
        """
        def show_image(img_data):
            cv_image = CvBridge().imgmsg_to_cv2(img_data, 'bgr8')

            color = cv_image

            cv2.namedWindow('kinect', cv2.CV_WINDOW_AUTOSIZE)
            cv2.setMouseCallback('kinect', callback)
            cv2.imshow('kinect', color)

            cv2.waitKey(1)

        sub = rospy.Subscriber(
            KinectROSAccessor.KINECT_COLOR_TOPIC,
            Image, show_image)

        try:
            rospy.init_node('interactive_display')
            rospy.spin()

        except KeyboardInterrupt:
            cv2.destroyWindow('kinect_grasp')
            sub.unregister()


if __name__ == '__main__':
    import time
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--display", action='store_true', help='display the rgb frame. For DEBUG only!')
    args = parser.parse_args()

    display_image = False
    if args.display:
        display_image = True

    rospy.init_node('kinect_show', log_level=rospy.DEBUG)

    camera = KinectROSInterface()

    rospy.loginfo("waiting for camera interface")
    while(camera.redisClient.get(KINECT2_INTERFACE_CONN_KEY) != b'True'):
        pass

    rospy.loginfo("interface connected")
    if camera.redisClient.get(KINECT2_INVERT_KEY) == 'True':
        camera.invert = True
    else:
        camera.invert = False
    while (camera.redisClient.get(KINECT2_INTERFACE_CONN_KEY) == b'True'):
        if (camera.redisClient.get(KINECT2_STREAM_ENABLED) == b'True'):
            start = time.time()
            camera.capture_frames()
            if display_image:
                camera.display()
            while ((time.time() - start) < 0.033):
                pass
    if (camera.redisClient.get(KINECT2_INTERFACE_CONN_KEY) == b'False'):
        rospy.loginfo('Camera interface disconnected')

    rospy.loginfo("exiting.")
