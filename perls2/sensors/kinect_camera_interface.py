""" Sensor Interface for Kinect Camera
"""
from perls2.sensors.camera_interface import CameraInterface

import numpy as np
import logging
from abc import ABCMeta, abstractmethod

import redis
import time
import struct
from perls2.redis_interfaces.redis_keys import *


def convert_encoded_frame_to_np(encoded_frame, dim):
    """Convert rgb, depth or ir bytes array to numpy
    """
    h, w = struct.unpack('>II', encoded_frame[:8])

    frame_np = np.frombuffer(
        encoded_frame, dtype=np.uint8, offset=8).reshape(h, w, dim)

    return frame_np


def convert_encoded_timestamp_to_np(encoded_frame, dim):
    """Convert rgb, depth or ir bytes array to numpy
    """
    h, w, timestamp = struct.unpack('>IIf', encoded_frame[:8])

    frame_np = np.frombuffer(
        encoded_frame, dtype=np.uint8, offset=8).reshape(h, w, dim)

    return (frame_np, timestamp)


def bstr_to_float(bytestring):
    """Convert bytestring array to 1d array
    """
    return np.fromstring(bytestring, dtype=np.float, sep=' ')[0]


class KinectCameraInterface(CameraInterface):
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

    DEPTH_DIM = 1
    RGB_DIM = 3
    IR_DIM = 1

    def __init__(self, config, res_mode="hd"):
        """ Set the redis parameters for the ROS interface
        Note: Stream is initialized as disabled
        """
        # Connect to redis
        self.redisClient = redis.Redis()

        # Set the res mode
        self.redisClient.set(KINECT2_RES_MODE_KEY, res_mode)
        self.config = config
        if 'invert' in self.config['sensor'].keys():
            self.redisClient.set(
                KINECT2_INVERT_KEY, str(self.config['sensor']['invert']))

        # Notify ROS Interface that camera interface is connected
        self.redisClient.set(KINECT2_INTERFACE_CONN_KEY, 'True')
        # Set stream as disabled at initialization.
        self.redisClient.set(KINECT2_STREAM_ENABLED_KEY, 'False')

        self._prev_rgb_timestamp = 0
        self._prev_rgb = []
        self.start()

    def start(self):
        """Starts the sensor stream.
        """
        self.redisClient.set(KINECT2_STREAM_ENABLED_KEY, 'True')

    def stop(self):
        """Stop the sensor stream.
        """
        self.redisClient.set(KINECT2_STREAM_ENABLED_KEY, 'False')

    def reset(self):
        """Reset the sensor stream
        TODO: Clear redis keys?
        """
        self.stop()
        self.start()

    @property
    def prev_rgb_timestamp(self):
        return self._prev_rgb_timestamp

    @prev_rgb_timestamp.setter
    def prev_rgb_timestamp(self, new_stamp):
        self._prev_rgb_timestamp = new_stamp

    @property
    def prev_rgb(self):
        return self._prev_rgb

    @prev_rgb.setter
    def prev_rgb(self, new_rgb):
        self._prev_rgb = new_rgb

    def frames(self):
        """Get frames from redis db.

        Params:
            None
        Returns:
            tuple of RGB, depth and IR frames as numpy arrays

        Images are retrieved from redis as encoded bytes array.
        They are converted to numpy arrays and returned.

        """

        encoded_rgb = self.redisClient.get(KINECT2_RGB_KEY)

        rgb_timestamp = self.redisClient.get(KINECT2_RGB_TSTAMP_KEY)

        rgb_np = convert_encoded_frame_to_np(
            encoded_rgb, KinectCameraInterface.RGB_DIM)

        encoded_depth = self.redisClient.get(KINECT2_DEPTH_KEY)
        depth_np = convert_encoded_frame_to_np(
            encoded_depth, KinectCameraInterface.DEPTH_DIM)

        encoded_ir = self.redisClient.get(KINECT2_IR_KEY)
        ir_np = convert_encoded_frame_to_np(
            encoded_ir, KinectCameraInterface.IR_DIM)

        image_dict = {'rgb': rgb_np,
                      'image_stamp': rgb_timestamp,
                      'depth': depth_np,
                      'ir': ir_np}

        return image_dict

    def frames_rgb(self):
        """Get frames from redis db.

        Params:
            None
        Returns:
            image_dict (dict): dict with key 'rgb' assigned to np.ndarray
        """
        encoded_rgb = self.redisClient.get('camera::rgb_frame')

        rgb_timestamp = self.redisClient.get('camera:rgb_timestamp')
        rgb_np = convert_encoded_frame_to_np(
            encoded_rgb, KinectCameraInterface.RGB_DIM)
        image_dict = {}
        image_dict['rgb'] = rgb_np
        return image_dict
    def disconnect(self):
        """ Set redis key to disconnect interface"""
        self.redisClient.set(KINECT2_INTERFACE_CONN_KEY, 'False')


if __name__ == '__main__':
    import time
    import pickle
    import numpy as np
    camera = KinectCameraInterface()

    logging.info("connected to interface")

    # Connect the environment and enable the stream
    camera.redisClient.set(KINECT2_INTERFACE_CONN_KEY, 'True')
    camera.redisClient.set(KINECT2_STREAM_ENABLED_KEY, 'True')
    logging.info("stream_enabled")

    camera.prev_rgb_timestamp = camera.redisClient.get(KINECT2_RGB_TSTAMP_KEY)
    camera.prev_rgb = camera.redisClient.get(KINECT2_RGB_KEY)
    timing_data = []

    input("Press Enter to continue...")
    #for sample in range(5000):
    while camera.redisClient.get('camera::stream_enabled') == 'True':
        # save rgb timestamp to compare
        start = time.time()

        while(camera.prev_rgb_timestamp == camera.redisClient.get(KINECT2_RGB_TSTAMP_KEY)):
            pass

            while ((time.time() - start) < 0.01):
                pass
                # wait to make it 100Hz
                # camera.display()

        camera.prev_rgb_timestamp = camera.redisClient.get(KINECT2_RGB_TSTAMP_KEY)
        end = time.time()

        timestamp_update_ms = (end - start) * 1000

        image_available_ms = end*1000 - bstr_to_float(camera.prev_rgb_timestamp)

        timing_data.append(
            [image_available_ms,
             timestamp_update_ms,
             end*1000,
             start*1000,
             camera.prev_rgb_timestamp])

    camera.redisClient.set(KINECT2_STREAM_ENABLED_KEY, 'False')
    camera.redisClient.set(KINECT2_INTERFACE_CONN_KEY, 'False')
    pickle_out = open("redis_timestamp_comp_pickle", 'wb')
    pickle.dump(timing_data, pickle_out)
    print("exiting.")
