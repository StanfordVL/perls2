""" Sensor Interface for Kinect Camera
"""
from perls2.sensors.camera_interface import CameraInterface

import numpy as np

from abc import ABCMeta, abstractmethod

import redis
import time
import struct


def convert_encoded_frame_to_np(encoded_frame, dim):
    """Convert rgb, depth or ir bytes array to numpy
    """
    h, w = struct.unpack('>II', encoded_frame[:8])

    frame_np = np.frombuffer(
        encoded_frame, dtype=np.uint8, offset=8).reshape(h, w, dim)

    return frame_np

def convert_redis_depth_to_np(redis_depth):
    h, w = (1080, 1920)

    frame_np = np.frombuffer(
        redis_depth, dtype=np.dtype(float)).reshape(h, w)

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
        self.redisClient.set('camera::res_mode', res_mode)
        self.config = config
        if 'invert' in self.config['sensor'].keys():
            self.redisClient.set(
                'camera::invert', str(self.config['sensor']['invert']))

        # Notify ROS Interface that camera interface is connected
        self.redisClient.set('camera::interface_connected', 'True')
        # Set stream as disabled at initialization.
        self.redisClient.set('camera::stream_enabled', 'False')


        self._prev_rgb_timestamp = 0
        self._prev_rgb = []
        self.start()

    def start(self):
        """Starts the sensor stream.
        """
        self.redisClient.set('camera::stream_enabled', 'True')

    def stop(self):
        """Stop the sensor stream.
        """
        self.redisClient.set('camera::stream_enabled', 'False')

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

        encoded_rgb = self.redisClient.get('camera::rgb_frame')

        rgb_timestamp = self.redisClient.get('camera:rgb_timestamp')

        rgb_np = convert_encoded_frame_to_np(
            encoded_rgb, KinectCameraInterface.RGB_DIM)

        #print(rgb_timestamp)
        encoded_depth = self.redisClient.get('camera::depth_frame')
        # depth_np = convert_encoded_frame_to_np(
        #    encoded_depth, KinectCameraInterface.DEPTH_DIM)
        depth_np = convert_redis_depth_to_np(encoded_depth)
        encoded_ir = self.redisClient.get('camera::ir_frame')
        ir_np = convert_encoded_frame_to_np(
            encoded_ir, KinectCameraInterface.IR_DIM)

        image_dict = {'rgb': rgb_np,
                      'image_stamp': rgb_timestamp,
                      'depth': depth_np,
                      'ir': ir_np}

        return image_dict

    def disconnect(self):
        """ Set redis key to disconnect interface"""
        self.redisClient.set('camera::interface_connected', 'False')


if __name__ == '__main__':
    import time
    import pickle
    import numpy as np
    camera = KinectCameraInterface()

    print("connected to interface")

    # Connect the environment and enable the stream
    camera.redisClient.set('camera::interface_connected', 'True')
    camera.redisClient.set('camera::stream_enabled', 'True')
    print("stream_enabled")

    camera.prev_rgb_timestamp = camera.redisClient.get('camera::rgb_timestamp')
    camera.prev_rgb = camera.redisClient.get('camera::rgb_frame')
    timing_data = []

    input("Press Enter to continue...")
    for sample in range(5000):
        # save rgb timestamp to compare
        start = time.time()

        while(
            camera.prev_rgb_timestamp == camera.redisClient.get(
                    'camera::rgb_timestamp')):
            pass

            while ((time.time() - start) < 0.001):
                pass
                # wait to make it 100Hz
                # camera.display()

        camera.prev_rgb_timestamp = camera.redisClient.get(
                'camera::rgb_timestamp')
        end = time.time()

        timestamp_update_ms = (end - start) * 1000

        image_available_ms = end*1000 - bstr_to_float(camera.prev_rgb_timestamp)

        timing_data.append(
            [image_available_ms,
             timestamp_update_ms,
             end*1000,
             start*1000,
             camera.prev_rgb_timestamp])

    camera.redisClient.set('camera::stream_enabled', 'False')
    camera.redisClient.set('camera::interface_connected', 'False')
    pickle_out = open("redis_timestamp_comp_pickle", 'wb')
    pickle.dump(timing_data, pickle_out)
    print("exiting.")
