"""Class definition for RosCameraInterface
"""

from perls2.redis_interfaces.redis_interface import RedisInterface
import perls2.redis_interfaces.redis_keys as R 
import perls2.utils.redis_utils as RU

class RosCameraInterface(object):
    """Class definition for RosCameraInterface. 

    The RosCameraInterface allows for images obtained from a camera that publishes to ros. 
    It works in conjunction with a CameraRosRedisBridge, which subscribes to the rostopic 
    the camera publishes to, and updates a key on a redis server when a new message is
    published. 

    This is meant to be used in the python3.6 perls2 environment. 

    """

    RGB_DIM = 3
    DEPTH_DIM = 1
    
    def __init__(self, name, config=None):
        """ Initialize the ROS Camera interface. 
        """
        self.name = name
        self.RGB_KEY = name + R.ROS_RGB_SUFF
        self.RGB_TSTAMP_KEY = name + R.ROS_RGB_TSTAMP_SUFF
        self.DEPTH_KEY = name + R.ROS_DEPTH_SUFF
        
        # Connect to redis-server
        self.redisClient = RedisInterface(host="127.0.0.1", port=6379)
        self.redisClient.set(self.name + R.ROS_ENV_CONN_SUFF, R.TRUE)

    def _get_rgb_frame(self):
        """Get rgb frame as a numpy array.
        """
        encoded_rgb = self.redisClient.get(self.RGB_KEY)

        rgb_np = RU.convert_encoded_frame_to_np(
            encoded_rgb, RosCameraInterface.RGB_DIM)

        return rgb_np

    def _get_rgb_tstamp(self):
        """Get rgb tstamp as a string
        """
        return self.redisClient.get(self.RGB_TSTAMP_KEY)
    
    def frames_rgb(self):
        """Get rgb frames from redis database. 
        """
        image_dict = {}
        image_dict['rgb'] = self._get_rgb_tstamp()
        image_dict['rgb_tstamp'] = self._get_rgb_timestamp
        return image_dict

    def _get_depth_frame(self):
        """Return depth frame as a numpy array. 
        """
        encoded_depth = self.redisClient.get(self.DEPTH_KEY)
        depth_np = RU.convert_encoded_frame_to_np(
        encoded_depth, RosCameraInterface.DEPTH_DIM)

        return depth_np

    def frames_depth(self):
        """Get depth frames from redis database. 
        """

        image_dict = {}
        image_dict['depth'] = self._get_depth_frame()
        return image_dict

    def frames(self):
        """Return dict with rgb and depth images. 

        Returns: 
            dictionary containing ndarray of frames as values. 

        Keys: 'rgb' : ndarray of rgb frame
              'depth': ndarray of depth frame
              'rgb_tstamp': timestamp of rgb frame from ROS msg.

        """
        images_raw = self.redisClient.mget_dict(
            [self.RGB_KEY, 
             self.RGB_TSTAMP_KEY,
             self.DEPTH_KEY])

        rgb_raw = images_raw[self.RGB_KEY]
        rgb_tstamp = images_raw[self.RGB_TSTAMP_KEY]
        depth_raw = images_raw[self.DEPTH_KEY]

        image_dict = {}
        image_dict['depth'] = RU.convert_encoded_frame_to_np(
        depth_raw, RosCameraInterface.DEPTH_DIM)
        image_dict['rgb'] = RU.convert_encoded_frame_to_np(
            rgb_raw, RosCameraInterface.RGB_DIM)
        image_dict['rgb_tstamp'] = rgb_tstamp

        return image_dict 

    def reset(self):
        pass

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import time
    import numpy as np
    import argparse

    parser = argparse.ArgumentParser(description='Test ROS Camera Interface')
    parser.add_argument('cameras', metavar='camera1', nargs='+',
                        help='list of camera rostopic names to test.')

    args = parser.parse_args()

    camera_interfaces = {}
    for camera_name in args.cameras:
        camera_interfaces[camera_name] = RosCameraInterface(camera_name)

    try: 
        for i in range(5):
            all_frames = {}
            combined_frames = None
            for camera_name, camera in camera_interfaces.items():
                frame = camera.frames()
                all_frames[camera_name] = frame
                if combined_frames is None:
                    combined_frames = frame['rgb']
                else: 
                    combined_frames = np.hstack((combined_frames, frame['rgb']))

            plt.imshow(combined_frames)
            plt.show()

    except KeyboardInterrupt:
        pass           
