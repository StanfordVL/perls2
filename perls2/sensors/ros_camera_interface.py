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


    def frames_rgb(self):
        """Get rgb frames from redis database. 
        """
        encoded_rgb = self.redisClient.get(self.RGB_KEY)
        rgb_timestamp = self.redisClient.get(self.RGB_TSTAMP_KEY)

        rgb_np = RU.convert_encoded_frame_to_np(
            encoded_rgb, RosCameraInterface.RGB_DIM)
        image_dict = {}
        image_dict['rgb'] = rgb_np
        image_dict['rgb_tstamp'] = rgb_timestamp
        return image_dict

    def frames_depth(self):
        """Get depth frames from redis database. 
        """
        encoded_depth = self.redisClient.get(self.DEPTH_KEY)
        depth_np = RU.convert_encoded_frame_to_np(
            encoded_depth, RosCameraInterface.DEPTH_DIM)
        image_dict = {}
        image_dict['depth'] = depth_np
        return image_dict

    def reset(self):
        pass

if __name__ == '__main__':
    import cv2
    import time
    camera_interface = RosCameraInterface('sr300')
    try: 
        for i in range(10000):
            #frame = camera_interface.frames_rgb()
            frame = camera_interface.frames_depth()
            # print(rgb_frame['rgb_tstamp'])
            # time.sleep(1)
            cv2.imshow('rgb', frame['depth'])
            cv2.waitKey(1000)
            cv2.destroyAllWindows()

    except KeyboardInterrupt:
        pass