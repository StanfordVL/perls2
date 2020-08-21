"""Class definition for Redis Interface.
"""
import redis
import hiredis
import socket
import numpy as np
import json
from perls2.ros_interfaces.redis_keys import *
class RedisInterface():
    """
    Redis interfaces allow for unified access to redis-servers by maintaining consistent keys. 
    Both control interfaces and robot interfaces use redis to communicate and key mismatch can 
    be a source of many bugs. 

    Attributes:
        host (str): name of redis host where server is located
        port (int): identifying port for where server is being hosted. Usually 6379
        pw (str): password to connect to secured server (Necessary for tcp connections.)
        """
    def __init__(self, host, port, password=None):
        # IP address rather than hostname is used to connect to redis. This is an idiosyncracy of our setup. 
        # It is more convenient however, to store the hostname in the config file. So we look it up via sockets.
        self.host = host
        self.port = port
        if 'localhost' not in host:
            self.host = socket.gethostbyname(host)

        setup_kwargs = {'host': self.host, 
                        'port': self.port}
        
        if password is not None: 
            setup_kwargs['password']  = password

        # Connect to redis server.
        self._client = redis.Redis(**setup_kwargs)

# Constant definitions for keys.

def bstr_to_ndarray(array_bstr): 
    """Convert bytestring array to 1d array
    """
    return np.fromstring(array_bstr[1:-1], dtype=np.float, sep = ',')

class RobotRedisInterface(RedisInterface):
    """ Redis interface for robots. 

    Redis interface specifically for robots. This keeps keys consistent between control interfaces and robot interfaces. 

    Attributes: 
        control_type_key (str): type of controller. 
        JOINT_IMPEDANCE (str): 
        cmd_type (str): type of command for robot to execute. Choose from: 
            set_joint_positions 
            move_ee_delta
            reset
        
    """

    def __init__(self, host, port, password=None):
        RedisInterface.__init__(self, host, port, password)

    def _get_key_ndarray(self, key):
        """Return value from desired key, converting bytestring to ndarray
        
            Args: 
                key (str): string of key storing ndarray
        """
        return bstr_to_ndarray(self._client.get(key))

    def _get_key_json(self, key):
        """get json value from desired key, converting it to a dict.
        
            arg:
                key (str): redis key storing json dict
        """

        return json.loads(self._client.get(key))

    def get(self, key):
        if ROBOT_STATE_KEY in key or ROBOT_MODEL_KEY in key:
            return self._get_key_ndarray(key)
        elif CONTROLLER_CONTROL_PARAMS_KEY in key or CONTROLLER_GOAL_KEY in key:
            return self._get_key_json(key)
        else:
            return self._client.get(key)


    def mset(self, key_val_dict):
        """ Set multiple keys to redis at same time. 
            All values for keys must be strings. 
        """
        self._client.mset(key_val_dict)

    def set(self, key, value):
        """ Wrapper for redis.SET cmd
            Args 
                key (str): key for redis database
                value (string, bytes): redis compatible value
        """
        self._client.set(key, value)