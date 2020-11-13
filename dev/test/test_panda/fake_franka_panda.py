"""Class to mimic franka-panda redis driver.
"""
from perls2.utils.yaml_config import YamlConfig
from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.ros_interfaces.panda_redis_keys import PandaKeys
P = PandaKeys('cfg/franka-panda.yaml')


class FakeFrankaPanda(object):
    """Class for faking panda redis driver.

    Sets fake values for redis, mocking franka-panda.
    """
    def __init__(self):
        self.driver_config = YamlConfig('cfg/franka-panda.yaml')
        # some renaming for the panda redis interface.
        redis_config = {"host": self.driver_config['redis']['ip'],
                        "port": self.driver_config['redis']['port'],
                        "driver_config":'cfg/franka-panda.yaml' }
        self.redisClient = PandaRedisInterface(**redis_config)

    def start(self):
        self.redisClient.set(P.DRIVER_CONN_KEY, P.DRIVER_CONNECTED_VALUE)

    def stop(self):
        self.redisClient.set(P.DRIVER_CONN_KEY, P.DRIVER_DISCONN_VALUE)

    def set_q(self, q):
        """ Set fake joint positions in format franka-panda redis uses

        Args:
            q (list): 7f joint positions.
        """
        self.redisClient.set(P.ROBOT_STATE_Q_KEY, str(q))

    def set_states(self, states):
        """Set robot state via redis, using franka-panda format for strings.
        """
        self.redisClient.mset(
            {P.ROBOT_STATE_Q_KEY: states["q"],
             P.ROBOT_STATE_DQ_KEY: states["dq"],
             P.ROBOT_STATE_EE_POSE_KEY : states["pose"],
             P.ROBOT_STATE_TAU_KEY : states["tau"]}
             )