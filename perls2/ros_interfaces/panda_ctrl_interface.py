"""Control interface for Panda
"""
import time

from perls2.ros_interfaces.ctrl_interface import CtrlInterface
from perls2.ros_interfaces.panda_redis_keys import PandaKeys
from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.utils.yaml_config import YamlConfig
import logging
logging.basicConfig(level=logging.DEBUG)
import numpy as np

P = PandaKeys('cfg/franka-panda.yaml')
class PandaCtrlInterface(CtrlInterface):
    """Interface for franka-panda redis driver and perls2.RealPandaInterface.

    The Panda Interface gets robot actions from persl2.RealPandaInterface via
    redis. Robot states are obtained from the franka-panda redis driver. The controllers,
    and robot states are combined to calculate torques. The PandaCtrlInterface then updates redis
    with the robot state for RealPandaInterface.

    """
    def __init__(self,
                 config,
                 controlType,
                 driver_config='cfg/franka-panda.yaml'):
        """ Initialize the control interface.
        """
        super().__init__(config, controlType)
        self.redisClient = PandaRedisInterface(**self.config['redis'])
        self.action_set = True

    @property
    def driver_connected(self):
        return self.redisClient.get(P.DRIVER_CONN_KEY) == P.DRIVER_CONNECTED_VALUE

    def step(self, start):
        """Update robot state and model, set torques from controller.
        """
        self.update_model()
        if self.action_set:
            self.set_torque_cmd(np.zeros(7))
        else:
            pass
        #self.set_to_float()
        self.action_set = False

    def update_model(self):
        """get states from redis, update model with these states.
        """
        states = self._get_states_from_redis()
        # self.model.update_states(ee_pos=np.asarray(self.ee_position),
        #                  ee_ori=np.asarray(self.ee_orientation),
        #                  ee_pos_vel=np.asarray(ee_v_world),
        #                  ee_ori_vel=np.asarray(ee_w_world),
        #                  joint_pos=np.asarray(self.q[:7]),
        #                  joint_vel=np.asarray(self.dq[:7]),
        #                  joint_tau=np.asarray(self.tau),
        #                  joint_dim=7,
        #                  torque_compensation=self.torque_compensation)

    def _get_states_from_redis(self):
        """ Get robot states from redis.
    `   """
        states = {}
        for state_key in P.ROBOT_STATE_KEYS:
            states[state_key] = self.redisClient.get(state_key)

        print(states)

    def set_torque_cmd(self, torques):
        logging.debug("setting torque command")
        if not isinstance(torques, np.ndarray):
            torques = np.asarray(torques)
        self.redisClient.set_eigen(P.TORQUE_CMD_KEY, torques)
        self.redisClient.set(P.CONTROL_MODE_KEY, P.TORQUE_CTRL_MODE)
        logging.debug(self.redisClient.get(P.CONTROL_MODE_KEY))

    def set_to_float(self):
        self.redisClient.set(P.CONTROL_MODE_KEY, P.FLOAT_CTRL_MODE)

    def run(self):
        if self.driver_connected:
            logging.info("driver is connected.")

            start = time.time()
            if self.driver_connected:
                self.step(start)
            else:
                #break
                pass
        else:
            print(P.DRIVER_CONN_KEY)
            print(self.redisClient.get(P.DRIVER_CONN_KEY))
            raise ValueError("run driver first.")

if __name__ == '__main__':
    ctrl_interface = PandaCtrlInterface(
        config='cfg/panda_ctrl_config.yaml', controlType=None)
    ctrl_interface.run()