"""Control interface for Panda
"""
import time

from perls2.ros_interfaces.ctrl_interface import CtrlInterface
from perls2.ros_interfaces.panda_redis_keys import PandaKeys
from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.robots.real_panda_interface import RealPandaInterface
from perls2.utils.yaml_config import YamlConfig
import logging
logging.basicConfig(level=logging.DEBUG)
import numpy as np


P = PandaKeys('cfg/franka-panda.yaml')

LOOP_LATENCY = 0.000
LOOP_TIME = (1.0 / 1000.0) - LOOP_LATENCY

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
        self.action_set = False
        self._num_joints = 7
        self.CLIP_CMD_TORQUES = [-1.0, 1.0]

        self.neutral_joint_position = self.config['panda']['neutral_joint_angles']

        # TODO: reset to neutral position.
        # Get state from redis
        # Update model.

    def prepare_to_run(self):
        """ Perform steps before running CtrlInterface.
        """
    @property
    def num_joints(self):
        return self._num_joints

    @property
    def driver_connected(self):
        return self.redisClient.get(P.DRIVER_CONN_KEY) == P.DRIVER_CONNECTED_VALUE
    

    def _get_update_args(self, new_states):
        """Reformat the states so they are compatible with the Model class.
        """
        update_args = {
        # ee pose is stored in a (4,4) matrix
        'ee_pos' : new_states[P.ROBOT_STATE_EE_POSE_KEY][:,3][0:3],
        'ee_ori': new_states[P.ROBOT_STATE_EE_POSE_KEY][0:3, 0:3],
        'joint_pos': new_states[P.ROBOT_STATE_Q_KEY],
        'joint_vel' :new_states[P.ROBOT_STATE_DQ_KEY],
        'joint_tau' :new_states[P.ROBOT_STATE_TAU_KEY],
        'J_full': new_states[P.ROBOT_MODEL_JACOBIAN_KEY],
        'mass_matrix': new_states[P.ROBOT_MODEL_MASS_MATRIX_KEY],
         }
        return update_args


    def update_model(self):
        """get states from redis, update model with these states.
        """
        new_states = self.redisClient.get_driver_state_model()

        self.model.update_state_model(**self._get_update_args(new_states))


    def set_torques(self, torques):
        """Set torque command to redis driver.

        Args:
            torques (list or ndarray): 7f list of joint torques to command.

        """
        logging.debug("setting torque command")
        assert len(torques) == self.num_joints, \
            'Input number of torque values must match the number of joints'

        if not isinstance(torques, np.ndarray):
            torques = np.asarray(torques)
        self.redisClient.set_eigen(P.TORQUE_CMD_KEY, torques)
        self.redisClient.set(P.CONTROL_MODE_KEY, P.TORQUE_CTRL_MODE)
        logging.debug(self.redisClient.get(P.CONTROL_MODE_KEY))

    def set_to_float(self):
        self.redisClient.set(P.CONTROL_MODE_KEY, P.FLOAT_CTRL_MODE)

    def step(self, start):
        """Update the robot state and model, set torques from controller

        Note this is different from other robot interfaces, as the Sawyer
        low-level controller takes gravity compensation into account.
        """
        self.update_model()

        if self.action_set:
            #torques = self.controller.run_controller()
            self.set_torques([0]*7)

            torques = np.clip(torques, self.CLIP_CMD_TORQUES[0], self.CLIP_CMD_TORQUES[1] )
            print("Torques {}".format(torques))

            #self.set_torques(torques)

            while (time.time() - start < LOOP_TIME):
                pass

        else:
            pass
    
    def wait_for_env_connect(self): 
        """Blocking code that waits for perls2.RobotInterface to connect to redis.
        
        Allows for ctrl+c key board interrputs/
        """
        logging.info("Waiting for perls2.RealRobotInterface to connect to redis.")
        try: 
            while True: 
                if not self.redisClient.is_env_connected():
                    pass
                else:
                    break
        except KeyboardInterrupt:
            logging.error("Keyboard interrupt received.")

        if self.redisClient.is_env_connected():
            logging.info("perls2.RealRobotInterface connected.")

    def run(self):
        if not self.driver_connected:
            raise ValueError("franka-panda driver must be started first.")

        self.wait_for_env_connect()




if __name__ == '__main__':
    ctrl_interface = PandaCtrlInterface(
        config='cfg/panda_ctrl_config.yaml', controlType=None)
    ctrl_interface.run()