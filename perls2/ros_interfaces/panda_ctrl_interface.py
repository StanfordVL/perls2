"""Control interface for Panda
"""
import time

from perls2.ros_interfaces.ctrl_interface import CtrlInterface
from perls2.ros_interfaces.panda_redis_keys import PandaKeys
from perls2.ros_interfaces.redis_interface import PandaRedisInterface
from perls2.ros_interfaces.redis_keys import *
from perls2.utils.yaml_config import YamlConfig

from perls2.controllers.utils import transform_utils as T

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
        self.action_set = False
        self._num_joints = 7
        self.CLIP_CMD_TORQUES = [-1.0, 1.0]

        self.neutral_joint_position = self.config['panda']['neutral_joint_angles']

        self.redisClient.set(ROBOT_CMD_TSTAMP_KEY, time.time())
        self.redisClient.set(ROBOT_SET_GRIPPER_CMD_KEY, 0.1)
        self.redisClient.set(ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY, time.time())
        self.last_cmd_tstamp = self.get_cmd_tstamp()
        self.last_gripper_cmd_tstamp = self.get_gripper_cmd_tstamp()
        self.loop_times = []
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
        ee_pos, ee_ori_quat = T.mat2pose(new_states[P.ROBOT_STATE_EE_POSE_KEY])
        update_args = {
        # ee pose is stored in a (4,4) matrix
        'ee_pos' :ee_pos,
        'ee_ori': ee_ori_quat,
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

    def reset_to_neutral(self): 
        """Signal driver to reset to neutral joint positions.
        
        Waits for control mode from panda to be set to idle, then
        notifies Real Panda Interface by setting reset complete flag.

        Blocking.
        """
        start_time = time.time()
        # set command torques to zero in case we resume torque control after reset.
        self.redisClient.set_eigen(P.TORQUE_CMD_KEY, np.zeros(6))
        self.redisClient.set(P.CONTROL_MODE_KEY, P.RESET_CTRL_MODE)
        while (self.redisClient.get(P.CONTROL_MODE_KEY).decode() != P.IDLE_CTRL_MODE):
            time.sleep(1)
        self.redisClient.set(ROBOT_RESET_COMPL_KEY, 'True')


    def set_torques(self, torques):
        """Set torque command to redis driver.

        Args:
            torques (list or ndarray): 7f list of joint torques to command.

        """
        #logging.debug("setting torque command")
        self.redisClient.set_eigen(P.TORQUE_CMD_KEY, torques)
        self.redisClient.set(P.CONTROL_MODE_KEY, P.TORQUE_CTRL_MODE)
        #logging.debug(self.redisClient.get(P.TORQUE_CMD_KEY))

    def set_to_float(self):
        self.redisClient.set(P.CONTROL_MODE_KEY, P.FLOAT_CTRL_MODE)

    def step(self, start):
        """Update the robot state and model, set torques from controller

        Note this is different from other robot interfaces, as the Sawyer
        low-level controller takes gravity compensation into account.
        """
        self.update_model()

        if self.action_set:
            
            torques = self.controller.run_controller()
            
            self.set_torques(np.clip(torques, -2.5, 2.5))

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

    def set_dummy_torque_redis(self):
        zero_torques = (np.random.rand(7) - 0.5)*0.00001
        zero_torques = np.clip(zero_torques, -0.001, 0.001)
        
        self.set_torques(zero_torques)

    def warm_up_driver(self):
        logging.info("warming up driver...setting to float")
        logging.info("setting torque command key to  very small random values.")
        
        self.set_to_float()
        assert(self.redisClient.get(P.CONTROL_MODE_KEY).decode() == P.FLOAT_CTRL_MODE)

        for _ in range(1000):
            zero_torques = (np.random.rand(7) - 0.5)*0.00001
            zero_torques = np.clip(zero_torques, -0.001, 0.001)
            
            self.redisClient.set_eigen(P.TORQUE_CMD_KEY, zero_torques)



    def run(self):
        if not self.driver_connected:
            raise ValueError("franka-panda driver must be started first.")
        
        self.warm_up_driver()

        self.update_model()
        self.wait_for_env_connect()
        self.controller = self.make_controller_from_redis(self.get_control_type(), self.get_controller_params())        
        logging.info("Beginning control loop")
        try:    
            while True:
                start = time.time()

                if (self.redisClient.is_env_connected()):
                    if self.check_for_new_cmd():
                        self.process_cmd(self.cmd_type)
                    # if self.check_for_new_gripper_cmd():
                    #     self.process_gripper_cmd()
                    self.step(start)

                else:
                    break
        except KeyboardInterrupt:
            np.savez('dev/test/loop_times.npz', loop=self.loop_times, allow_pickle=True)






if __name__ == '__main__':
    ctrl_interface = PandaCtrlInterface(
        config='cfg/panda_ctrl_config.yaml', controlType=None)
    ctrl_interface.run()