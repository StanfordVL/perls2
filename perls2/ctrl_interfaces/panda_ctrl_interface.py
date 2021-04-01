"""Control interface for Panda
"""
import sys
import os
import socket
import signal   # Catch interrupts from bash scripts
import time
import numpy as np
import json
from perls2.ctrl_interfaces.ctrl_interface import CtrlInterface
import perls2.redis_interfaces.panda_redis_keys as P
from perls2.redis_interfaces.redis_interface import PandaRedisInterface
from redis.exceptions import ConnectionError as RedisConnectionError
import perls2.redis_interfaces.redis_keys as R
from perls2.controllers.utils import transform_utils as T
import perls2.utils.redis_utils as RU
import logging
logging.basicConfig(level=logging.DEBUG)


ENV_CHECK_COUNT = 100   # how often we check if workstation is connected.
LOOP_TIME_REPORT_COUNT = 1000
MAX_TORQUE = 1.5        # Clip torque commands before sending to driver.
MIN_TORQUE = -1.5

def keyboardInterruptHandler(signal, frame):
    print("KeyboardInterrupt: Exiting Panda Ctrl Interface".format(signal))
    sys.exit(0)

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
                 redis_passfile=None):
        """ Initialize the control interface.
        """
        super().__init__(config, controlType)
        
        # overwrite config passfile if given as argument.
        if redis_passfile is not None:
            self.config['redis']['password'] = redis_passfile

        self.redisClient = PandaRedisInterface(**self.config['redis'])
        self.action_set = False
        self._num_joints = 7
        self.CLIP_CMD_TORQUES = [-1.0, 1.0]

        # Set default reset joint angle for driver and workstation key
        self.neutral_joint_angles = np.array(self.config['panda']['neutral_joint_angles'])
        self.redisClient.set_reset_q(P.RESET_Q_KEY, self.neutral_joint_angles)
        self.redisClient.set_reset_q(R.ROBOT_RESET_Q_KEY, self.neutral_joint_angles)

        self.redisClient.set(R.ROBOT_CMD_TSTAMP_KEY, time.time())
        self.redisClient.set(R.ROBOT_SET_GRIPPER_CMD_KEY, 0.1)
        self.redisClient.set(R.ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY, time.time())
        self.last_cmd_tstamp = self.get_cmd_tstamp()
        self.last_gripper_cmd_tstamp = self.get_gripper_cmd_tstamp()
        self.prev_gripper_state = self.des_gripper_state
        self.loop_times = []
        self.torque_calc_time = -1
        self.zero_torques = np.zeros(7)
        # TODO: reset to neutral position.
        # Get state from redis
        # Update model.
        self.start_tstamp = []
        self.update_model_tstamp = []
        self.set_torques_tstamp = []
        self.end_tstamp = []

    def prepare_to_run(self):
        """ Perform steps before running CtrlInterface.
        """
    @property
    def num_joints(self):
        return self._num_joints

    @property
    def driver_connected(self):
        return self.redisClient.get(P.DRIVER_CONN_KEY).decode() == P.DRIVER_CONNECTED_VALUE

    def _get_update_args(self, new_states):
        """Reformat the states so they are compatible with the Model class.
        """

        # ee pose is stored in a (4,4) matrix
        ee_pos, ee_ori_quat = T.mat2pose(new_states[P.ROBOT_STATE_EE_POSE_KEY])
        update_args = {
            'ee_pos': ee_pos,
            'ee_ori': ee_ori_quat,
            'joint_pos': new_states[P.ROBOT_STATE_Q_KEY],
            'joint_vel': new_states[P.ROBOT_STATE_DQ_KEY],
            'joint_tau': new_states[P.ROBOT_STATE_TAU_KEY],
            'J_full': new_states[P.ROBOT_MODEL_JACOBIAN_KEY],
            'mass_matrix': new_states[P.ROBOT_MODEL_MASS_MATRIX_KEY]}
        return update_args

    def update_model(self):
        """get states from redis, update model with these states.
        """
        new_states = self.redisClient.get_driver_state_model()

        self.model.update_state_model(**self._get_update_args(new_states))

    def _update_reset_angles(self):
        """Update and set neutral joint positions from WS

        TODO: add check for joint limits.
        """
        # Update neutral joint positions from workstation. 
        self.neutral_joint_angles = self.redisClient.get_reset_q()
        self.redisClient.set_reset_q(P.RESET_Q_KEY, self.neutral_joint_angles)

    def reset_to_neutral(self):
        """Signal driver to reset to neutral joint positions.

        Waits for control mode from panda to be set to idle, then
        notifies Real Panda Interface by setting reset complete flag.

        Blocking.
        """
        # set command torques to zero in case we resume torque control after reset.
        self.redisClient.set_eigen(P.TORQUE_CMD_KEY, np.zeros(7))
        
        # Get neutral joint angles from workstation.
        self._update_reset_angles()
        self.redisClient.set(P.CONTROL_MODE_KEY, P.RESET_CTRL_MODE)

        # Wait for reset to complete.
        while (self.redisClient.get(P.CONTROL_MODE_KEY).decode() != P.IDLE_CTRL_MODE):
            time.sleep(1)
        self.redisClient.set(R.ROBOT_RESET_COMPL_KEY, 'True')
        self.action_set = False

    def set_torques(self, torques):
        """Set torque command to redis driver.

        Args:
            torques (list or ndarray): 7f list of joint torques to command.

        """
        torque_cmd_str = RU.ndarray_to_eigen_str(torques)
        self.redisClient.mset({
            P.TORQUE_CMD_KEY: torque_cmd_str,
            P.CONTROL_MODE_KEY: P.TORQUE_CTRL_MODE})

    def set_to_float(self):
        self.redisClient.set(P.CONTROL_MODE_KEY, P.FLOAT_CTRL_MODE)

    def close_gripper(self):
        """
        Close the gripper of the robot
        :return: None
        """
        raise NotImplementedError

    def open_gripper(self):
        """
        Open the gripper of the robot
        :return: None
        """
        raise NotImplementedError

    def set_gripper_to_value(self, value):
        """
        Set the gripper grasping opening state
        :param openning: a float between 0 (closed) and 1 (open).
        :return: None
        """

        self.redisClient.set(P.GRIPPER_WIDTH_CMD_KEY, value)

        self.redisClient.set(P.GRIPPER_MODE_KEY, "move")

    def step(self, start):
        """Update the robot state and model, set torques from controller

        Note this is different from other robot interfaces, as the Sawyer
        low-level controller takes gravity compensation into account.
        """

        self.update_model()

        if self.action_set:
            torques = self.controller.run_controller()
            self.set_torques(np.clip(torques, MIN_TORQUE, MAX_TORQUE))
        else:
            pass

    def get_gripper_cmd_tstamp(self):
        return self.redisClient.get(R.ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY)

    @property
    def des_gripper_state(self):
        return float(self.redisClient.get(R.ROBOT_SET_GRIPPER_CMD_KEY))

    def check_for_new_gripper_cmd(self):
        gripper_cmd_tstamp = self.get_gripper_cmd_tstamp()
        if self.last_gripper_cmd_tstamp is None or (self.last_gripper_cmd_tstamp != gripper_cmd_tstamp):
            self.last_gripper_cmd_tstamp = gripper_cmd_tstamp
            return True
        else:
            return False

    def process_gripper_cmd(self, cmd_data):
        """Process the new gripper command by setting gripper state.

        Only send gripper command if current gripper open fraction is not
        equal to desired gripper open fraction.
        """
        if self.prev_gripper_state != cmd_data:
            self.set_gripper_to_value(cmd_data)
            self.prev_gripper_state = cmd_data
        else:
            pass

    def wait_for_env_connect(self):
        """Blocking code that waits for perls2.RobotInterface to connect to redis.

        Allows for ctrl+c key board interrputs/
        """
        logging.info("Waiting for perls2.RealRobotInterface to connect to redis.")
        while True:
            if not self.redisClient.is_env_connected():
                pass
            else:
                break

        if self.redisClient.is_env_connected():
            logging.info("perls2.RealRobotInterface connected.")

    def set_dummy_torque_redis(self):
        """Set random very small nonzero torque to redis.

        Used to test sending torque commands and measuring latency in
        communicating with the driver.
        """
        zero_torques = (np.random.rand(7) - 0.5) * 0.00001
        zero_torques = np.clip(zero_torques, -0.001, 0.001)

        self.set_torques(zero_torques)

    def set_zero_torques_redis(self):
        """Set zero torque command to driver.
        """
        self.set_torques(self.zero_torques)

    def check_driver_float(self):
        """Confirm driver is in floating mode.
        """
        return self.redisClient.get(P.CONTROL_MODE_KEY).decode() == P.FLOAT_CTRL_MODE

    def warm_up_driver(self):
        """Send many zero commands to redis to reduce latencies.

        Redis latency spikes tend to occur in the beginning of driver operation.
        This is a bit of a hack to try and query the database many times and
        hopefully get over any intialization slow down.
        """
        logging.info("warming up driver...setting to float")
        logging.info("setting torque command key to  very small random values.")

        self.set_to_float()
        assert(self.check_driver_float())

        for _ in range(5000):
            zero_torques = (np.random.rand(7) - 0.5) * 0.00001
            zero_torques = np.clip(zero_torques, -0.001, 0.001)

            self.redisClient.set_eigen(P.TORQUE_CMD_KEY, zero_torques)

    def get_cmd_data(self):
        """Get robot and gripper command using mget.
        """
        cmd_data = self.redisClient.mget_dict([R.ROBOT_CMD_TSTAMP_KEY,
                                               R.ROBOT_CMD_TYPE_KEY,
                                               R.ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY,
                                               R.ROBOT_SET_GRIPPER_CMD_KEY])

        return cmd_data

    def process_cmd_data(self, cmd_data):
        """Process command data by checking if new command.
        """
        # Process new robot command.
        if self.last_cmd_tstamp is None or (self.last_cmd_tstamp != cmd_data[R.ROBOT_CMD_TSTAMP_KEY]):
            self.last_cmd_tstamp = cmd_data[R.ROBOT_CMD_TSTAMP_KEY]
            self.process_cmd(cmd_data[R.ROBOT_CMD_TYPE_KEY])

        # Process new gripper command.
        if self.last_gripper_cmd_tstamp is None or (self.last_gripper_cmd_tstamp != cmd_data[R.ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY]):
            self.last_gripper_cmd_tstamp = cmd_data[R.ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY]
            self.process_gripper_cmd(cmd_data[R.ROBOT_SET_GRIPPER_CMD_KEY])

    def check_env_connection(self, loop_count):
        """ Check if workstation is connected every few loops. (Control loops should run on order of ms)
        This reduces redis queries.
        """
        if (loop_count % ENV_CHECK_COUNT == 0):
            return self.redisClient.is_env_connected()
        else:
            return True

    def run(self):
        """Run main control loop for ctrl interface.

        Grab and process new commands from redis, update the model
        and send torques to the libfranka driver.
        """
        signal.signal(signal.SIGINT, keyboardInterruptHandler)
        try:
            if not self.driver_connected:
                raise ValueError("franka-panda driver must be started first.")

            self.warm_up_driver()
            self.update_model()
            logging.info("Waiting for perls2.RealRobotInterface to connect to redis.")
            while True:
                if not self.redisClient.is_env_connected():
                    pass
                else:
                    break

            if self.redisClient.is_env_connected():
                logging.info("perls2.RealRobotInterface connected.")

            self.controller = self.make_controller_from_redis(self.get_control_type(), self.get_controller_params())
            logging.info("Beginning control loop")
            self.loop_count = 0
            while True:
                start = time.time()
                if (self.check_env_connection(self.loop_count)):
                    self.process_cmd_data(self.get_cmd_data())
                    self.step(start)

                    while ((time.time() - start) < 0.001):
                        pass

                    self.loop_count += 1

                else:
                    break
    
        except (RedisConnectionError):
            print("Connection error: redis-server disconnected. ")
        finally: 
            print("Shutting down perls.PandaCtrlInterface")



    def run_dummy(self):
        """ Run control loop in dummy mode.

        Helpful for testing driver <-> ctrl_interface.
        Sends very small dummy torques to measure driver latency.
        """
        logging.info("Running contrl interface in dummy mode.")
        if not self.driver_connected:
            raise ValueError("franka-panda driver must be started first.")

        self.warm_up_driver()
        self.update_model()

        self.default_control_type = self.config['controller']['selected_type']
        self.default_params = self.config['controller']['Real'][self.default_control_type]
        self.redisClient.set(R.CONTROLLER_CONTROL_TYPE_KEY, self.default_control_type)
        self.redisClient.set(R.CONTROLLER_CONTROL_PARAMS_KEY, json.dumps(self.default_params))

        self.controller = self.make_controller_from_redis(self.get_control_type(), self.get_controller_params())

        logging.info("Beginning dummy control loop")

        try:
            while True:
                start = time.time()
                self.set_dummy_torque_redis()
                while ((time.time() - start) < 0.001):
                    pass

        except KeyboardInterrupt:
            pass



if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Run Control Interface for Franka Panda')
    parser.add_argument('--dummy', action="store_true", help="Run Control Interface in dummy mode, sending very small torques to driver only.")
    parser.add_argument('--config', default='cfg/panda_ctrl_config.yaml', help="panda control yaml config filepath")
    parser.add_argument('--redis_passfile', default=None, help="filepath for redis password")
    args = parser.parse_args()
    kwargs = vars(args)

    ctrl_interface = PandaCtrlInterface(
        config=kwargs['config'], controlType=None, redis_passfile=kwargs['redis_passfile'])

    if (kwargs['dummy']):
        ctrl_interface.run_dummy()
    else:
        ctrl_interface.run()
