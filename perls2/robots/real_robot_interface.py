"""
Abstract class defining the interface to the robots.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import abc   # For abstract class definitions
import six   # For abstract class definitions
import redis  # For communicating with RobotCtrlInterfaces
from scipy.spatial.transform import Rotation as R
from perls2.robots.robot_interface import RobotInterface
import numpy as np
import logging
import json 
import time
from perls2.ros_interfaces.redis_keys import * 
from perls2.ros_interfaces.redis_interface import RobotRedisInterface
AVAILABLE_CONTROLLERS = ["EEImpedance",
                         "EEPosture",  
                         "Internal",
                         "JointVelocity",
                         "JointImpedance", 
                         "JointTorque"]

class RealRobotInterface(RobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self, config, controlType=None):
        """ Initialize the Real Robot Interface.

            Args:
                config (dict) : parsed YAML dictionary with config information
                controlType (str): string identifying controller for robot. Must be in
                    AVAILABLE CONTROLLERS

        """
        super().__init__(config=config, controlType=controlType )
        self.config = config
        self.robot_cfg  = self.config[self.config['world']['robot']]

    def create(config, controlType):
        """Factory for creating robot interfaces based on config type

            Args: 
                config (dict): dictionary containing robot information
                controlType (str): string id for type of controller. Must be in 
                    AVAILABLE CONTROLLERS. 

            returns: 
                RobotInterface 

        """
        if (config['world']['robot'] == 'sawyer'):
            from perls2.robots.real_sawyer_interface import RealSawyerInterface
            return RealSawyerInterface(
                config=config, controlType=controlType)
        else:
            raise ValueError("invalid robot interface type. choose 'sawyer'")
    

    def step(self):
        """Compatability only

            Actions are executed by CtrlInterface, so here do nothing.
        """
        pass

    def update_model(self):
        """Compatability only

            Model for Controller updated by Controller in CtrlInterface. do nothing.
        """
        pass
        
    def set_control_params(self):
        """Set the control parameters key on redis
        """
        control_config = self.config['controller']['Real'][self.control_type]
        self.redisClient.set(CONTROLLER_CONTROL_PARAMS_KEY, json.dumps(control_config))
  
    def change_controller(self, next_type):
        """Change to a different controller type.
        Args: 
            next_type (str): keyword for desired control type. 
                Choose from: 
                    -'EEImpedance'
                    -'JointVelocity'
                    -'JointImpedance'
                    -'JointTorque'
        """ 
        if next_type in AVAILABLE_CONTROLLERS:            
            # self.controller = self.make_controller(next_type)
            self.controlType = next_type
            self.redisClient.set(CONTROLLER_CONTROL_TYPE_KEY, next_type)
            control_config = self.config['controller']['Real'][self.controlType]
            self.redisClient.set(CONTROLLER_CONTROL_PARAMS_KEY, json.dumps(control_config))
            return self.controlType
        else:
            raise ValueError("Invalid control type " + 
                "\nChoose from EEImpedance, JointVelocity, JointImpedance, JointTorque")

    def set_joint_positions(self, pose, **kwargs):
        """ Set goal as joint positions for robot by sending robot command.
        
            Args: 
                pose (7f): list of joint positions to set as controller goal. 

            Returns: None

            Example:

                env.robot_interface.set_joint_positions([0,-1.18,0.00,2.18,0.00,0.57,3.3161])

            Notes: 
                Does not check if desired joint position within robot limits. 
                TODO: add this check.
                Does not check if desired joint position appropriate dims. 
                TODO: add this check.
        """    
        self.check_controller("JointImpedance")
        kwargs = {'cmd_type': "set_joint_positions", 'delta':None,'set_qpos':pose}
        self.set_controller_goal(**kwargs)

    def set_joint_delta(self, delta, **kwargs):
        """ Set goal as delta from current joint position by sending robot command.

            Args 
                delta (7f): list of delta from current joint position to set as goal. 

            Returns: None

            Example:
                env.robot_interface.set_joint_delta([0.1, 0.2, 0.1, 0.05, -0.05, 0, 0.0])

            Notes: 
                Does not check if desired joint position within robot limits. 
                TODO: add this check.
                Does not check if desired joint position appropriate dims. 
                TODO: add this check.
        """ 
        self.check_controller("JointImpedance")
        kwargs['delta'] = delta.tolist()
        kwargs['cmd_type'] = "set_joint_delta"
        self.set_controller_goal(**kwargs)

    def move_ee_delta(self, delta, set_pos=None, set_ori=None):
        """ Use controller to move end effector by some delta.

        Args: 
            delta (6f): delta position (dx, dy, dz) concatenated with delta orientation.
                Orientation change is specified as an Euler angle body XYZ rotation about the
                end effector link. 
            set_pos (3f): end effector position to maintain while changing orientation. 
                [x, y, z]. If not None, the delta for position is ignored. 
            set_ori (4f): end effector orientation to maintain while changing orientation
                as a quaternion [qx, qy, qz, w]. If not None, any delta for orientation is ignored. 
        
        Note: only for use with EE impedance controller
        Note: to fix position or orientation, it is better to specify using the kwargs than
            to use a 0 for the corresponding delta. This prevents any error from accumulating in 
            that dimension. 

        """
        #self.check_controller("EEImpedance")
        if set_ori is not None:
            if len(set_ori) != 4:
                raise ValueError('set_ori incorrect dimensions, should be quaternion length 4')
            set_ori= T.quat2mat(set_ori)
        if set_pos is not None:
            if len(set_pos) != 3:
                raise ValueError('set_pos incorrect dimensions, should be length 3')

        kwargs = {'cmd_type': "move_ee_delta", 'delta': delta.tolist(), 'set_pos': set_pos, 'set_ori':set_ori}
        self.set_controller_goal(**kwargs)

    def set_controller_goal(self, cmd_type, **kwargs): 
        """ Set the appropriate redis keys for the robot command. Internal use only.
            
            Sets the command timestamp, command type and controller goal.
            This version is specific for real robots. 

            Args: 
                cmd_type (str): string identifier for fn to execute. Must exactly match
                    function name for CtrlInterface. 

                kwargs (dict): dictionary containing keys specific to robot command. 

            Returns: None

            Example: 
                        delta = [0., 0., 0.1, 0., 0., 0., 0.1]
                        kwargs['delta'] = delta
                        kwargs['cmd_type'] = "set_joint_delta"
                        self.set_controller_goal(**kwargs)

        """
        logging.debug("cmd_type {}".format(cmd_type))

        control_cmd = { ROBOT_CMD_TSTAMP_KEY: time.time(),
                        ROBOT_CMD_TYPE_KEY : cmd_type, 
                        CONTROLLER_GOAL_KEY : json.dumps(kwargs)}
        self.redisClient.mset(control_cmd)
        self.action_set = True
