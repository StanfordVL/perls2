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

class RealRobotInterface(RobotInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 config,
                 controlType=None):

        super().__init__(config=config, controlType=controlType )
        self.config = config
        self.robot_cfg  = self.config[self.config['world']['robot']]

        self.update()
        self.controller = self.make_controller(controlType)


    def create(config, controlType):
        """Factory for creating robot interfaces based on config type
        """
        if (config['world']['robot'] == 'sawyer'):
            from perls2.robots.real_sawyer_interface import RealSawyerInterface
            return RealSawyerInterface(
                config=config, controlType=controlType)
        else:
            raise ValueError("invalid robot interface type. choose 'sawyer'")
    
    def step(self):
        """Update the robot state and model, set torques from controller
        """
        self.update()
        if self.action_set:               
            torques = self.controller.run_controller() 
            self.set_torques(torques)
        else:
            print(" real robot ACTION NOT SET")

    def update(self):
        orn = R.from_quat(self.ee_orientation)
        self.model.update_states(ee_pos=np.asarray(self.ee_position),
                                 ee_ori= np.asarray(self.ee_orientation),
                                 ee_pos_vel=np.asarray(self.ee_v),
                                 ee_ori_vel=np.asarray(self.ee_omega),
                                 joint_pos=np.asarray(self.q[:7]),
                                 joint_vel=np.asarray(self.dq[:7]),
                                 joint_tau=np.asarray(self.tau)
                                 )

        self.model.update_model(J_pos=self.linear_jacobian,
                                J_ori=self.angular_jacobian,
                                mass_matrix=self.mass_matrix)
    
