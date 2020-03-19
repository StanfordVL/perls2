"""The parent class for Arenas encapsulating robots, sensors and objects.
"""
import numpy as np
import yaml
from perls2.arenas.arena import Arena
import pybullet
import os
import logging


class RealArena(Arena):
    """The class definition for real world arenas.
    Loads pybullet models for IK.
    Arenas contain interfaces for robots, sensors and objects.
    """

    def __init__(self,
                 config,
                 physics_id):
        """ Initialization function.

        Parameters
        ----------
        config: dict
            A dict with config parameters
        robot_interface:
            a robot to place in this env
        sensor_interface:
            a sensor to place in this env e.g. camera
        object_interface:
            an interface to the object in ths environment.
        control_type (optional):
            control_type for the robot to perform actions specified by policy
        key:
            The key for running multiple simulations in parallel.
        debug:
            If it is debugging.
        """
        self.config = config
        self.data_dir = self.config['data_dir']
        # Get the robot config dict by using the name of the robot
        # as a key. The robot config yaml should be included at
        # project config file level.
        robot_name = self.config['world']['robot']
        self.robot_cfg = self.config[robot_name]

        self.physics_id = physics_id
        robot_name = self.config['world']['robot']
        self.robot_cfg = self.config[robot_name]
        #print(self.robot_cfg['arm']['path'])

        self.arm_id, self.base_id = self.load_robot()
        # Get the robot config dict by using the name of the robot
        # as a key. The robot config yaml should be included at
        # project config file level.


    def load_robot(self):
        """ Load the robot and return arm_id, base_id
        """
        arm_file = os.path.join(
            self.data_dir, self.robot_cfg['arm']['path'])

        base_file = os.path.join(
            self.data_dir, self.robot_cfg['base']['path'])

        arm_id = pybullet.loadURDF(
            fileName=arm_file,
            basePosition=self.robot_cfg['arm']['pose'],
            baseOrientation=pybullet.getQuaternionFromEuler(
                self.robot_cfg['arm']['orn']),
            globalScaling=1.0,
            useFixedBase=self.robot_cfg['arm']['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)

        logging.debug("Loaded robot" + " arm_id :" + str(arm_id))

        # Load Arm
        base_id = pybullet.loadURDF(
            fileName=base_file,
            basePosition=self.robot_cfg['base']['pose'],
            baseOrientation=pybullet.getQuaternionFromEuler(
                self.robot_cfg['base']['orn']),
            globalScaling=1.0,
            useFixedBase=self.robot_cfg['base']['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)

        return (arm_id, base_id)

    @property
    # TODO: should this be a part of the real arena?
    def goal_position(self):
        goal = self.randomize_param(self.config['goal_position'])
        return goal
