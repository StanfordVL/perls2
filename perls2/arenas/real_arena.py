"""The parent class for Arenas encapsulating robots, sensors and objects.
"""
from perls2.arenas.arena import Arena
import perls2
import os

class RealArena(Arena):
    """The class definition for real world arenas.
    Loads pybullet models for IK.
    Arenas contain interfaces for robots, sensors and objects.
    """

    def __init__(self,
                 config):
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
        perls2_path = os.path.dirname(perls2.__path__[0])
        self.perls2_data_dir = os.path.join(perls2_path, 'data')
        if 'data_dir' not in self.config:
            data_dir = self.perls2_data_dir
        else:
            data_dir = self.config['data_dir']

        # Get the robot config dict by using the name of the robot
        # as a key. The robot config yaml should be included at
        # project config file level.
        robot_name = self.config['world']['robot']
        self.robot_cfg = self.config[robot_name]
