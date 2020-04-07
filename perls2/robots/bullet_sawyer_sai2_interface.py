"""
Class defining the interface to the Sawyer Robots in PyBullet.
Uses Sai2 Python for mass Matrix
Author: Rohun Kulkarni
"""

import pybullet
import numpy as np
import sai2python as sai2

from perls2.robots.bullet_sawyer_interface import BulletSawyerInterface

class BulletSawyerSai2Interface(BulletSawyerInterface):
    """ Class for Sawyer Robot Interface in Pybullet. This class provides the
    functionsfor information about the state of the robot as well as sending
    commands.


    """

    def __init__(self,
                 physics_id,
                 arm_id,
                 config=None,
                 controlType=None):
        """ Set up the sai2 model
        """

        super().__init__(physics_id, arm_id, config, controlType)
        urdf_name = '/vision2/u/rohunk/sai2_python/resources/sawyer/sawyer.urdf'
        self.model = sai2.sai2python(urdf_name, 'sawyer')

    # @property
    # def mass_matrix(self):
    #     mass_matrix = np.zeros(49)
    #     q_pos = np.asarray(self.motor_joint_positions)
    #     self.model.mass_matrix(q_pos[:7], mass_matrix)
    #     return mass_matrix.reshape(7,7)