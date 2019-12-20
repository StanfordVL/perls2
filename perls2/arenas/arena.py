"""The parent class for Arenas encapsulating robots, sensors and objects.
"""
import numpy as np
import yaml
import logging


class Arena:
    """The class definition for arenas
    Arenas contain interfaces for robots, sensors and objects.
    """

    def __init__(self,
                 config):
        """ Initialization function.

            Gets parameters from configuration file for managing experimental
            setup and episodes. These include randomized parameters for
            camera extrinsic/intrinsics, object placement.
        Args:
            config (dict): A dict with config parameters

        Returns:
            None
        """
        self.config = config
        logging.info("Arena Created")
        self.randomize_after_reset = True

        # Load camera parameters
        camera_extrinsics_cfg = self.config['sensor']['camera']['extrinsics']

        self.camera_eye_pos = camera_extrinsics_cfg['eye_position']
        self.camera_target_pos = camera_extrinsics_cfg['target_position']
        self.camera_up_vector = camera_extrinsics_cfg['up_vector']

        # Image parameters
        camera_intrinsics_cfg = self.config['sensor']['camera']['intrinsics']

        self.image_height = camera_intrinsics_cfg['image_height']
        self.image_width = camera_intrinsics_cfg['image_width']
        self.near_plane = camera_intrinsics_cfg['near_plane']
        self.far_plane = camera_intrinsics_cfg['far_plane']
        self.fov = camera_intrinsics_cfg['fov']

        self._rand_camera_intrin_cfg = (
            self.config['sensor']['camera']['random']['intrinsics'])
        self._rand_camera_extrin_cfg = (
            self.config['sensor']['camera']['random']['extrinsics'])
        self._random_obj_cfg = self.config['object']['random']

    def random_vec_bounded(self, lower_bound, upper_bound):
        """Create a xf random position within bounds
            Args:
                lower_bound (ndarray): lower limit for random position x,y,z
                upper_bound (ndarray): upper limitfor random position x,y,z
            Returns:
                random numpy xf within bounds

            Note:  DOES NOT CHECK if upper bounds > lower bounds
        """
        size = np.size(upper_bound)
        return (np.random.random((size,))*(upper_bound - lower_bound) +
                lower_bound)

    def randomize_param(self, config_key):
        """ Randomize parameter according to bounds found in config file

            Args:
                config_key (dict): config dict to random parameter definitions.
                -
            Returns:
                -randomized Xf within the bounds of the upper and lower
                limits specified in the config file
            Example

                random_obj_pos = self.randomize_param(
                    self.config['several']['keys']['leading']['to']['param_to_randomize'])

                cameraEyePosition = self.randomize_param(
                    self.config['sensor']['camera']['random']['intrinsics'])

            NOTE: bounds must be defined in config file as keys 'upper' and
                'lower'
            NOTE: currently no checks on whether upper > lower TODO
            NOTE: does not check if string does not exist.
            NOTE: Does not check that dimensions of upper = lower
            TODO: Add support for default parameters

        """

        upper_bound = np.asarray(config_key['upper'])
        lower_bound = np.asarray(config_key['lower'])

        return self.random_vec_bounded(lower_bound, upper_bound)

    def randomize_obj_pos(self):
        """Randomize camera placement in sim
            Args: None
            Returns:
                -camera_eye_pos (3f) randomized position
        """

        return self.randomize_param(self._random_obj_cfg['position'])

    @property
    def view_matrix(self):
        raise NotImplementedError
