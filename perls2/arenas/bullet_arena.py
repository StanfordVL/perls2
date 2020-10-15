"""The class for Arenas in Pybullet.
"""
import pybullet
import os
import sys
import numpy as np
from perls2.arenas.arena import Arena
import perls2
import logging
logging.basicConfig(level=logging.INFO)


class BulletArena(Arena):
    """The class definition for Areanas using pybullet.
    Arenas
    """

    def __init__(self,
                 config,
                 physics_id,
                 has_camera):
        """ Initialization function.

            Gets parameters from configuration file for managing experimental
            setup and episodes. These include randomized parameters for
            camera extrinsic/intrinsics, object placement.
        Args:
            config (dict): A dict with config parameters
            physics_id (int): unique id for pybullet physics simulation.
            has_camera (bool): if config has camera parameter.

        Returns:
            None
        """
        super().__init__(config, has_camera)
        # If a specific directory for data dir is not defined, use perls2's.
        perls2_path = os.path.dirname(perls2.__path__[0])
        self.perls2_data_dir = os.path.join(perls2_path, 'data')
        if 'data_dir' not in self.config:
            data_dir = self.perls2_data_dir
        else:
            data_dir = self.config['data_dir']

        self.data_dir = os.path.abspath(data_dir)
        self.physics_id = physics_id
        self._bodies_pbid_dict = {}
        self._objects_dict = {}
        self.robot_type = self.config['world']['robot']
        self.has_camera = has_camera
        if self.has_camera:
            # initialize view matrix
            self._view_matrix = pybullet.computeViewMatrix(
                cameraEyePosition=self.camera_eye_pos,
                cameraTargetPosition=self.camera_target_pos,
                cameraUpVector=self.camera_up_vector)

            self._random_view_matrix = self._view_matrix

            # Initialize projection matrix
            self._projection_matrix = pybullet.computeProjectionMatrixFOV(
                fov=self.fov,
                aspect=float(self.image_width) / float(self.image_height),
                nearVal=self.near_plane,
                farVal=self.far_plane)

            self._random_projection_matrix = self._projection_matrix
            self._randomize_on = (
                self.config['sensor']['camera']['random']['randomize'])

        # Load URDFs to set up simulation environment.
        logging.debug("Bullet Arena Created")
        if 'ground' in self.config:
            self.plane_id = self.load_urdf('ground', self.perls2_data_dir)#self.load_ground()
        logging.debug("ground loaded")
        (self.arm_id, self.base_id) = self.load_robot()
        logging.debug("Robot loaded")
        self.reset_robot_to_neutral()
        if 'scene_objects' in self.config:
            self.load_scene_objects()
        if 'object' in self.config:
            self.load_objects_from_config()

    def reload(self):
        """ Reload all scene objects, objects and robots.

        Return: tuple of pybullet ids for assigning to robots.
        """
        self.plane_id = self.load_ground()

        (self.arm_id, self.base_id) = self.load_robot()
        self.reset_robot_to_neutral()

        self.load_scene_objects()
        self.load_objects_from_config()
        return (self.arm_id, self.base_id)

    def reset_robot_to_neutral(self):
        """ Reset robot to neutral joint angles.

        This step is important to keep the robot from messing up
        simulation set up.
        """
        reset_angles = self.robot_cfg['neutral_joint_angles']
        for i, angle in enumerate(reset_angles):
            # Force reset (breaks physics)

            pybullet.resetJointState(
                bodyUniqueId=self.arm_id,
                jointIndex=i,
                targetValue=angle,
                physicsClientId=self.physics_id)

    def load_scene_objects(self):
        """ Load scene objects from config file.
        """
        self.scene_objects_dict = {}

        # Load scene objects (e.g. table, bins)
        for obj_key in self.config['scene_objects']:
            if obj_key in self.config:
                self.scene_objects_dict[obj_key] = self.load_urdf(obj_key, self.perls2_data_dir)
                logging.debug(obj_key + " loaded")

                for step in range(10):
                    pybullet.stepSimulation(self.physics_id)

    def load_objects_from_config(self):
        """ Load objects from config file
        """
        self.object_dict = {}
        if (isinstance(self.config['object'], dict)):
            # Load the objects from the config file and
            # save their names and pybullet body id
            # (not object id from config file)
            if ('object_dict' in self.config['object'].keys()):
                for obj_idx, obj_key in enumerate(
                        self.config['object']['object_dict']):
                    pb_obj_id = self.load_object(obj_idx)
                    obj_name = \
                        self.config['object']['object_dict'][obj_key]['name']
                    logging.debug(obj_name + " loaded")

                    # key value for object_dict is obj_name: pb_obj_id
                    # example - '013_apple': 3
                    # This makes it easier to reference.
                    self.object_dict[obj_name] = pb_obj_id
                    for step in range(50):
                        pybullet.stepSimulation(self.physics_id)
        else:
            print("No objects from config")

    def load_robot(self):
        """ Load the robot and return arm_id, base_id
        """
        arm_file = os.path.join(self.perls2_data_dir, self.robot_cfg['arm']['path'])

        arm_id = pybullet.loadURDF(
            fileName=arm_file,
            basePosition=self.robot_cfg['arm']['pose'],
            baseOrientation=pybullet.getQuaternionFromEuler(
                self.robot_cfg['arm']['orn']),
            globalScaling=1.0,
            useFixedBase=self.robot_cfg['arm']['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | pybullet.URDF_USE_INERTIA_FROM_FILE,
            physicsClientId=self.physics_id)
        logging.info("Loaded robot" + " arm_id :" + str(arm_id))
        # Load Arm
        if (self.robot_cfg['base'] != 'None'):
            base_file = os.path.join(self.perls2_data_dir, self.robot_cfg['base']['path'])
            base_id = pybullet.loadURDF(
                fileName=base_file,
                basePosition=self.robot_cfg['base']['pose'],
                baseOrientation=pybullet.getQuaternionFromEuler(
                    self.robot_cfg['base']['orn']),
                globalScaling=1.0,
                useFixedBase=self.robot_cfg['base']['is_static'],
                flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | pybullet.URDF_USE_INERTIA_FROM_FILE,
                physicsClientId=self.physics_id)
        else:
            base_id = -1

        return (arm_id, base_id)

    def load_urdf(self, key, data_dir=None):
        """General function to load urdf based on key

        Args:
            key (string): key for the urdf to be loaded.

        Returns:
            uid (int): pybullet Body ID for that body.

        Note: Keys must be specified at top level of config.
              Function does not traverse directory.
        """
        if data_dir is None:
            data_dir = self.data_dir

        path = os.path.join(data_dir, self.config[key]['path'])

        uid = pybullet.loadURDF(
            fileName=path,
            basePosition=self.config[key]['pose'][0],
            baseOrientation=pybullet.getQuaternionFromEuler(
                self.config[key]['pose'][1]),
            globalScaling=1.0,
            useFixedBase=self.config[key]['is_static'],
            flags=(pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | pybullet.URDF_USE_INERTIA_FROM_FILE),
            physicsClientId=self.physics_id)
        return uid

    def load_ground(self):
        """ Load ground and return ground_id
        """
        plane_path = os.path.join(self.data_dir, self.config['ground']['path'])
        logging.debug("plane path " + str(plane_path))
        plane_id = pybullet.loadURDF(
            fileName=plane_path,
            basePosition=self.config['ground']['pose'][0],
            baseOrientation=pybullet.getQuaternionFromEuler(
                self.config['ground']['pose'][1]),
            globalScaling=1.0,
            useFixedBase=self.config['ground']['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)

        return plane_id

    def load_object(self, object_id=0):
        obj_key = 'object_' + str(object_id)
        object_dict = self.config['object']['object_dict'][obj_key]
        obj_path = os.path.join(self.data_dir, object_dict['path'])
        obj_id = pybullet.loadURDF(
            obj_path,
            basePosition=object_dict['default_position'],
            baseOrientation=pybullet.getQuaternionFromEuler(
                object_dict['orientation']),
            globalScaling=object_dict['scale'],
            useFixedBase=object_dict['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)
        logging.debug(" Loaded object " + str(obj_key) + " with id: " + str(obj_id))

        return obj_id

    def _load_object_path(self, path, name, pose, scale, is_static):
        """Wrapper for bullet Load URDF function.

        Args:
            path (str): relative filepath to object urdf
            name (str): unique identifying string to name object.
            pose (list): 6f or 7f pose. [Position, orientation]
                Orientation may be euler or quaternion.
            scale (float): Scale to resize object globally
            is_static (bool): if object remains fixed during sim.

        Returns
            obj_id (int): unique int identifying object in pybullet sim.
        """
        obj_path = os.path.join(self.data_dir, path)
        position = pose[:3]
        orientation = pose[3:]
        # Convert to quaternion if euler angle.
        if len(orientation) == 3:
            orientation = pybullet.getQuaternionFromEuler(orientation)

        obj_id = pybullet.loadURDF(
            obj_path,
            basePosition=position,
            baseOrientation=orientation,
            globalScaling=scale,
            useFixedBase=is_static,
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)

        return obj_id

    def _remove_object(self, object_id=0, phys_id=None):
        """ Remove object from simulation.
        Internal use only. Use world.remove_object instead.
        Args:
            object_id (int): pybullet id from load urdf
        """

        logging.debug("Num bodies" + str(pybullet.getNumBodies(physicsClientId=self.physics_id)))
        logging.debug(str(pybullet.getBodyInfo(object_id, physicsClientId=self.physics_id)))

        logging.debug(self.object_dict)

        if phys_id is None:
            pybullet.removeBody(
                bodyUniqueId=object_id,
                physicsClientId=self.physics_id)
        else:
            pybullet.removeBody(
                bodyUniqueId=object_id,
                physicsClientId=phys_id)

    def view_matrix_to_extrinsic(self):
        L = (np.asarray(self.camera_target_pos) -
             np.asarray(self.camera_eye_pos))
        L = L / np.linalg.norm(L)
        s = np.cross(L, self.camera_up_vector)

        s = s / np.linalg.norm(s)
        u_prime = np.cross(s, L)
        R = np.array([[s[0], s[1], s[2]],
                      [u_prime[0], u_prime[1], u_prime[2]],
                      [L[0], L[1], L[2]]])
        return R

    @property
    def view_matrix(self):
        if (self._randomize_on):
            return self.random_view_matrix()
        else:
            return self._view_matrix

    def random_view_matrix(self):

        random_view_matrix = pybullet.computeViewMatrix(
            cameraEyePosition=self.randomize_param(
                self._rand_camera_extrin_cfg['eye_position']),
            cameraTargetPosition=self.randomize_param(
                self._rand_camera_extrin_cfg['target_position']),
            cameraUpVector=self.camera_up_vector)

        return random_view_matrix

    @property
    def projection_matrix(self):
        if (self._randomize_on):
            return self.random_projection_matrix()
        else:
            return self._projection_matrix

    def random_projection_matrix(self):
        rand_fov = self.randomize_param(self._rand_camera_intrin_cfg['fov'])
        rand_near_plane = self.randomize_param(
            self._rand_camera_intrin_cfg['near_plane'])
        rand_far_plane = self.randomize_param(
            self._rand_camera_intrin_cfg['far_plane'])

        random_projection_matrix = pybullet.computeProjectionMatrixFOV(
            fov=rand_fov,
            aspect=float(self.image_width) / float(self.image_height),
            nearVal=rand_near_plane,
            farVal=rand_far_plane)

        return random_projection_matrix

