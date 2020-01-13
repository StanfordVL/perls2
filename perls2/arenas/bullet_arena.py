"""The parent class for Arenas encapsulating robots, sensors and objects.
"""
import pybullet
import os
import numpy as np
from perls2.arenas.arena import Arena
import logging


class BulletArena(Arena):
    """The class definition for arenas
    Arenas contain interfaces for robots, sensors and
    """

    def __init__(self,
                 config,
                 physics_id):
        """ Initialization function.

        Parameters
        ----------
        config: dict
            A dict with config parameters

        """
        super().__init__(config)
        self.data_dir = os.path.abspath(self.config['data_dir'])
        print(self.data_dir)
        self.physics_id = physics_id

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
        logging.info("Bullet Arena Created")

        self.plane_id = self.load_ground()
        (self.arm_id, self.base_id) = self.load_robot()

        reset_angles = self.robot_cfg['neutral_joint_angles']
        for i in range(len(reset_angles)):
            # Force reset (breaks physics)
            pybullet.resetJointState(
                bodyUniqueId=self.arm_id,
                jointIndex=i,
                targetValue=reset_angles[i])

        for obj_key in self.config['scene_objects']:
            if obj_key in self.config:
                self.load_urdf(obj_key)
                for step in range(10):
                    pybullet.stepSimulation(self.physics_id)

        # Load the objects from the config file and
        # save their names and bullet body id (not object id
        # from config file)
        self.object_dict = {}
        for obj_idx, obj_key in enumerate(
                self.config['object']['object_dict']):
            pb_obj_id = self.load_object(obj_idx)
            obj_name = self.config['object']['object_dict'][obj_key]['name']
            logging.debug(obj_name + " loaded")

            # key value for object_dict is obj_name: pb_obj_id
            # example - '013_apple': 3
            # This makes it easier to reference.
            self.object_dict[obj_name] = pb_obj_id
            for step in range(50):
                #logging.debug("stepping for stability")
                pybullet.stepSimulation(self.physics_id)

        input("what's happening")




    def load_robot(self):
        """ Load the robot and return arm_id, base_id
        """
        arm_file = os.path.join(self.data_dir, self.robot_cfg['arm']['path'])
        base_file = os.path.join(self.data_dir, self.robot_cfg['base']['path'])

        arm_id = pybullet.loadURDF(
            fileName=arm_file,
            basePosition=self.robot_cfg['arm']['pose'],
            baseOrientation=pybullet.getQuaternionFromEuler(
                                    self.robot_cfg['arm']['orn']),
            globalScaling=1.0,
            useFixedBase=self.robot_cfg['arm']['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)
        logging.info("Loaded robot" + " arm_id :" + str(arm_id))

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


        # Reset to neutral position, while nothing is around.

        return (arm_id, base_id)

    def load_urdf(self, key):
        """General function to load urdf based on key"""
        path = os.path.join(self.data_dir, self.config[key]['path'])

        uid = pybullet.loadURDF(
            fileName=path,
            basePosition=self.config[key]['pose'][0],
            baseOrientation=pybullet.getQuaternionFromEuler(
                self.config[key]['pose'][1]),
            globalScaling=1.0,
            useFixedBase=self.config[key]['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)
        return uid


    def load_ground(self):
        """ Load ground and return ground_id
        """
        # import pybullet_data
        # pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

        plane_path = os.path.join(self.data_dir, self.config['ground']['path'])
        plane_id = pybullet.loadURDF(
            fileName=plane_path,
            basePosition=self.config['ground']['pose'][0],
            baseOrientation=pybullet.getQuaternionFromEuler(
                self.config['ground']['pose'][1]),
            globalScaling=1.0,
            useFixedBase=self.config['ground']['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)

        # Load object
        return plane_id

    def load_object(self, object_id=0):
        obj_key = 'object_' + str(object_id)
        object_dict = self.config['object']['object_dict'][obj_key]
        obj_path = os.path.join(self.data_dir, object_dict['path'])
        obj_id = pybullet.loadURDF(
                    obj_path,
                    basePosition=object_dict['default_position'],
                    baseOrientation=pybullet.getQuaternionFromEuler(
                            object_dict['pose'][1]),
                    globalScaling=1.0,
                    useFixedBase=object_dict['is_static'],
                    flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
                    physicsClientId=self.physics_id)

        return obj_id

    def view_matrix_to_extrinsic(self):
        L = (np.asarray(self.camera_target_pos) -
             np.asarray(self.camera_eye_pos))
        L = L/np.linalg.norm(L)
        s = np.cross(L, self.camera_up_vector)

        s = s/np.linalg.norm(s)
        u_prime = np.cross(s, L)
        R = np.array([[s[0], s[1], s[2]],
                      [u_prime[0], u_prime[1], u_prime[2]],
                      [L[0], L[1], L[2]]])

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
