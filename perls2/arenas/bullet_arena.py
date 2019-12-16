"""The parent class for Arenas encapsulating robots, sensors and objects.
"""
import pybullet
import os
import numpy as np
from perls2.arenas.arena import Arena


class BulletArena(Arena):
    """The class definition for arenas
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
        print("Bullet Arena Created")

        self.plane_id = self.load_ground()

        obj_path = os.path.join(self.data_dir, self.config['object']['path'])
        self.obj_id = pybullet.loadURDF(
                    obj_path,
                    basePosition=self.config['object']['default_position'],
                    baseOrientation=pybullet.getQuaternionFromEuler(
                            self.config['object']['pose'][1]),
                    globalScaling=1.0,
                    useFixedBase=self.config['object']['is_static'],
                    flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
                    physicsClientId=self.physics_id)

        (self.arm_id, self.base_id) = self.load_robot()

        # self.table_id = pybullet.loadURDF(
        # fileName=self.config['table']['path'],
        # basePosition=self.config['table']['pose'][0],
        # baseOrientation=pybullet.getQuaternionFromEuler(
        #                         self.config['table']['pose'][1]),
        # globalScaling=1.0,
        # useFixedBase=True,
        # flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        # physicsClientId=self.physics_id )

        self.bin_id = pybullet.loadURDF(
            fileName=self.config['bin']['path'],
            basePosition=self.config['bin']['pose'][0],
            baseOrientation=pybullet.getQuaternionFromEuler(
                                    self.config['bin']['pose'][1]),
            globalScaling=1.0,
            useFixedBase=True,
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)

        # Set constraints
        # Constrain base to floor
        # self.cid = pybullet.setConstraint()

    def load_robot(self):
        """ Load the robot and return arm_id, base_id
        """
        print("Loading robot")
        arm_id = pybullet.loadURDF(
            fileName=self.config['robot']['arm']['path'],
            basePosition=self.config['robot']['arm']['pose'],
            baseOrientation=pybullet.getQuaternionFromEuler(
                                    self.config['robot']['arm']['orn']),
            globalScaling=1.0,
            useFixedBase=self.config['robot']['arm']['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)
        print("arm_id :" + str(arm_id))

        # Load Arm
        base_id = pybullet.loadURDF(
            fileName=self.config['robot']['base']['path'],
            basePosition=self.config['robot']['base']['pose'],
            baseOrientation=pybullet.getQuaternionFromEuler(
                                    self.config['robot']['base']['orn']),
            globalScaling=1.0,
            useFixedBase=self.config['robot']['base']['is_static'],
            flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=self.physics_id)

        return (arm_id, base_id)

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
            print(self.random_view_matrix)
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
        print(random_projection_matrix)
        return random_projection_matrix
