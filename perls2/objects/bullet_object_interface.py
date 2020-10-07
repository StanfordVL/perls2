"""Class defining the interface to the objects in PyBullet
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions

import pybullet
import numpy as np

from perls2.objects.object_interface import ObjectInterface


class BulletObjectInterface(ObjectInterface):
    """Interface to objects in Pybullet.

    Attributes:
        physics_id (int): physicsClientId from Pybullet
        obj_id (int): bodyId from Bullet loadURDF
        name (str): identifer for the object, best to keep unique.
        position (ndarray 3f): xyz position in world space
        pose (ndarray 7f): x,y,z,qx,qy,qz,w pose of the object in world frame.
        linear_velocity (ndarray 3f): linear vel. of object in world frame.
            xdot, ydot, zdot
        angular_velocity (ndarray 3f): ang. vel. of object in world frame.
            wx, wy, wz.


    """

    def __init__(self,
                 physics_id,
                 obj_id,
                 name='object'):
        """
        Initialize variables

        Args:
            physics_id (int): physicsClientId from Pybullet
            obj_id (int): bodyId from Bullet loadURDF
            name (str): identifer for the object, best to keep unique.
        # TODO: support multiple copies of the same object

        Returns:
            None
        """
        super().__init__()
        self._physics_id = physics_id
        self._obj_id = obj_id
        self.name = name

    @property
    def position(self):
        """Get position of object in world frame.

        Returns numpy array
        """
        obj_position, _ = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return np.asarray(obj_position)

    def set_position(self, des_position):
        """ Set position of object in world frame.

        Args:
            des_position (list 3f): xyz position in world coordinates

        Returns:
            None

        Note: does not check for collisions
        """
        _, obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        pybullet.resetBasePositionAndOrientation(
            self._obj_id, des_position, obj_orn, self._physics_id)

    @property
    def orientation(self):
        """Orientation of object in world frame expressed as quaternion.
        """
        _, obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return obj_orn

    def set_orientation(self, des_ori):
        """ Set position of object in world frame.

        Args:
            des_ori (list): 4f orientation in world coordinates as quaternion.

        Returns:
            None
        Note: does not check for collisions
        """
        obj_pos, _ = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        pybullet.resetBasePositionAndOrientation(
            self._obj_id, obj_pos, des_ori, self._physics_id)

    @property
    def pose(self):
        """Pose of object in world frame. Expressed as list 7f position + quaternion.

        [x, y, z, qx, qy, qz, w]
        """
        obj_position, obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return list(obj_position + obj_orn)

    def set_pose(self, des_pose):
        """Set pose of object in world frame.

        Args:
            pose (list 7f): [x,y,z,qx,qy,qz,w] pose in world coordinates

        Returns:
            None

        Note: does not check for collisions
        """
        if len(des_pose) != 7:
            raise ValueError("Invalid dim for des_pose, should be list 7f")
        pybullet.resetBasePositionAndOrientation(
            self._obj_id, des_pose[:3], des_pose[3:], self._physics_id)

    @property
    def physics_id(self):
        return self._physics_id

    @physics_id.setter
    def physics_id(self, physics_id):
        """ set physics id for pybullet sim
        """
        self._physics_id = physics_id

    @property
    def obj_id(self):
        return self._obj_id

    @obj_id.setter
    def obj_id(self, new_id):
        """ setter for obj_id
        """
        self._obj_id = new_id

    def place(self, new_object_pos=None, new_object_orn=None):
        """ Places object in new position and orientation.

        Args:
            new_object_pos (list): 3f new positions for object in world frame.
            new_object_orn (list): 4f new object orientation as quaternion.

        Notes:
            If new_object_pos is None, maintains original position.
            If new_object_orn is None, maintains original orientation.
            If both are None, does nothing.
        """
        # Get the current orietation so it is maintained during reset

        self.obj_pos, self.obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)

        if new_object_pos is not None:
            self.obj_pos = new_object_pos
        if new_object_orn is not None:
            self.obj_orn = new_object_orn

        pybullet.resetBasePositionAndOrientation(
            self._obj_id, self.obj_pos, self.obj_orn, self._physics_id)

    def reset(self):
        """Reset object to last placed pose.
        """
        pybullet.resetBasePositionAndOrientation(
            self._obj_id, self.obj_pos, self.obj_orn, self._physics_id)

    @property
    def linear_velocity(self):
        """Get the lienar velocity of the body.

        Parameters
        ----------
        body_uid :
            The body Unique ID.

        Returns
        -------

            A 3-dimensional float32 numpy array.

        """
        linear_velocity, _ = pybullet.getBaseVelocity(
            bodyUniqueId=self._obj_id,
            physicsClientId=self._physics_id)
        return np.array(linear_velocity, dtype=np.float32)

    def set_linear_velocity(self, des_linear_vel):
        """ Set linear velocity of object. Breaks physics
        """
        pybullet.resetBaseVelocity(
            objectUniqueId=self.obj_id,
            linearVelocity=des_linear_vel,
            angularVelocity=self.angular_velocity,
            physicsClientId=self.physics_id)

    @property
    def angular_velocity(self):
        """Get the angular velocity of the body.

        Return:

            A 3-dimensional float32 numpy array.

        """
        _, angular_velocity = pybullet.getBaseVelocity(
            bodyUniqueId=self._obj_id,
            physicsClientId=self._physics_id)
        return np.array(angular_velocity, dtype=np.float32)

    def set_angular_velocity(self, des_angular_vel):
        """ Set linear velocity of object. Breaks physics

        Args:
            des_angular_vel (list 3f): Desired angular velocity as wx, wy, wz
        """
        pybullet.resetBaseVelocity(
            objectUniqueId=self.obj_id,
            linearVelocity=self.linear_velocity,
            angularVelocity=des_angular_vel,
            physicsClientId=self.physics_id)
