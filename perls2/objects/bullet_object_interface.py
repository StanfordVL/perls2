"""
Abstract class defining the interface to the objects.
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions

import pybullet
import numpy as np

from perls2.objects.object_interface import ObjectInterface


class BulletObjectInterface(ObjectInterface):
    """Abstract interface to be implemented for each real and simulated
    robot.
    """

    def __init__(self,
                 physics_id,
                 obj_id,
                 name='object'):
        """
        Initialize variables

        Parameters
        ----------
            pose: list of seven floats (optional)
                Pose of robot base in world frame
                x y z qx qy qz qw
        """
        super().__init__()
        self._physics_id = physics_id
        self._obj_id = obj_id
        self.name = name

    def get_position(self):
        """Get xyz position of object in world frame.
        """

        is_connected, method = pybullet.getConnectionInfo(self._physics_id)

        obj_position, obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return np.asarray(obj_position)

    @property
    def position(self):
        is_connected, method = pybullet.getConnectionInfo(self._physics_id)

        obj_position, _ = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return np.asarray(obj_position)

    @position.setter
    def position(self, des_position):
        _, obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        pybullet.resetBasePositionAndOrientation(
            self._obj_id, des_position, obj_orn, self._physics_id)
    @property
    def pose(self):
        obj_position, obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return list(obj_position + obj_orn)

    @pose.setter
    def pose(self, pose):
        pybullet.resetBasePositionAndOrientation(
            self._obj_id, pose[0], pose[1], self._physics_id)

    @property
    def physics_id(self):
        return self._physics_id

    def set_obj_id(self, obj_id):
        """ Set object id for getting information about object
        """
        self._obj_id = obj_id

    @property
    def obj_id(self):
        return self._obj_id

    def set_physics_id(self, physics_id):
        """ set physics id for pybullet sim
        """
        self.physics_id = physics_id

    def place(self, new_object_pos):
        """ Given an upper and lower bound,
            set the location of the object to a new position

            NOTE: this should only be done on an env reset as it
            disregards the physics
        """
        # Get the current orietation so it is maintained during reset
        self.obj_pos, self.obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)

        self.obj_pos = new_object_pos
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
                bodyUniqueId=self._obj_id, physicsClientId=self._physics_id)
        return np.array(linear_velocity, dtype=np.float32)

    @linear_velocity.setter
    def linear_velocity(self, des_linear_vel):
        """ Set linear velocity of object. Breaks physics
        """
        pybullet.resetBaseVelocity(objectUniqueId=self.obj_id,
            linearVelocity=des_linear_vel,
            angularVelocity=self.angular_velocity,
            physicsClientId=self.physics_id)

    @property
    def angular_velocity(self):
        """Get the lienar velocity of the body.

        Parameters
        ----------
        body_uid :
            The body Unique ID.

        Returns
        -------

            A 3-dimensional float32 numpy array.

        """
        _, angular_velocity = pybullet.getBaseVelocity(
                bodyUniqueId=self._obj_id, physicsClientId=self._physics_id)
        return np.array(angular_velocity, dtype=np.float32)

    @angular_velocity.setter
    def angular_velocity(self, des_angular_vel):
        """ Set linear velocity of object. Breaks physics
        """
        pybullet.resetBaseVelocity(objectUniqueId=self.obj_id,
            linearVelocity=self.linear_velocity,
            angularVelocity=des_angular_vel,
            physicsClientId=self.physics_id)

