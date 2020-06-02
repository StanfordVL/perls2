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

    def get_position(self):
        """Get xyz position of object in world frame.

        Returns numpy array. 
        """
        obj_position, obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return np.asarray(obj_position)

    @property
    def position(self):
        """Get position of object in world frame.

        Returns numpy array
        """
        obj_position, _ = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return np.asarray(obj_position)

    @position.setter
    def position(self, des_position):
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
    def pose(self):
        """Set pose of object in world frame.
        Args: 
            des_position (list 7f): [x,y,z,qx,qy,qz,w] pose in world coordinates
        Returns: 
            None
        Note: does not check for collisions
        """        
        obj_position, obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return list(obj_position + obj_orn)

    @property
    def orientation(self):
        """Get the orientation of the object in world frame.
        """
        _, obj_orn = pybullet.getBasePositionAndOrientation(
            self._obj_id, self._physics_id)
        return np.asarray(obj_orn)        

    @pose.setter
    def pose(self, des_pose):
        """Set pose of object in world frame.
        Args: 
            pose (list 7f): [x,y,z,qx,qy,qz,w] pose in world coordinates
        Returns: 
            None
        Note: does not check for collisions
        """       
        pybullet.resetBasePositionAndOrientation(
            self._obj_id, des_pose[0], des_pose[1], self._physics_id)

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
        """Get the angular velocity of the body.

        Return:

            A 3-dimensional float32 numpy array.

        """
        _, angular_velocity = pybullet.getBaseVelocity(
                bodyUniqueId=self._obj_id, physicsClientId=self._physics_id)
        return np.array(angular_velocity, dtype=np.float32)

    @angular_velocity.setter
    def angular_velocity(self, des_angular_vel):
        """ Set linear velocity of object. Breaks physics

        Args: 
            des_angular_vel (list 3f): Desired angular velocity as wx, wy, wz
        """
        pybullet.resetBaseVelocity(objectUniqueId=self.obj_id,
            linearVelocity=self.linear_velocity,
            angularVelocity=des_angular_vel,
            physicsClientId=self.physics_id)

