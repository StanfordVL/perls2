""" Parent class for Abstract Pybullet Environments
"""

import abc  # For abstract class definitions
import six  # For abstract class definitions
import pybullet
import numpy as np
import os
import time
import timeit
import logging

from perls2.worlds.world import World
from perls2.arenas.bullet_arena import BulletArena
from perls2.robots.bullet_robot_interface import BulletRobotInterface
from perls2.sensors.bullet_camera_interface import BulletCameraInterface
from perls2.objects.bullet_object_interface import BulletObjectInterface


class BulletWorld(World):
    """Class for PyBullet worlds.

    A BulletWorld will automatically create instances of BulletRobot,\
    BulletCamera and BulletObjectInterfaces based on the config file.

    Attributes:
        arena (BulletArena): Manages the sim by loading models, and for simulations,
            randomizing objects and sensors params.

        robot_interface (BulletRobotInterface): Communicates with robots and
            executes robot commands.

        camera_interface (BulletCameraInterface): Retrieves camera images and
            executes changes to params (e.g. intrinsics/extrinsics)

        object_interfaces (dict): A dictionary of perls2.BulletObjectInterfaces currently in the simulation.
            Keys are string unique identifying names for the object.

        physics_id (int): Unique id identifying physics client for Pybullet.
            Used to connect other interfaces when working with multiple
            simulations.

        time_step: float
            Float defining timestep to increment Pybullet simulation during
            pybullet.stepSimulation() calls. Set in config file

        MAX_STEPS: int
            Constant for maximum number of steps before terminating epsiode.
            Set in config file
        name: str
            A name to describe the world (e.g. training or testing)

    """

    def __init__(self,
                 config,
                 use_visualizer=False,
                 name=None):
        """Initialize.

        Args:
            config (dict): YAML config dict containing specifications.
            use_visualizer (bool): Whether or not to use visualizer.
                Note:: Pybullet only allows for one simulation to be
                connected to GUI. It is up to the user to manage this.
            name (str): Name of the world. (for multiple worlds)

        Returns:
            None

        Example:
            BulletWorld('cfg/my_config.yaml', True, 'MyBulletWorld')

        Notes:
            The world_factory is the recommended way for creating an
            appropriate instance of world according to the config parameters.

            Upon initialization the world automatically creates instances of
            BulletRobotInterface, BulletSensorInterface and
            BulletObjectInterface according to the config dictionary.

        """

        # Get configuration parameters
        self.config = config
        # Connect to appropriate pybullet channel based on use_visualizer flag
        self.use_visualizer = use_visualizer
        if self.use_visualizer:
            self._physics_id = pybullet.connect(pybullet.GUI)
        else:
            self._physics_id = pybullet.connect(pybullet.DIRECT)

        logging.info("New PhysicsID: " + str(self._physics_id))
        self._time_step = self.config['sim_params']['time_step']

        self.set_pb_physics()
        # check if config has camera.
        self.has_camera = False
        if 'sensor' in self.config:
            if 'camera' in self.config['sensor']:
                self.has_camera = True

        self.has_object = False
        # check if config has objects.
        if 'object' in self.config:
            self.has_object = True

        # Create an arena to load robot and objects
        self.arena = BulletArena(self.config, self._physics_id, has_camera=self.has_camera)

        self.robot_interface = BulletRobotInterface.create(
            config=self.config,
            physics_id=self._physics_id,
            arm_id=self.arena.arm_id,
            controlType=self.config['world']['controlType'])

        self.control_freq = self.config['control_freq']
        if self.has_camera:
            self.camera_interface = BulletCameraInterface(
                physics_id=self._physics_id,
                image_height=self.config['sensor']['camera']['image']['height'],
                image_width=self.config['sensor']['camera']['image']['width'],
                near=self.config['sensor']['camera']['intrinsics']['near_plane'],
                far=self.config['sensor']['camera']['intrinsics']['far_plane'],
                fov=self.config['sensor']['camera']['intrinsics']['fov'],
                cameraEyePosition=self.config['sensor']['camera']['extrinsics']['eye_position'],
                cameraTargetPosition=self.config['sensor']['camera']['extrinsics']['target_position'],
                cameraUpVector=self.config['sensor']['camera']['extrinsics']['up_vector'])

        if self.has_object:
            self._load_object_interfaces()
        self.name = name

        # self.ctrl_steps_per_action = int((self.config['control_freq'] / float(self.config['policy_freq'] * self.config['sim_params']['time_step'])))
        self.ctrl_steps_per_action = int((self.config['control_freq'] / float(self.config['policy_freq'])))
        self.is_sim = True

        self.step_counter = 0

        self.ee_list = []

    def _reinitialize(self):
        """ Reinitialize arenas, robot interfaces etc after reconnecting.
        """
        self.set_pb_physics()
        self.arena = BulletArena(self.config, self._physics_id)

        self.robot_interface = BulletRobotInterface.create(
            config=self.config,
            physics_id=self._physics_id,
            arm_id=self.arena.arm_id,
            controlType=self.config['controller']['selected_type'])
        self.camera_interface = BulletCameraInterface(
            physics_id=self._physics_id,
            image_height=self.config['sensor']['camera']['image']['height'],
            image_width=self.config['sensor']['camera']['image']['width'],
            cameraEyePosition=self.config['sensor']['camera']['extrinsics']['eye_position'],
            cameraTargetPosition=self.config['sensor']['camera']['extrinsics']['target_position'],
            cameraUpVector=self.config['sensor']['camera']['extrinsics']['up_vector'])
        self._load_object_interfaces()
        self.is_sim = True

    @property
    def physics_id(self):
        return self._physics_id

    def set_pb_physics(self):
        """ Set physics parameters for pybullet simulation.
        """
        pybullet.setGravity(0, 0, -9.8, physicsClientId=self._physics_id)
        pybullet.setTimeStep(self._time_step, physicsClientId=self._physics_id)
        pybullet.setPhysicsEngineParameter(deterministicOverlappingPairs=1, physicsClientId=self._physics_id)

    def _load_object_interfaces(self):
        """ Create a dictionary of object interfaces.

        Uses arena to create object interfaces for each object.
        """
        self.object_interfaces = {}

        # Create object interfaces for each of the objects found in the arena
        # dictionary
        for obj_idx, obj_name in enumerate(self.arena.object_dict):
            self.object_interfaces[obj_name] = BulletObjectInterface(
                physics_id=self._physics_id,
                obj_id=self.arena.object_dict[obj_name],
                name=obj_name)

    def add_object(self, path, name, pose, scale=1.0, is_static=False):
        """ Add object to world explicitly.

        Args:
            path (str): filepath name to object urdf
            name (str): name of object used for dictionary key
            pose (list): ((3,),(4,)) pose of the object as [[x, y, z], [qx, qy, qz, w]]
                orientation as quaternion.
            scale (double): scale of object.
            is_static (bool): whether object should remain fixed or not.

        Returns:
            object_interface (ObjectInterface): object interface added to world.

        Examples:
            objI = self.add_object('objects/ycb/013_apple/google_16k/textured.urdf', '013_apple', [0, 0, 0, 0, 0, 0, 1], 1.0,)
        """
        # Get the pybullet id from arena
        obj_id = self.arena._load_object_path(path=path,
                                              name=name,
                                              pose=pose,
                                              scale=scale,
                                              is_static=False)
        self.arena.object_dict[name] = obj_id

        # Create the BulletObject Interface
        object_interface = BulletObjectInterface(
            physics_id=self._physics_id,
            obj_id=obj_id,
            name=name)
        # Add to Objects dictionary
        self.object_interfaces[name] = object_interface

        return object_interface

    def remove_object(self, name):
        """ Remove object from world.
        Args:
            name (str): name of the object as stored in objects dictionary.

        Notes: Removes object from arena as well as objects dictionary.
        """
        try:
            objectI = self.object_interfaces.pop(name)
        except KeyError:
            raise KeyError('Invalid name -- object interface not found')
        self.arena._remove_object(objectI.obj_id, objectI.physics_id)

    def reset(self):
        """Reset the world.

        Returns:
            None

        TODO: Should this return something? an error? This needs to be
        defined by derived class. But all derived reset functions share
        these steps.
        """
        pass

    def reconnect(self):
        """Disconnects and reconnects to new physics engine
        """
        pybullet.disconnect(physicsClientId=self.physics_id)

        if self.use_visualizer:
            self._physics_id = pybullet.connect(pybullet.GUI)
        else:
            self._physics_id = pybullet.connect(pybullet.DIRECT)
        self.arena.physics_id = self._physics_id
        self.robot_interface.physics_id = self._physics_id
        self.camera_interface.physics_id = self._physics_id

    def reboot(self):
        """ Reboot pybullet simulation by clearing all objects, urdfs and
        resetting sim state.

        Warning: this is a slow process and should only be used for restoring state
        and deterministic applications.
        """
        pybullet.resetSimulation(physicsClientId=self._physics_id)
        self._reinitialize()

    def step(self, start=None):
        """Step the world(simulation) forward.

        Args:
            start (float): timestamp for when env.step was called by policy. Ignored for this class.
        Returns:
            None

        Step simulation forward a number of times per action
        to ensure smoothness when for collisions
        """

        # Prepare for next step by executing action
        #print("stepping world {}".format(self.ctrl_steps_per_action))
        for exec_steps in range(self.ctrl_steps_per_action):
            self.robot_interface.step()
            pybullet.stepSimulation(physicsClientId=self._physics_id)
        self.step_counter += 1

    def visualize(self, observation, action):
        """Visualize the action.

        Add visual markers to the world (in case of sim)
        or execute some movements (in case of real) to
        indicate the action about to be performed.

        Args:
            observation: The observation of the current step.
            action: The selected action.
        """
        pass

    def _draw_boundary_line(self, c1, c2):
        """Helper function to add debug line between two points.
        """
        pybullet.addUserDebugLine(c1, c2, [1, 0, 0], self.physics_id)

    def visualize_safenet_boundaries(self):
        """Add visualization to pb sim for safenet boundary.
        """
        if self.robot_interface.use_safenet:
            (upper, lower) = self.robot_interface.get_safenet_limits()
            if (upper is not None) and (lower is not None):
                corners = {}
                corners['000'] = lower
                corners['100'] = [upper[0], lower[1], lower[2]]
                corners['110'] = [upper[0], upper[1], lower[2]]
                corners['010'] = [lower[0], upper[1], lower[2]]
                corners['001'] = [lower[0], lower[1], upper[2]]
                corners['011'] = [lower[0], upper[1], upper[2]]
                corners['111'] = upper
                corners['101'] = [upper[0], lower[1], upper[2]]

                self._draw_boundary_line(corners['000'], corners['100'])
                self._draw_boundary_line(corners['100'], corners['110'])
                self._draw_boundary_line(corners['110'], corners['010'])
                self._draw_boundary_line(corners['010'], corners['000'])

                self._draw_boundary_line(corners['000'], corners['001'])
                self._draw_boundary_line(corners['001'], corners['011'])
                self._draw_boundary_line(corners['011'], corners['010'])

                self._draw_boundary_line(corners['011'], corners['111'])
                self._draw_boundary_line(corners['111'], corners['101'])
                self._draw_boundary_line(corners['101'], corners['001'])
                
                self._draw_boundary_line(corners['111'], corners['110'])
                self._draw_boundary_line(corners['101'], corners['100'])

    def wait_until_stable(self,
                          linear_velocity_threshold=0.005,
                          angular_velocity_threshold=0.005,
                          check_after_steps=100,
                          min_stable_steps=100,
                          max_steps=30000):
        """Wait until the objects are stable.

        Blocking code that checks if object linear and angular velocity are
        within respectives thresholds of 0.

        Args:
            linear_velocity_threshold (float)
            angular_velocity_threshold (float)
            check_after_steps (int): min number of steps to wait before
                checking object state
            min_stable_step (int): min number of steps for object to be stable
                before exiting
            max_steps (int): max number of steps to wait for stable object

        Returns:
            None

        :TODO:
            * add check for angular velocity threshold
        """
        assert self.is_sim, 'This function is only used in simulation.'

        # logger.debug('Waiting for objects to be stable...')

        num_steps = 0
        num_stable_steps = 0

        while(1):
            pybullet.stepSimulation(physicsClientId=self._physics_id)
            num_steps += 1
            all_stable = True

            if num_steps < check_after_steps:
                continue

            for obj_idx, obj_key in enumerate(self.object_interfaces):
                if (np.linalg.norm(self.object_interfaces[obj_key].linear_velocity) >=
                        linear_velocity_threshold):
                    all_stable = False

                if all_stable:
                    num_stable_steps +=1

            if ((num_stable_steps >= min_stable_steps) or (num_steps >= max_steps)):

                break

    def set_state(self, filepath):
        """ Set simulation to .bullet path found in filepath

            Args:
                filepath (str): filepath of .bullet file for saved state.

            Returns:
                None

            Examples:
                env.world.set_state('/path/to/state.bullet')

            Notes:
                To use this function correctly, the same objects / robots must be loaded
                in the same order. Therefore it is only recommended to use with the same
                world / config, rather than trying to load an empty world.
        """
        pybullet.restoreState(fileName=filepath, physicsClientId=self.physics_id)

