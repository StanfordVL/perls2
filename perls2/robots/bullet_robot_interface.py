"""Class defining the interface to the Pybullet simulation robots.
"""

import pybullet
import numpy as np
import abc

from perls2.robots.robot_interface import RobotInterface
import logging

class BulletRobotInterface(RobotInterface):
    """ Abstract interface to be implemented for each Pybullet simulated robot
    """
    def __init__(self,
                 physics_id,
                 arm_id,
                 config=None,
                 controlType=None):
        """
        Initialize variables

        Args: 
            -physics_id (int): Bullet physicsClientId
            -arm_id (int): bodyID for the robot arm from pybullet.loadURDF
            -config (dict): configuration file for the robot.
            -control_type: Controller type (currently not implemented.)
        """
        
        super().__init__(controlType)
        self._physics_id = physics_id
        self._arm_id = arm_id
        self._link_id_dict = self.get_link_dict()
        self.config = config
        robot_name = self.config['world']['robot']
        self.robot_cfg = self.config[robot_name]
        self._ee_index = 7  # Default
        self._num_joints = pybullet.getNumJoints(self._arm_id)

        # set the default values
        self._speed = self.robot_cfg['limb_max_velocity_ratio']
        self._position_threshold = (
            self.robot_cfg['limb_position_threshold'])
        self._velocity_threshold = (
            self.robot_cfg['limb_velocity_threshold'])
        self._default_force = 100
        self._default_position_gain = 0.1
        self._default_velocity_gain = 2.5

    def create(config, physics_id, arm_id):
        """Factory for creating robot interfaces based on type

        Creates a Bullet Sawyer or Panda Interface.

        Args:
            config (dict): config dictionary specifying robot type
            physics_id (int): unique id for identifying pybullet sim
            arm_id (int): unique id for identifying robot urdf from the arena

        Returns:
            None
        Notes:
            Only Rethink Sawyer Robots are currently supported
        """
        if (config['world']['robot'] == 'sawyer'):
            from perls2.robots.bullet_sawyer_interface import BulletSawyerInterface
            return BulletSawyerInterface(
                    physics_id=physics_id, arm_id=arm_id, config=config)
        if (config['world']['robot'] == 'panda'):
            from perls2.robots.bullet_panda_interface import BulletPandaInterface
            return BulletPandaInterface(
                physics_id=physics_id, arm_id=arm_id, config=config)
        else:
            raise ValueError(
                "invalid robot interface type. Specify in [world][robot]")

    def reset(self):
        """Reset the robot and move to rest pose.

        Args: None

        Returns: None

        :TODO:
            *This may need to do other things
        """
        self.set_joints_to_neutral_positions()

    @property
    def num_joints(self):
        """Returns number of joints from pybullet.getNumJoints
        Note: This likely not to be the same as the degrees of freedom. 
              PB can sometimes be difficult requiring the correct number of joints for IK
        """

        num_joints =  pybullet.getNumJoints(self._arm_id, physicsClientId=self._physics_id)
        return num_joints
    
    def set_joints_pos_vel(self, joint_pos, joint_vel=[0]*7):
        """Manualy reset joints to positions and velocities. 
            Args: 
                joint_pos (list 7f): list of desired joint positions
                joint_vel (list 7f): list of desired joint velocities
            Returns:
                None

        Note: breaks physics only to be used for IK, Mass Matrix and Jacobians 
        """
        if len(joint_pos) != 7:
            raise ValueError("joint_pos incorrect dimensions")
        if len(joint_pos) != len(joint_vel):
            raise ValueError("Joint positions and velocities should be same length")
        for i in range(len(joint_pos)):
            # Force reset (breaks physics)
            pybullet.resetJointState(
                bodyUniqueId=self._arm_id,
                jointIndex=i,
                targetValue=joint_pos[i],
                targetVelocity=joint_vel[i], 
                physicsClientId=self.physics_id)


    def inverse_kinematics(self, position, orientation):
        """Calculate inverse kinematics to get joint angles for a pose.

        Use pybullet's internal IK solver.
        To be used with a joint-space controller. 

        Args: 
            position (list 3f): [x, y, z] of desired ee position
            orientation (list 4f): [qx, qy, qz, w] for desired ee orientation as quaternion.

        returns:
            jointPoses (list 7f): joint positions that solve IK in radians.
        """        
 
        ikSolver = 0
        orientation = self.ee_orientation

        jointPoses = pybullet.calculateInverseKinematics(
            self._arm_id,
            self._ee_index,
            position,
            orientation,
            solver=ikSolver,
            maxNumIterations=100,
            residualThreshold=.01,
            physicsClientId=self._physics_id)

        jointPoses = list(jointPoses)
        return jointPoses    def set_joints_to_neutral_positions(self):
        """Set joints on robot to neutral positions as specified by the config file.

        Note: Breaks physics by forcibly setting the joint state. To be used only at
              reset of episode.
        """
        if self._arm_id is None:
            raise ValueError("no arm id")
        else:

            self._num_joints = 7 #

            joint_indices = [i for i in range(0, self._num_joints)]

            for i in range(self._num_joints):
                # Force reset (breaks physics)
                pybullet.resetJointState(
                    bodyUniqueId=self._arm_id,
                    jointIndex=i,
                    targetValue=self.limb_neutral_positions[i])

                # Set position control to maintain position
                pybullet.setJointMotorControl2(
                    bodyIndex=self._arm_id,
                    jointIndex=i,
                    controlMode=pybullet.POSITION_CONTROL,
                    targetPosition=self.limb_neutral_positions[i],
                    targetVelocity=0,
                    force=100,
                    positionGain=0.1,
                    velocityGain=1.2)
    @property
    def ee_index(self):
        return self._ee_index

    def get_link_name(self, link_uid):
        """Get the name of the link. (joint)

        Parameters
        ----------
        link_uid :
            A tuple of the body Unique ID and the link index.

        Returns
        -------

            The name of the link.

        """
        self.arm_id, link_ind = link_uid
        _, _, _, _, _, _, _, _, _, _, _, _, link_name, _, _, _, _ = (
                pybullet.getJointInfo(bodyUniqueId=self._arm_id,
                                      jointIndex=link_ind,
                                      physicsClientId=self._physics_id))
        return link_name

    def get_link_id_from_name(self, link_name):
        """Get link id from name

        Args:
            link_name (str): name of link in urdf file
        Returns:
            link_id (int): index of the link in urdf file.
             OR -1 if not found.
        """
        if link_name in self._link_id_dict:
            return self._link_id_dict.get(link_name)
        else:
            return -1

    def get_link_dict(self):
        """Create a dictionary between link id and link name
        Dictionary keys are link_name : link_id

        Notes: Each link is connnected by a joint to its parent, so
               num links = num joints
        """

        num_links = pybullet.getNumJoints(
            self._arm_id, physicsClientId=self._physics_id)

        link_id_dict = {}

        for link_id in range(num_links):
            link_id_dict.update(
                {
                    self.get_link_name(
                        (self._arm_id, link_id)).decode('utf-8'): link_id
                })

        return link_id_dict

    def get_joint_limit(self, joint_ind):
        """Get the limit of the joint.

        These limits are specified by the URDF.

        Parameters
        ----------
        joint_uid :
            A tuple of the body Unique ID and the joint index.

        Returns
        -------
        limit
            A dictionary of lower, upper, effort and velocity.

        """
        (_, _, _, _, _, _, _, _, lower, upper, max_force, max_vel, _,
         _, _, _, _) = pybullet.getJointInfo(
            bodyUniqueId=self._arm_id,
            jointIndex=joint_ind,
            physicsClientId=self._physics_id)

        limit = {
                'lower': lower,
                'upper': upper,
                'effort': max_force,
                'velocity': max_vel
                }

        return limit

    def set_gripper_to_value(self, value):
        """Close the gripper of the robot
        """
        # Get the joint limits for the right and left joint from config file
        l_finger_joint_limits = self.get_joint_limit(
            self.get_link_id_from_name(self.robot_cfg['l_finger_name']))

        r_finger_joint_limits = self.get_joint_limit(
            self.get_link_id_from_name(self.robot_cfg['r_finger_name']))

        # Get the range based on these joint limits
        l_finger_joint_range = (
            l_finger_joint_limits.get('upper') -
            l_finger_joint_limits.get('lower'))

        r_finger_joint_range = (
            r_finger_joint_limits.get('upper') -
            r_finger_joint_limits.get('lower'))

        # Determine the joint position by clipping to upper limit.
        l_finger_position = (
            l_finger_joint_limits.get('upper') -
            value * l_finger_joint_range)

        r_finger_position = (
            r_finger_joint_limits.get('upper') -
            value * r_finger_joint_range)

        # Set the joint angles all at once
        l_finger_index = self.get_link_id_from_name(
            self.robot_cfg['l_finger_name'])
        r_finger_index = self.get_link_id_from_name(
            self.robot_cfg['r_finger_name'])

        gripper_q = self.q
        gripper_q[l_finger_index] = l_finger_position
        gripper_q[r_finger_index] = r_finger_position
        
        gripper_des_q = [gripper_q[l_finger_index], 
                          gripper_q[r_finger_index]]
        gripper_indices = [l_finger_index, r_finger_index]

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self._arm_id,
            jointIndices=gripper_indices,
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=gripper_des_q)

    def open_gripper(self):
        """Open the gripper of the robot
        """
        self.set_gripper_to_value(0.01)

    def close_gripper(self):
        """Close the gripper of the robot
        """
        self.set_gripper_to_value(0.99)

    # Properties

    @property
    def physics_id(self):
        return self._physics_id

    @physics_id.setter
    def physics_id(self, physics_id):
        self._physics_id = physics_id

    @property
    def arm_id(self):
        return self._arm_id

    @arm_id.setter
    def arm_id(self, arm_id):
        self._arm_id = arm_id

    @property
    def name(self):
        return self._name

    @property
    @abc.abstractmethod
    def version(self):
        """dict of current versions of robot SDK, gripper, and robot
        """
        raise NotImplementedError

    @property
    def ee_position(self):
        """list of three floats [x, y, z] of the position of the
        end-effector.

        Updates every call. Does not store property.
        """
        ee_position, _, _, _, _, _, = pybullet.getLinkState(
            self._arm_id,
            self._ee_index,
            physicsClientId=self._physics_id)
        return list(ee_position)

    @ee_position.setter
    def ee_position(self, position):
        """Set the end effector position in xyz coordinates

        Uses IK to determine desired joint configuration to acheive ee
        position.Uses position control with defaulat parameters to move to
        calculated joint configuration

        Args:
            position (list): desired position vector (3f) for end effector

        """
        ikSolver = 0
        orientation = self.ee_orientation

        jointPoses = pybullet.calculateInverseKinematics(
            self._arm_id,
            self._ee_index,
            position,
            orientation,
            solver=ikSolver,
            maxNumIterations=100,
            residualThreshold=.01,
            physicsClientId=self._physics_id)

        jointPoses = list(jointPoses)
        # For dealing with extra joints in gripper and hand.
        # TODO: should this be changed?
        jointPoses.extend([0, 0, 0, 0, 0])

        for i in range(len(jointPoses)):
            pybullet.setJointMotorControl2(
                    bodyIndex=self._arm_id,
                    jointIndex=i,
                    controlMode=pybullet.POSITION_CONTROL,
                    targetPosition=jointPoses[i],
                    targetVelocity=0,
                    force=self._default_force,
                    positionGain=self._default_position_gain,
                    velocityGain=self._default_velocity_gain,
                    physicsClientId=self._physics_id)

            position, _, _, _ = pybullet.getJointState(
                bodyUniqueId=self._arm_id, jointIndex=i,
                physicsClientId=self._physics_id)

    @property
    def ee_orientation(self):
        _, ee_orientation, _, _, _, _, = pybullet.getLinkState(
            self._arm_id, self._ee_index, physicsClientId=self._physics_id)
        return list(ee_orientation)

    @property
    def ee_pose(self):
        return self.ee_position + self.ee_orientation

    @ee_pose.setter
    def ee_pose(self, des_ee_pose):
        """Set the end effector position in xyz coordinates
        Args: 
            -des_ee_pose (list 7f): x, y, z, qx, qy, qz
        TODO: make this use the controller
        """
        position = des_ee_pose[0:3]

        ikSolver = 0
        orientation = des_ee_pose[3:]

        jointPoses = pybullet.calculateInverseKinematics(
            self._arm_id,
            self._ee_index,
            position,
            orientation,
            solver=ikSolver,
            maxNumIterations=100,
            residualThreshold=.01,
            physicsClientId=self._physics_id)

        jointPoses = list(jointPoses)

        for i in range(len(jointPoses)):
            pybullet.setJointMotorControl2(
                bodyIndex=self._arm_id,
                jointIndex=i,
                controlMode=pybullet.POSITION_CONTROL,
                targetPosition=jointPoses[i],
                targetVelocity=0,
                force=self._default_force,
                positionGain=self._default_position_gain,
                velocityGain=self._default_velocity_gain,
                physicsClientId=self._physics_id)

    def goto_ee_pose(self, des_ee_pose):
        """Use position control to acheive pose. 
        Args: 
            -des_ee_pose (list 7f): x, y, z, qx, qy, qz

        TODO: make this do some form of motion planning.
        """
        self.ee_pose = des_ee_pose

    def set_ee_pose_position_control(
            self,
            target_position,
            target_velocity=None,
            max_velocity=None,
            max_force=None,
            position_gain=None,
            velocity_gain=None):
        """Position control of a joint.

        Args:
            target_position (3f): The target ee position in xyz
            target_velocity (xf):
                The target joint velocity. (Default value = None)
            max_velocity :
                The maximal joint velocity. (Default value = None)
            max_force :
                The maximal joint force. (Default value = None)
            position_gain :
                The position gain. (Default value = None)
            velocity_gain :
                The velocity gain. (Default value = None)

        Returns
        -------

        """
        target_joint_position = pybullet.calculateInverseKinematics(
            self._arm_id,
            self._ee_index,
            target_position,
            orientation,
            solver=ikSolver,
            maxNumIterations=100,
            residualThreshold=.01,
            physicsClientId=self._physics_id)

        kwargs = dict()
        kwargs['physicsClientId'] = self._physics_id
        kwargs['controlMode'] = pybullet.POSITION_CONTROL
        kwargs['targetPosition'] = target_joint_position

        if target_velocity is not None:
            kwargs['targetVelocities'] = target_velocity

        if max_velocity is not None:
            kwargs['maxVelocities'] = max_velocity

        if max_force is not None:
            kwargs['forces'] = max_force

        if position_gain is not None:
            kwargs['positionGains'] = position_gain

        if velocity_gain is not None:
            kwargs['velocityGains'] = velocity_gain

        pybullet.setJointMotorControlArray(**kwargs)

    def set_ee_velocity(self, des_ee_velocity):
        """ Set the ee velocity using velocity control

        Calculates linear Jacobian and uses it to determine
        desired joint velocities

        """
        raise NotImplementedError

    def set_joint_velocity_control(
            self,
            joint_index,
            target_velocity,
            max_force=None,
            position_gain=None,
            velocity_gain=None):
        """Velocity control of a joint.

        Parameters
        ----------
        joint_uid :
            The tuple of the body unique ID and the joint index.
        target_velocity :
            The joint velocity.
        max_joint_force :
            The maximal force of the joint.
        position_gain :
            The position gain. (Default value = None)
        velocity_gain :
            The velocity gain. (Default value = None)
        max_force :
             (Default value = None)

        Returns
        -------

        """
        kwargs = dict()
        kwargs['physicsClientId'] = self._physics_id
        kwargs['bodyUniqueId'] = self._arm_id
        kwargs['jointIndex'] = joint_index
        kwargs['controlMode'] = pybullet.VELOCITY_CONTROL
        kwargs['targetVelocity'] = target_velocity

        if max_force is not None:
            kwargs['force'] = max_force

        if position_gain is not None:
            kwargs['positionGain'] = position_gain

        if velocity_gain is not None:
            kwargs['velocityGain'] = velocity_gain

        pybullet.setJointMotorControlArray(**kwargs)

    @property
    def q(self):
        """
        Get the joint configuration of the robot arm.
        Args:
            None
        Returns:
            list of joint positions in radians ordered by
            from small to large.
        """
        q = []
        for joint_index in range(self.num_joints):
            q_i, _, _, _, = pybullet.getJointState(
                self._arm_id,
                joint_index,
                physicsClientId=self._physics_id)
            q.append(q_i)

        return q

    @q.setter
    def q(self, qd):
        """
        Get the joint configuration of the robot arm.
        Args:
            qd: list
                list of desired joint position
        """
        # for joint_index in range(self._num_joints)
        #     q[joint_index], _, _, _, = pybullet.getLinkState(
        #         self._arm_id,
        #         joint_index,
        #         physicsClientId=self._physics_id)

        pybullet.setJointMotorControlArray(
            bodyUniqueId=self._arm_id,
            jointIndices=range(0, 7),
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=qd)
        return q

    def set_joint_position_control(
            self,
            joint_ind,
            target_position,
            target_velocity=0,
            force=50,
            max_velocity=1.0,
            max_force=None,
            position_gain=0.5,
            velocity_gain=5.0):
        """Position control of a single joint.

        Parameters
        ----------
        joint_ind (int):
            The tuple of the body unique ID and the joint index.
        target_position :
            The target joint position.
        target_velocity :
            The target joint velocity. (Default value = None)
        max_velocity :
            The maximal joint velocity. (Default value = None)
        max_force :
            The maximal joint force. (Default value = None)
        position_gain :
            The position gain. (Default value = None)
        velocity_gain :
            The velocity gain. (Default value = None)

        Returns
        -------

        """

        kwargs = dict()
        kwargs['physicsClientId'] = self._physics_id
        kwargs['bodyUniqueId'] = self._arm_id
        kwargs['jointIndex'] = joint_ind
        kwargs['controlMode'] = pybullet.POSITION_CONTROL
        kwargs['targetPosition'] = target_position

        if target_velocity is not None:
            kwargs['targetVelocity'] = target_velocity

        if max_velocity is not None:
            kwargs['maxVelocity'] = max_velocity

        if max_force is not None:
            kwargs['force'] = max_force

        if position_gain is not None:
            kwargs['positionGain'] = position_gain

        if velocity_gain is not None:
            kwargs['velocityGain'] = velocity_gain

        pybullet.setJointMotorControl2(**kwargs)

    def set_joints_position_control(
            self,
            joint_inds,
            target_positions,
            target_velocities=None,
            max_velocities=None,
            max_forces=None,
            position_gains=None,
            velocity_gains=None):
        """Position control of a list of joints of a body.

        Parameters
        ----------
        self.arm_id :
            The body unique ID.
        joint_inds :
            The list of joint indices.
        target_positions :
            The list of target joint positions.
        target_velocities :
            The list of of target joint velocities. (Default value = None)
        max_velocities :
            The list of maximal joint velocities. (Default value = None)
        max_forces :
            The list of maximal joint forces. (Default value = None)
        position_gains :
            The list of position gains. (Default value = None)
        velocity_gains :
            The list of velocity gains. (Default value = None)

        Returns
        -------

        """
        kwargs = dict()
        kwargs['physicsClientId'] = self.physics_id
        kwargs['bodyUniqueId'] = self.arm_id
        kwargs['jointIndices'] = joint_inds
        kwargs['controlMode'] = pybullet.POSITION_CONTROL
        kwargs['targetPositions'] = target_positions

        if target_velocities is not None:
            kwargs['targetVelocities'] = target_velocities

        if max_velocities is not None:
            raise NotImplementedError(
                'This is not implemented in pybullet')

        if max_forces is not None:
            kwargs['forces'] = max_forces

        if position_gains is not None:
            if isinstance(position_gains, (float, int)):
                kwargs['positionGains'] = [position_gains] * len(joint_inds)
            else:
                kwargs['positionGains'] = position_gains

        if velocity_gains is not None:
            if isinstance(velocity_gains, (float, int)):
                kwargs['velocityGains'] = [velocity_gains] * len(joint_inds)
            else:
                kwargs['velocityGains'] = velocity_gains

        pybullet.setJointMotorControlArray(**kwargs)

    def set_joint_velocity_control(
            self,
            joint_ind,
            target_velocity,
            max_force=None,
            position_gain=None,
            velocity_gain=None):
        """Velocity control of a joint.

        Parameters
        ----------
        joint_uid :
            The tuple of the body unique ID and the joint index.
        target_velocity :
            The joint velocity.
        max_joint_force :
            The maximal force of the joint.
        position_gain :
            The position gain. (Default value = None)
        velocity_gain :
            The velocity gain. (Default value = None)
        max_force :
             (Default value = None)

        Returns
        -------

        """

        kwargs = dict()
        kwargs['physicsClientId'] = self.physics_id
        kwargs['bodyUniqueId'] = self.arm_id
        kwargs['jointIndex'] = joint_ind
        kwargs['controlMode'] = pybullet.VELOCITY_CONTROL
        kwargs['targetVelocity'] = target_velocity

        if max_force is not None:
            kwargs['force'] = max_force

        if position_gain is not None:
            kwargs['positionGain'] = position_gain

        if velocity_gain is not None:
            kwargs['velocityGain'] = velocity_gain

        pybullet.setJointMotorControl2(**kwargs)

    @property
    def dq(self):
        """
        Get the joint velocities of the robot arm.
        :return: a list of joint velocities in radian/s ordered by
        indices from small to large.
        Typically the order goes from base to end effector.
        """
        dq = []
        for joint_index in range(self.num_joints):
            _, dq_i, _, _, = pybullet.getJointState(
                self._arm_id, joint_index,
                physicsClientId=self._physics_id)
            dq.append(dq_i)
        return dq

    @dq.setter
    def dq(self, dq_d):
        """
        Set the  desired joint velocities of the robot arm.
        :return: a list of joint velocities in radian/s ordered by
        indices from small to large.
        Typically the order goes from base to end effector.
        """
        pybullet.setJointMotorControlArray(
            bodyUniqueId=self._arm_id,
            jointIndices=range(0, 7),
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocities=dq_d)
