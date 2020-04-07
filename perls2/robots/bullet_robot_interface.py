"""Class defining the interface to the Pybullet simulation robots.
"""

import pybullet
import numpy as np
import abc

from perls2.robots.robot_interface import RobotInterface
from tq_control.controllers.ee_imp import EEImpController
from tq_control.robot_model.manual_model import ManualModel
import logging
logging.basicConfig(level=logging.DEBUG)
from perls2.robots.controller import OperationalSpaceController
import scipy


def nested_tuple_to_list(tuple_input):
    for elem in tuple_input:
        if (isinstance(tuple_input, tuple)):
            return list(map(nested_tuple_to_list(elem)))
        else:
            return elem

class BulletRobotInterface(RobotInterface):
    """ Abstract interface to be implemented for each Pybullet simulated robot
    """
    def __init__(self,
                 physics_id,
                 arm_id,
                 config=None,
                 controlType='EEImp'):
        """
        Initialize variables

        Args: 
            -physics_id (int): Bullet physicsClientId
            -arm_id (int): bodyID for the robot arm from pybullet.loadURDF
            -config (dict): configuration file for the robot.
            -control_type: Controller type (currently not implemented.)
        """       
        # super().__init__(controlType)
        self._physics_id = physics_id
        self._arm_id = arm_id
        self._link_id_dict = self.get_link_dict()
        self.config = config
        robot_name = self.config['world']['robot']
        self.robot_cfg = self.config[robot_name]
        self._ee_index = 7  # Default
        self._num_joints = pybullet.getNumJoints(self._arm_id)
        self._motor_joint_indices = self.get_motor_joint_indices()

        # set the default values
        self._speed = self.robot_cfg['limb_max_velocity_ratio']
        self._position_threshold = (
            self.robot_cfg['limb_position_threshold'])
        self._velocity_threshold = (
            self.robot_cfg['limb_velocity_threshold'])

        self._default_force = 100
        self._default_position_gain = 0.1
        self._default_velocity_gain = 2.5

        # URDF properties
        self.joint_limits = self.get_joint_limits()
        self._joint_max_velocities = self.get_joint_max_velocities()
        self._joint_max_forces = self.get_joint_max_forces()
        self._dof = self.get_dof()
        
        super().__init__(controlType)
        self.update()
        if self.controlType == 'EEImp':
            self.controller = EEImpController(self.model,
                kp=200, damping=0.5,
                interpolator_pos =None,
                interpolator_ori=None,
                control_freq=self.config['sim_params']['steps_per_action'])

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

    def set_joints_to_neutral_positions(self):
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

    def set_joints_to_neutral_positions(self):
        """Set joints on robot to neutral
        """
        if self._arm_id is None:
            print("arm id not set")
        else:
            self._num_joints = pybullet.getNumJoints(self._arm_id)

            joint_indices = [i for i in range(0,7)]
            # pybullet.setJointMotorControlArray(bodyUniqueId=self._arm_id,
            #                                jointIndices= joint_indices,
            #                                controlMode=pybullet.PD_CONTROL,
            #                                targetPositions=self.limb_neutral_positions)
            #print(str(self._arm_id))

            for i in range(len(joint_indices)):
                #print ("Writing joint   " + str(i) + "to pos " + str(self.limb_neutral_positions[i]))
                pybullet.resetJointState(bodyUniqueId=self._arm_id,
                    jointIndex=i,
                    targetValue=self.limb_neutral_positions[i])
                #actual_joint_pos = self.get_joint_position((self._arm_id, i))
                #print("Actual joint positions:       ")
                #print(actual_joint_pos)

                pybullet.setJointMotorControl2(bodyIndex=self._arm_id,
                            jointIndex=i,
                            controlMode=pybullet.POSITION_CONTROL,
                            targetPosition=self.limb_neutral_positions[i],
                            targetVelocity=0,
                            force=250,
                            positionGain=0.1,
                            velocityGain=1.2)

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
    def get_joint_limits(self):
        """ Get list of all joint limits

        Args: Non
        Returns: list of joint limit dictionaries
        """
        joint_limits = [self.get_joint_limit(joint_index) for joint_index in range(self._num_joints)]
        return joint_limits

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

        Uses IK to determine desired joint configuration to acheive ee position.
        Uses position control with defaulat parameters to move to calculated
        joint configuration

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


        pybullet.setJointMotorControlArray(
            bodyUniqueId=self._arm_id,
            jointIndices=range(0,self._num_joints),
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=qd)
        return q

    def set_to_torque_mode(self):
        """ Set the torques to the
        """
        # Pybullet uses velocity motors by default. Setting max force to 0
        # allows for torque control
        maxForce = 0.00
        mode = pybullet.VELOCITY_CONTROL

        for i in range(self._dof):
            pybullet.setJointMotorControl2(
                self._arm_id, i, mode, force=maxForce)


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
            jointIndices=range(0,self._num_joints),
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocities=dq_d)

    ########### OPERATIONAL SPACE PROPERTIES ##################
    @property
    def ee_v(self):
        link_state = pybullet.getLinkState(self._arm_id,
            self._ee_index,
            computeLinkVelocity=1
            )
        return link_state[6]

    @property
    def ee_w(self):
        link_state = pybullet.getLinkState(self._arm_id,
            self._ee_index,
            computeLinkVelocity=1
            )
        return link_state[7]

    @property
    def ee_twist(self):
        return np.hstack((self.ee_v, self.ee_w))

    @property
    def rotation_matrix(self):
        """ Return rotation matrix from quaternion as 9x1 array
        """
        return np.asarray(
            pybullet.getMatrixFromQuaternion(self.ee_orientation))


    @property
    def state_dict(self):
        """ Return a dictionary containing the robot state,
        useful for controllers"""
        state = {
            "ee_pose" : self.ee_pose,
            "ee_twist" : self.ee_twist,
            "R": self.rotation_matrix,
            "jacobian" : self.jacobian,
            "lambda" : self.op_space_mass_matrix,
            "mass_matrix": self.mass_matrix,
            "N_x": self.N_x,
            "N_q": self.N_q,
            "joint_positions" : self.motor_joint_positions[:7],
            "joint_velocities" : self.motor_joint_velocities[:7],
            "nullspace_matrix" : self.nullspace_matrix
        }
        return state

    def get_motor_joint_indices(self):
        """ Go through urdf and get joint indices of "free" joints.
        """
        motor_joint_indices = []

        for joint_index in range(self.num_joints):

            info = pybullet.getJointInfo(
                bodyUniqueId=self._arm_id,
                jointIndex=joint_index,
                physicsClientId=self._physics_id)

            if (info[2] != pybullet.JOINT_FIXED):
                motor_joint_indices.append(joint_index)

        return motor_joint_indices

    @property
    def num_free_joints(self):
        return len(self._motor_joint_indices)


    @property
    def num_joints(self):
        """ Return number of joints in urdf.
        Note: This is total number of joints, not number of free joints
        """
        return pybullet.getNumJoints(self._arm_id)

    @property
    def dof(self):
        """ Number of degrees of freedom of robot.
        Note: this may not be the same as number of "free" joints according to
        pybullet. Pybullet counts prismatic joints in gripper.
        """
        return 7

    @property
    def jacobian(self):
        """ Return jacobian J(q,dq) for the current robot configuration, where
        xdot = J*qdot

        NOTE: this is trimmed to make a dof x dof matrix. (not including
        grippers).
        """

        linear_jacobian, angular_jacobian = pybullet.calculateJacobian(
            bodyUniqueId=self._arm_id,
            linkIndex=6,
            localPosition=[9.3713e-08,    0.28673,  -0.237291],
            objPositions=self.motor_joint_positions,
            objVelocities=self.motor_joint_velocities,
            objAccelerations=[0]*len(self.motor_joint_positions),
            physicsClientId=self._physics_id
            )
        linear_jacobian = np.reshape(
            linear_jacobian, (3, self.num_free_joints))

        angular_jacobian = np.reshape(
            angular_jacobian, (3, self.num_free_joints))

        jacobian = np.vstack(
            (linear_jacobian[:,:self.dof],angular_jacobian[:,:self.dof]))

        return jacobian

    @property
    def mass_matrix(self):

        mass_matrix = pybullet.calculateMassMatrix(
            bodyUniqueId=self._arm_id,
            objPositions=self.motor_joint_positions,
            physicsClientId=self._physics_id)
        print(self.motor_joint_positions)
        mass_matrix = np.reshape(mass_matrix,
            (self.num_free_joints, self.num_free_joints))

        mass_matrix = mass_matrix[:self.dof,:self.dof]
        return mass_matrix


    def get_dof(self):
        """ Return number of free joints according to pybullet
        """
        dof=0
        joint_infos = [pybullet.getJointInfo(self._arm_id, i) for i in range(pybullet.getNumJoints(self._arm_id))]
        for info in joint_infos:
            if info[2] != pybullet.JOINT_FIXED:
                print(info[1])
                dof+=1
        return dof

    def get_joint_max_velocities(self):
        """ Get the max velocities for each not fixed joint.
        """
        joint_max_velocities = []
        for joint_limit in self.joint_limits:
            max_vel = joint_limit.get('velocity')
            joint_max_velocities.append(max_vel)
        return np.asarray(joint_max_velocities[:7])

    def get_joint_max_forces(self):
        """ Get the max velocities for each not fixed joint.
        """
        joint_max_forces = []
        for joint_limit in self.joint_limits:
            max_force = joint_limit.get('effort')
            joint_max_forces.append(max_force)
        return np.asarray(joint_max_forces[:7])


    def get_joint_velocity_control(self, dqd,
        kv=[2000, 1500, 1500, 1500, 1500, 500, 1000]):
        """Joint velocity control.
        dqd = (q-qd) / dt
        Torques = -kv(dq - dqd) + g(q)  + b(q,dq)
        Args:
            qd (list): desired joint positions as a column vector
            kv(list): gain for velocity error
        Returns:
            joint_torques (list): list (ndof) of torques to command
        """
        # Differentiate to get the desired joint velocities
        dq = np.asarray(self.motor_joint_velocities[:7])

        # clip desired velocities to joint limits
        dqd = np.clip(
            dqd,
            -self._joint_max_velocities,
            self._joint_max_velocities)

        control_law = -np.multiply(kv, (dq-dqd))
        # append 0 torques for gripper motors
        control_law = np.append(control_law, [0,0])
        control_law = np.dot(self.mass_matrix, control_law)

        dynamics_torques = pybullet.calculateInverseDynamics(
            bodyUniqueId=self._arm_id,
            objPositions=self.motor_joint_positions,
            objVelocities=self.motor_joint_velocities,
            objAccelerations=[0]*len(self.motor_joint_positions),
            physicsClientId=self._physics_id
            )

        compensated_torques = control_law + dynamics_torques

        compensated_torques = np.clip(
            compensated_torques[:7],
            -self._joint_max_forces[:7],
            self._joint_max_forces[:7])

        return compensated_torques[:7]


    @property
    def N_x(self):
        """ get combined gravity, coriolis and centrifigual terms
        in op space"""

        inv_J_t = np.linalg.pinv(np.transpose(self.jacobian))
        Nx = np.dot(inv_J_t, self.N_q)
        return Nx

    @property
    def JBar(self):
        """ dynamically consistent inverse jacobian
        """
        JBar = np.dot(np.linalg.inv(self.mass_matrix),
                np.dot(np.transpose(self.jacobian), self.op_space_mass_matrix[2]))
        return JBar

    @property
    def N_q(self):
        Nq = pybullet.calculateInverseDynamics(
            bodyUniqueId=self._arm_id,
            objPositions=self.motor_joint_positions,
            objVelocities=self.motor_joint_velocities,
            objAccelerations=[0]*len(self.motor_joint_positions),
            physicsClientId=self._physics_id
            )
        return np.asarray(Nq)[:7]

    @property
    def op_space_mass_matrix(self):
        """ Compute operational space mass matrix given joint positiosn
        """
        inv_M = np.linalg.inv(self.mass_matrix)

        lambda_x_matrix_inv = np.dot(
            np.dot(self.linear_jacobian, inv_M), np.transpose(self.linear_jacobian))

        lambda_r_matrix_inv = np.dot(
            np.dot(self.angular_jacobian, inv_M), np.transpose(self.angular_jacobian))

        lambda_inv = np.dot(
            np.dot(self.jacobian, inv_M), np.transpose(self.jacobian))

        svd_u, svd_s, svd_v = np.linalg.svd(lambda_inv)
        singularity_threshold = 0.00025
        svd_s_inv = [0 if x<singularity_threshold else 1./x for x in svd_s]
        lambda_matrix = svd_v.T.dot(np.diag(svd_s_inv)).dot(svd_u.T)
        lambda_matrix = np.linalg.inv(lambda_inv)

        # zero out elements in case of singularity
        svd_u, svd_s, svd_v = np.linalg.svd(lambda_x_matrix_inv)
        singularity_threshold = 0.00025
        svd_s_inv = [0 if x<singularity_threshold else 1./x for x in svd_s]
        lambda_x_matrix = svd_v.T.dot(np.diag(svd_s_inv)).dot(svd_u.T)

        svd_u, svd_s, svd_v = np.linalg.svd(lambda_r_matrix_inv)
        singularity_threshold = 0.00025
        svd_s_inv = [0 if x<singularity_threshold else 1./x for x in svd_s]
        lambda_r_matrix = svd_v.T.dot(np.diag(svd_s_inv)).dot(svd_u.T)

        return (lambda_x_matrix, lambda_r_matrix, lambda_matrix)

    @property
    def mass_matrix(self):
        """ compute the system inertia given its joint positions. Uses
        pybullet Composite Rigid Body Algorithm.
        """
        mass_matrix = pybullet.calculateMassMatrix(
            bodyUniqueId=self._arm_id,
            objPositions=self.q,
            physicsClientId=self._physics_id)
        mass_matrix = np.array(mass_matrix)[:7, :7]
        return mass_matrix

    @property
    def jacobian(self):
        """ calculate the jacobian for the end effector position at current
        joint state.

        Returns:
            jacobian tuple(mat3x, mat3x): translational jacobian ((dof), (dof), (dof))
                and angular jacobian  ((dof), (dof), (dof))

        Notes:
            localPosition: point on the specified link to compute the jacobian for, in
            link coordinates around its center of mass. by default we assume we want it
            around ee center of mass.

        TODO: Verify this jacobian cdis what we want or if ee position is further from
            com.
        """

        motor_pos, motor_vel, motor_accel = self.getMotorJointStates()
        linear, angular = pybullet.calculateJacobian(
            bodyUniqueId=self._arm_id,
            linkIndex=self._ee_index,
            localPosition=[0.0,0,0],
            objPositions=motor_pos,
            objVelocities=[0]*len(motor_pos),
            objAccelerations=[0]*len(motor_pos),
            physicsClientId=self._physics_id)


        jacobian = np.vstack((linear, angular))
        jacobian = jacobian[:7, :7]
        return jacobian
    @property
    def linear_jacobian(self):
        """The linear jacobian x_dot = J_t*q_dot
        """
        motor_pos, motor_vel, motor_accel = self.getMotorJointStates()
        linear, _ = pybullet.calculateJacobian(
            bodyUniqueId=self._arm_id,
            linkIndex=self._ee_index,
            localPosition=[0.0,0,0],
            objPositions=motor_pos,
            objVelocities=[0]*len(motor_pos),
            objAccelerations=[0]*len(motor_pos),
            physicsClientId=self._physics_id)

        return np.asarray(linear)[:, :7]

    @property
    def angular_jacobian(self):
        """ The angular jacobian rdot= J_r*qdot
        """
        motor_pos, motor_vel, motor_accel = self.getMotorJointStates()
        _, angular = pybullet.calculateJacobian(
            bodyUniqueId=self._arm_id,
            linkIndex=self._ee_index,
            localPosition=[0.0,0,0],
            objPositions=motor_pos,
            objVelocities=[0]*len(motor_pos),
            objAccelerations=[0]*len(motor_pos),
            physicsClientId=self._physics_id)

        return np.asarray(angular)[:,:7]

    def getMotorJointStates(self):
      joint_states = pybullet.getJointStates(
        self._arm_id, range(pybullet.getNumJoints(self._arm_id)))

      joint_infos = [pybullet.getJointInfo(self._arm_id, i) for i in range(pybullet.getNumJoints(self._arm_id))]
      joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
      joint_positions = [state[0] for state in joint_states]
      joint_velocities = [state[1] for state in joint_states]
      joint_torques = [state[3] for state in joint_states]
      return joint_positions, joint_velocities, joint_torques


    @property
    def motor_joint_positions(self):
        """ returns the motor joint positions for "each DoF" according to pybullet.

        Note: fixed joints have 0 degrees of freedoms.
        """
        joint_states = pybullet.getJointStates(
        self._arm_id, range(pybullet.getNumJoints(self._arm_id)))
        # Joint info specifies type of joint ("fixed" or not)
        joint_infos = [pybullet.getJointInfo(self._arm_id, i) for i in range(pybullet.getNumJoints(self._arm_id))]
        # Only get joint states of free joints
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[2] != pybullet.JOINT_FIXED]

        joint_positions = [state[0] for state in joint_states]

        return joint_positions

    @property
    def motor_joint_velocities(self):
        """ returns the motor joint positions for "each DoF" according to pybullet.

        Note: fixed joints have 0 degrees of freedoms.
        """
        joint_states = pybullet.getJointStates(
        self._arm_id, range(pybullet.getNumJoints(self._arm_id)))
        # Joint info specifies type of joint ("fixed" or not)
        joint_infos = [pybullet.getJointInfo(self._arm_id, i) for i in range(pybullet.getNumJoints(self._arm_id))]
        # Only get joint states of free joints
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[2] != pybullet.JOINT_FIXED]

        joint_velocities = [state[1] for state in joint_states]

        return joint_velocities

    @property
    def motor_joint_accelerations(self):
        """ returns the motor joint positions for "each DoF" according to pybullet.

        Note: fixed joints have 0 degrees of freedoms.
        """
        joint_states = pybullet.getJointStates(
        self._arm_id, range(pybullet.getNumJoints(self._arm_id)))
        # Joint info specifies type of joint ("fixed" or not)
        joint_infos = [pybullet.getJointInfo(self._arm_id, i) for i in range(pybullet.getNumJoints(self._arm_id))]
        # Only get joint states of free joints
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[2] != pybullet.JOINT_FIXED]

        joint_accelerations = [state[2] for state in joint_states]

        return joint_accelerations
    @property
    def gravity_vector(self):
        """Compute the gravity vector at the current joint state.

        Pybullet does not currently expose the gravity vector so we compute it
        using inverse dynamics.

        Args : None
        Returns:
            gravity_torques  (list): num_joints x 1 list of gravity torques on each joint.

        Notes: to ignore coriolis forces we set the object velocities to zero.
        """

        gravity_forces = pybullet.calculateInverseDynamics(
            bodyUniqueId=self._arm_id,
            objPositions=self.q,
            objVelocities=[0]*self._num_joints,
            objAccelerations=[0]*self._num_joints,
            physicsClientId=self._physics_id)

        gravity_torques = np.transpose(self.jacobian)*gravity_forces

    @property
    def nullspace_matrix(self):
        return np.eye(7,7) - np.dot(self.JBar, self.jacobian)

    def set_torques(self, joint_torques):
        """Set torques to the motor. Useful for keeping torques constant through
        multiple simulation steps.

        Args: joint_torques (list): list of joint torques with dimensions (num_joints,)
        """
        clipped_torques = np.clip(
            joint_torques[:7],
            -self._joint_max_forces[:7],
            self._joint_max_forces[:7])
        logging.debug(clipped_torques)
        # For some reason you have to keep disabling the velocity motors
        # before every torque command.
        self.set_to_torque_mode()
        pybullet.setJointMotorControlArray(
            bodyUniqueId=self._arm_id,
            jointIndices=range(0,7),
            controlMode=pybullet.TORQUE_CONTROL,
            forces=clipped_torques)
        return clipped_torques

    @property
    def last_torques(self):
        last_torques =[]
        for joint in range(7):
            positions, velocities, forces, torque = pybullet.getJointState(self._arm_id,
                    joint,
                    self._physics_id)
            last_torques.append(torque)
        return last_torques
