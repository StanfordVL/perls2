"""
Class defining the interface to the Sawyer Robots in PyBullet.
Author: Roberto Martin-Martin
        Rohun Kulkarni
"""

import pybullet
import numpy as np

from perls2.robots.bullet_robot_interface import BulletRobotInterface


class BulletSawyerInterface(BulletRobotInterface):
    """ Class for Sawyer Robot Interface in Pybullet. This class provides the
    functionsfor information about the state of the robot as well as sending
    commands.


    """

    def __init__(self,
                 physics_id,
                 arm_id,
                 config=None,
                 controlType='EEImp'):
        """
        Initialize variables

        Parameters
        ----------
            pose: list of seven floats (optional)
            Pose of robot base in world frame
            x y z qx qy qz qw
        """
        super().__init__(physics_id, arm_id, config, controlType)
        self._ee_index = self.get_link_id_from_name('right_hand')

        # Neutral positions and configuration specifics
        self.limb_neutral_positions = [0,-1.18,0.00,2.18,0.00,0.57,3.3161, 0, 0, 0, 0, 0, 0, 0]
        self._name = "Rethink Bullet Sawyer"
        print("BulletSawyerInterface created")

    def start(self):
        """Start the robot
        """
        raise NotImplementedError

    @BulletRobotInterface.ee_index.getter
    def ee_index(self):
        return self.get_link_id_from_name("right_hand")

    def set_joints_to_neutral_positions(self):
        """Set joints on robot to neutral
        """
        if self._arm_id is None:
            print("arm id not set")
        else:
            self._num_joints = pybullet.getNumJoints(self._arm_id)

            joint_indices = [i for i in range(0,self._num_joints)]

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
    @property
    def version(self):
        """dict of current versions of robot SDK, gripper, and robot
        """
        return {
                'robot' : '5.0.4',
                'gripper': 'rethink_ee',
                'robotSDK': '5.0.4'
                }

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
        return super().q

    @q.setter
    def q(self, qd):
        """
        Get the joint configuration of the robot arm.
        Args:
            qd: list
                list of desired joint position
        """
        pybullet.setJointMotorControlArray(
            bodyIndex=self._arm_id,
            jointIndices=range(0,self._num_joints),
            controlMode=pybullet.POSITION_CONTROL,
            targetPositions=qd,
            forces=[250]*self._num_joints,
            positionGains=[0.1]*self._num_joints,
            velocityGains=[1.2]*self._num_joints)




