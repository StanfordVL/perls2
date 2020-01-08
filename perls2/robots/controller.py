"""
Abstract class for controllers

"""
import pybullet  # used for calculations. should remove as dep
import numpy as np
from scipy.spatial.transform import Rotation as R


class Controller():
    """ Abstract class to be implemented for controllers
    """

    def __init__(self, type, config):
        """
        Args:
            type (str): type of controller (joint space, ops, impedance)
            config (dict): dictionary specifying parameters for controller.
        """
        raise NotImplementedError

class OperationalSpaceController(Controller):
    """ Implement operational space control functions
    """

    def __init__(self,
        kpx=[10, 10, 10, 1.50, 1.50, 1.50],
        kvx=[3, 3, 3, 0.20, 0.20, 0.20],
        damping=0.75):
        """ Initialize controller with default gains

        Args:
            kpx (list): gains for position error in op space (dof x 1)
            kvx (list): gains for velocity in op space (dof x 1)
        """
        self.kpx = np.ones(6)*100
        self.kvx = np.ones(6)* 2* np.sqrt(self.kpx)*damping
        print(self.kvx)

    def get_states(self, state):
        self.lambda_x = state.get('lambda')[0]
        self.lambda_r = state.get('lambda')[1]
        self.lambda_mat = state.get('lambda')[2]

        self.mass_matrix = np.reshape(state.get('mass_matrix'), (7,7))
        self.ee_dx = state.get('ee_twist')
        self.R = state.get('R')
        self.x = state.get('ee_pose')
        self.jacobian = state.get('jacobian')

        self.joint_position = state.get('joint_positions')
        self.joint_velocity = state.get('joint_velocities')
        self.nullspace_torques = state.get('nullspace_matrix')
        self.N_q = state.get('N_q')

    def compute_torques(self, state, xd, kpx=None, kvx=None):
        """ Compute the torques based on error and compensation for inertia

        Args:
            state (dict): dictionary containing robot states

        """
        # Get the robot states
        self.get_states(state)

        if kpx==None:
            kpx = self.kpx
        if kvx==None:
            kvx = self.kvx

        error_pos = np.subtract(self.x[:3],xd[:3])
        Rd = np.asarray(pybullet.getMatrixFromQuaternion(xd[3:]))
        error_orn = self.calculate_orientation_error(Rd, self.R)
        error_orn_aa = self.calculate_orientation_error_aa(Rd, self.R)

        error = np.hstack((error_pos, error_orn))

        # zero out error if close enough
        for index in range(len(error)):
            if np.abs(error[index]) < .001:
                error[index] = 0

        des_forces = (
            -np.multiply(kpx[:3], error[:3]) - np.multiply(kvx[:3], self.ee_dx[:3]))
        des_torques = (
            -np.multiply(kpx[3:], error[3:])) - np.multiply(kvx[3:], self.ee_dx[3:])

        # Artificially setting des torques to zero to verify decoupling
        #des_torques = [0, 0, 0]
        des_wrench = np.hstack((des_forces, des_torques))

        # decouple
        decoupled_forces = (
            np.dot(self.lambda_x, des_forces))
        decoupled_torques = (
            np.dot(self.lambda_r, des_torques))

        # decoupled_wrench = np.hstack((decoupled_forces, decoupled_torques))
        decoupled_wrench = np.dot(self.lambda_mat, des_wrench)

        osc_torques = np.dot(np.transpose(self.jacobian), decoupled_wrench)

        return  (error_orn, error_orn_aa), error, des_wrench, decoupled_wrench,  osc_torques


    def compute_torques_compensated(self, state, posture, xd,
        kpx=None,
        kvx =None):
        #kvx=[70, 70, 70, 125, 125, 125]):

        """ add compensation to the osc torque
        kpx=[800, 700, 40, 600, 600, 600],
        kvx=[400, 500, 10, 350, 350, 350]"""

        orn_error, error, kpkv_forces, mass_kpkv_forces, osc_torques = self.compute_torques(state, xd, kpx, kvx)
        compensated_torques = osc_torques + self.N_q

        # joint_kp = 10
        # joint_kv = np.sqrt(joint_kp)*2
        # pose_torques = np.dot(self.mass_matrix,np.asarray((posture-self.joint_position) - joint_kv*self.joint_velocity))
        # nullspace_torques = np.dot(self.nullspace_matrix.transpose(), pose_torques)
        # torques += nullspace_torques

        return orn_error, error, kpkv_forces, mass_kpkv_forces, osc_torques, compensated_torques


    def calculate_orientation_error(self, desired, current):
        """
        Optimized function to determine orientation error
        """
        def cross_product(vec1, vec2):
            S= np.array(([0, -vec1[2], vec1[1]],
                        [vec1[2], 0, -vec1[0]],
                        [-vec1[1], vec1[0], 0]))

            return np.dot(S, vec2)

        rc1 = [current[i] for i in range(len(current)) if (i % 3) == 0]
        rc2 = [current[i] for i in range(len(current)) if (i % 3) == 1]
        rc3 = [current[i] for i in range(len(current)) if (i % 3) == 2]
        rd1= [desired[i] for i in range(len(current)) if (i%3)== 0]
        rd2= [desired[i] for i in range(len(current)) if (i%3)== 1]
        rd3= [desired[i] for i in range(len(current)) if (i%3)== 2]

        orientation_error = 0.5 * (cross_product(rc1, rd1) + cross_product(rc2, rd2) + cross_product(rc3, rd3))

        return orientation_error

    def calculate_orientation_error_aa(self, desired, current):
        """Calculate orientation error as the difference between two
        rotation vectors """
        des = (np.reshape(desired, (3,3)))
        curr = (np.reshape(current, (3,3)))
        delta = np.dot(np.transpose(des), curr)
        error = R.from_matrix(delta)
        #error = np.subtract(curr.as_rotvec(),des.as_rotvec())

        return error.as_rotvec()

