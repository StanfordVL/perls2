from perls2.controllers.ee_imp import EEImpController
from perls2.controllers.robot_model.model import Model
from perls2.controllers.utils.control_utils import *
import perls2.controllers.utils.transform_utils as T
import numpy as np
import time

class EEPostureController(EEImpController):
    """ Class definition for end effector impedance controller with posture control. 

    End effector uses pd control, with a desired joint position in null space.
    
    Attributes:
        posture (list): 7f joint positions to maintain within nullspace of task. 
        posture_gain (float): constant gain to control posture. 

    """
    def __init__(self, 
                 robot_model,
                 input_max=1,
                 input_min=-1,
                 output_max=1.0,
                 output_min=-1.0,
                 kp=50,
                 kv=None,
                 damping=1,
                 posture_gain=0,
                 posture=[0,-1.18,0.00,2.18,0.00,0.57,3.3161],
                 control_freq=20,
                 position_limits=None,
                 orientation_limits=None,
                 interpolator_pos=None,
                 interpolator_ori=None,
                 uncouple_pos_ori=False, ):
        """ Initialize controller.
        """
        self.posture_gain = np.asarray(posture_gain)
        self.goal_posture = np.asarray(posture)
        super(EEPostureController, self).__init__(
            robot_model=robot_model, 
            input_max=input_max,
            input_min=input_min,
            output_max=output_max,
            output_min=output_min,
            kp=kp,
            kv=kv,
            damping=damping,
            control_freq=control_freq,
            position_limits=position_limits,
            orientation_limits=orientation_limits,
            interpolator_pos=interpolator_pos,
            interpolator_ori=interpolator_ori,
            uncouple_pos_ori=uncouple_pos_ori)
        # Compile numba jit in advance to reduce initial calc time.
        self._compile_jit_functions()

    def _compile_jit_functions(self):
        dummy_mat = np.eye(3)
        dummy_quat = np.zeros(4)
        dummy_quat[-1] = 1.
        T.mat2quat(dummy_mat)
        T.quat2mat(dummy_quat)

        #dummy_nullspace_matrix = np.zeros((7, 7))
        _, _, _, dummy_nullspace_matrix =opspace_matrices(
            mass_matrix=self.model.mass_matrix,
            J_full=self.model.J_full,
            J_pos=self.model.J_pos,
            J_ori=self.model.J_ori,
        )
        orientation_error(dummy_mat, dummy_mat)

        nullspace_torques(
            mass_matrix=self.model.mass_matrix, 
            nullspace_matrix=dummy_nullspace_matrix, 
            initial_joint=self.goal_posture,
            joint_pos=self.model.joint_pos,
            joint_vel=self.model.joint_vel,
        )

    def set_goal(self, delta,  
        set_pos=None, set_ori=None, **kwargs):
        super(EEPostureController, self).set_goal(delta, set_pos, set_ori)
    
    def run_controller(self):
        """ Run controller to calculate torques to acheive desired pose. 

        Args: None
        Return: 
            torques (list): 7f list of torques to command. 
        
        Runs impedance controllers and adds nullspace constrained posture compensation. 
        """
        
        torques = super(EEPostureController, self).run_controller()
        self.torques = torques + nullspace_torques(self.model.mass_matrix, 
                                          self.nullspace_matrix, 
                                          self.goal_posture, 
                                          self.model.joint_pos, 
                                          self.model.joint_vel, 
                                          self.posture_gain)



        return self.torques

