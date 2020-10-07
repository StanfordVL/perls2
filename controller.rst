Controller
==========
.. toctree::
  :maxdepth: 2
  :caption: Contents:

Every Robot is equipped with a controller, which determines both the action space as well as how its
values are mapped into command torques. By default, all controllers have a pre-defined set of methods and
properities, though specific controllers may extend and / or override the default functionality found in
the base class.

Base Controller
---------------

.. autoclass:: perls2.controllers.base_controller.Controller

  .. automethod:: run_controller
  .. automethod:: scale_action


Joint Impedance Controller
--------------------------

.. autoclass:: perls2.controllers.joint_imp.JointImpController

  .. automethod:: set_goal
  .. automethod:: run_controller



Joint Velocity Controller
-------------------------

.. autoclass:: perls2.controllers.joint_vel.JointVelController

  .. automethod:: set_goal
  .. automethod:: run_controller



Joint Torque Controller
-----------------------

.. autoclass:: perls2.controllers.joint_torque.JointTorqueController

  .. automethod:: set_goal
  .. automethod:: run_controller


EEImpedance Controller
--------------------------

.. autoclass:: perls2.controllers.ee_imp.EEImpController

  .. automethod:: set_goal
  .. automethod:: run_controller



EEPosture Controller
--------------------------

.. autoclass:: perls2.controllers.ee_posture.EEPostureController

  .. automethod:: set_goal
  .. automethod:: run_controller



Robot Model
--------------------------
.. autoclass:: perls2.controllers.robot_model.model.Model
  :members: perls2.controllers.robot_model.model.Model.ee_pos_vel

  .. automethod:: update_states
  .. automethod:: update_model

  .. autoproperty:: ee_pos
  .. autoproperty:: ee_ori_mat
  .. autoproperty:: ee_ori_quat
  .. autoproperty:: ee_pos_vel
  .. autoproperty:: ee_ori_vel
  .. autoproperty:: joint_pos
  .. autoproperty:: joint_vel
  .. autoproperty:: joint_tau
  .. autoproperty:: joint_dim
  .. autoproperty:: J_pos
  .. autoproperty:: J_ori
  .. autoproperty:: J_full
  .. autoproperty:: mass_matrix
  .. autoproperty:: offset_mass_matrix
  .. autoproperty:: mass_matrix_offset_val
  .. autoproperty:: torque_compensation
  .. autoproperty:: nullspace