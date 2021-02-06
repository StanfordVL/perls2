""" List of redis keys for the robot
"""
ROBOT_KEY = "robot::"

# Robot command keys
ROBOT_RESET_COMPL_KEY = ROBOT_KEY + "reset_complete"
ROBOT_RESET_Q_KEY = ROBOT_KEY + "reset_q"
ROBOT_CMD_TYPE_KEY = ROBOT_KEY + "cmd_type"
ROBOT_ENV_CONN_KEY = ROBOT_KEY + "env_connected"
ROBOT_CMD_TSTAMP_KEY = ROBOT_KEY + "cmd_tstamp"
ROBOT_LAST_CMD_TSTAMP = ROBOT_KEY + "last_cmd_tstamp"
ROBOT_SET_GRIPPER_CMD_KEY = ROBOT_KEY + "set_gripper_value"
ROBOT_SET_GRIPPER_CMD_TSTAMP_KEY = ROBOT_KEY + "set_gripper_cmd_tstamp"

# Robot State Keys
STATE_KEY = "proprio::"
ROBOT_STATE_KEY =  ROBOT_KEY + STATE_KEY
ROBOT_STATE_TSTAMP_KEY = ROBOT_STATE_KEY + "tstamp"
ROBOT_STATE_Q_KEY = ROBOT_STATE_KEY + "q"
ROBOT_STATE_DQ_KEY = ROBOT_STATE_KEY + "dq"
ROBOT_STATE_EE_POS_KEY = ROBOT_STATE_KEY + "ee_position"
ROBOT_STATE_EE_ORN_KEY = ROBOT_STATE_KEY + "ee_orientation"
ROBOT_STATE_EE_POSE_KEY = ROBOT_STATE_KEY + "ee_pose"
ROBOT_STATE_EE_V_KEY = ROBOT_STATE_KEY +"ee_v"
ROBOT_STATE_EE_OMEGA_KEY = ROBOT_STATE_KEY + "ee_omega"
ROBOT_STATE_TAU_KEY = ROBOT_STATE_KEY + "tau"

# Robot Controller Model Keys
MODEL_KEY = "model::"
ROBOT_MODEL_KEY = ROBOT_KEY + MODEL_KEY
ROBOT_MODEL_JACOBIAN_KEY = ROBOT_MODEL_KEY + "jacobian"
ROBOT_MODEL_L_JACOBIAN_KEY = ROBOT_MODEL_KEY + "linear_jacobian"
ROBOT_MODEL_A_JACOBIAN_KEY = ROBOT_MODEL_KEY + "angular_jacobian"
ROBOT_MODEL_MASS_MATRIX_KEY = ROBOT_MODEL_KEY + "mass_matrix"

# Robot Controller Keys
CONTROL_KEY = "controller::"
CONTROLLER_CONTROL_TYPE_KEY = CONTROL_KEY + "control_type"
CONTROLLER_GOAL_KEY = CONTROL_KEY + "goal"
CONTROLLER_CONTROL_PARAMS_KEY = CONTROL_KEY + "control_params"

# Kinect2 Key
KINECT2_KEY = 'kinect2::'
KINECT2_RGB_KEY = KINECT2_KEY + 'rgb_frame'
KINECT2_RGB_TSTAMP_KEY = KINECT2_KEY + 'rgb_timestamp'
KINECT2_RES_MODE_KEY = KINECT2_KEY + 'res_mode'
KINECT2_DEPTH_KEY = KINECT2_KEY + 'depth_frame'
KINECT2_IR_KEY = KINECT2_KEY + 'ir_frame'
KINECT2_INVERT_KEY = KINECT2_KEY + 'invert'
KINECT2_STREAM_ENABLED_KEY = KINECT2_KEY + 'stream_enabled'
KINECT2_INTERFACE_CONN_KEY = KINECT2_KEY + 'interface_connected'

# ROS Camera suffixes
ROS_ENV_CONN_SUFF = "::env_connected"
ROS_RGB_SUFF = "::rgb_frame"
ROS_RGB_TSTAMP_SUFF = "::rgb_timestamp"
ROS_DEPTH_SUFF = "::depth_frame"

# Flag values
TRUE = "True"
FALSE = "False"

""" References to create uniformity for setting redis values.
"""
# Controller types
JOINT_IMPEDANCE = "JointImpedance"
JOINT_VELOCITY = "JointVelocty"
JOINT_TORQUE = "JointTorque"
EE_IMPEDANCE = "EEImpedance"
EE_POSTURE = "EEPosture"

# Robot Commands
RESET = "reset_to_neutral"
CHANGE_CONTROLLER = "CHANGE_CONTROLLER"
MOVE_EE_DELTA = "move_ee_delta"
SET_EE_POSE = "set_ee_pose"
SET_JOINT_DELTA = "set_joint_delta"
SET_JOINT_POSITIONS = "set_joint_positions"
SET_JOINT_VELOCITIES = "set_joint_velocities"
SET_JOINT_TORQUES = "set_joint_torques"
IDLE = "IDLE"
