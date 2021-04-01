"""Constant keys for franka-panda redis driver. 
"""
DRIVER_PREFIX = "franka_panda::"

## Read by driver: 

# Control Mode Keys
CONTROL_MODE_KEY = DRIVER_PREFIX + "control::mode"

# Control Mode Values
TORQUE_CTRL_MODE = "torque"
FLOAT_CTRL_MODE = "floating"
RESET_CTRL_MODE = "reset"
IDLE_CTRL_MODE = "idle"

# Gripper Keys:
GRIPPER_MODE_KEY = DRIVER_PREFIX + "gripper::control::mode"
GRIPPER_WIDTH_CMD_KEY = DRIVER_PREFIX + "gripper::control::width"
GRIPPER_SPEED_CMD_KEY = DRIVER_PREFIX + "gripper::control::speed"
GRIPPER_FORCE_CMD_KEY = DRIVER_PREFIX + "gripper::control::force"

# Torque cmd key: 
TORQUE_CMD_KEY = DRIVER_PREFIX + "control::tau"
## Set by driver: 

# Driver connection status key (if driver is running.)
DRIVER_CONN_KEY = DRIVER_PREFIX + "driver::status"

# Driver connection status values
DRIVER_CONNECTED_VALUE = "running"
DRIVER_DISCONN_VALUE = "off"

# Robot state keys:
ROBOT_STATE_Q_KEY = DRIVER_PREFIX + "sensor::q"
ROBOT_STATE_DQ_KEY = DRIVER_PREFIX + "sensor::dq"
ROBOT_STATE_EE_POSE_KEY = DRIVER_PREFIX + "sensor::pose"
ROBOT_STATE_TAU_KEY = DRIVER_PREFIX + "sensor::tau"

ROBOT_MODEL_MASS_MATRIX_KEY = DRIVER_PREFIX + "model::mass_matrix"
ROBOT_MODEL_JACOBIAN_KEY = DRIVER_PREFIX + "model::jacobian"
ROBOT_MODEL_GRAVITY_KEY = DRIVER_PREFIX + "model::gravity"
ROBOT_MODEL_CORIOLIS_KEY = DRIVER_PREFIX + "model::coriolis"

# Gripper state keys: 
GRIPPER_WIDTH_KEY = DRIVER_PREFIX + "gripper::sensor::width"
GRIPPER_MAX_WIDTH_KEY = DRIVER_PREFIX + "gripper::model::max_width"
GRIPPER_STATUS_KEY = DRIVER_PREFIX + "gripper::status"

# Reset neutral joint angle key: 
RESET_Q_KEY = DRIVER_PREFIX + "control::reset_q"

## Convenience defines
ROBOT_STATE_KEYS = [
			ROBOT_STATE_Q_KEY, 
			ROBOT_STATE_DQ_KEY,
			ROBOT_STATE_EE_POSE_KEY, 
			ROBOT_STATE_TAU_KEY, 
			]

ROBOT_MODEL_KEYS = [
	ROBOT_MODEL_MASS_MATRIX_KEY, 
	ROBOT_MODEL_JACOBIAN_KEY, 
	ROBOT_MODEL_GRAVITY_KEY, 
	ROBOT_MODEL_CORIOLIS_KEY]

MASS_MATRIX_SHAPE = (7,7)
JACOBIAN_SHAPE = (6,7)
EE_POSE_SHAPE = (4,4)