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

# Redis reads commands as bytes strings
bSET_EE_POSE = bytes(SET_EE_POSE, 'utf-8')
bMOVE_EE_DELTA = bytes(MOVE_EE_DELTA, 'utf-8')
bSET_JOINT_DELTA = bytes(SET_JOINT_DELTA, 'utf-8')
bSET_JOINT_POSITIONS = bytes(SET_JOINT_POSITIONS, 'utf-8')
bSET_JOINT_VELOCITIES = bytes(SET_JOINT_VELOCITIES, 'utf-8')
bSET_JOINT_TORQUES = bytes(SET_JOINT_TORQUES, 'utf-8')
bIDLE = bytes(IDLE, 'utf-8')
bCHANGE_CONTROLLER = bytes(CHANGE_CONTROLLER, 'utf-8')
