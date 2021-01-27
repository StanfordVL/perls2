import rospy
from sensor_msgs.msg import JointState
import redis
import hiredis
from perls2.utils.yaml_config import YamlConfig
from perls2.redis_interfaces.redis_interface import RobotRedisInterface as RobotRedis
from perls2.redis_interfaces.redis_keys import *
import json
import numpy as np
try:
    import intera_interface as iif
except ImportError:
    print('intera_interface not imported, did you remember to run cd ~/ros_ws;./intera.sh?')
import time
def on_joint_states(msg):
    # Set initial redis keys
    global redisClient
    global _limb

    robot_state = {
        ROBOT_STATE_TSTAMP_KEY : str(msg.header.stamp),
        ROBOT_STATE_EE_POS_KEY :  str(list(_limb.endpoint_pose()['position'])),
        ROBOT_STATE_EE_POSE_KEY: str(list(_limb.endpoint_pose()['position']) + list(_limb.endpoint_pose()['orientation'])),
        ROBOT_STATE_EE_ORN_KEY: str(list(_limb.endpoint_pose()['orientation'])),
        ROBOT_STATE_EE_V_KEY: str(list(_limb.endpoint_velocity()['linear'])),
        ROBOT_STATE_Q_KEY: str( _limb.joint_ordered_angles()),
        ROBOT_STATE_DQ_KEY: json.dumps(_limb.joint_velocities()),
        ROBOT_STATE_TAU_KEY: json.dumps(_limb.joint_efforts()),
        ROBOT_MODEL_JACOBIAN_KEY: str(np.zeros((6,7))),
        ROBOT_MODEL_L_JACOBIAN_KEY: str(np.zeros((3,6))),
        ROBOT_MODEL_A_JACOBIAN_KEY: str(np.zeros((3,6))),
        ROBOT_MODEL_MASS_MATRIX_KEY: str(np.zeros((7,7)))
    }
    redisClient.mset(robot_state)

print("Initializing ros redis interface.")
rospy.init_node("ros_redis_interface")
_limb = iif.Limb(limb="right", synchronous_pub=False)

config = YamlConfig('cfg/sawyer_ctrl_config.yaml')
# redis_kwargs = config['redis']
# if 'localhost' not in redis_kwargs['host']:
#     redis_kwargs['host'] = socket.gethostbyname(config['redis']['host'])

# redisClient = redis.Redis(**redis_kwargs)

# redisClient.flushall()
redisClient = RobotRedis(**config['redis'])

joint_state_topic = 'robot/joint_states'
_joint_state_sub = rospy.Subscriber(
    joint_state_topic,
    JointState,
    on_joint_states,
    queue_size=1,
    tcp_nodelay=True)

print("Running ROS Redis interface.")
rospy.spin()