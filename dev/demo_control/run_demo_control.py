from demo_control_env import DemoControlEnv
import numpy as np
import time 
import logging
logging.basicConfig(level=logging.INFO)
mpl_logger = logging.getLogger('matplotlib')
mpl_logger.setLevel(logging.WARNING) 
EE_POSITIONS_LIST = [[0.1, 0.0,    0.0, 0.0,    0.0,    0.0],
                      [0.0,  -0.1,   0.0, 0.0,    0.0,    0.0], 
                      [0.0,  0.0,   0.1, 0.0,    0.0,    0.0]]


JOINT_VELOCITY_LIST =[[0.1, 0.0,    0.0, 0.0,    0.0,    0.0, 0.0]] #,
# [0.0, -0.1,    0.0, 0.0,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.1, 0.0,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.0, -0.1,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.1,    0.0, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.0,    -0.1, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.0,    0.0, 0.05]]      

JOINT_IMP_LIST = [[0.1, 0.0,    0.0, 0.0,    0.0,    0.0, 0.0],
[0.0, -0.1,    0.0, 0.0,    0.0,    0.0, 0.0],
[0.0, 0.0,    0.1, 0.0,    0.0,    0.0, 0.0],
[0.0, 0.0,    0.0, -0.1,    0.0,    0.0, 0.0],
[0.0, 0.0,    0.0, 0.0,    0.1,    0.0, 0.0],
[0.0, 0.0,    0.0, 0.0,    0.0,    -0.1, 0.0],
[0.0, 0.0,    0.0, 0.0,    0.0,    0.0, 0.05]]           

JOINT_TORQUE_LIST = [[0.05, 0.0,    0.0, 0.0,    0.0,    0.0, 0.0],
[0.0, -0.1,    0.0, 0.0,    0.0,    0.0, 0.0],
[0.0, 0.0,    0.1, 0.0,    0.0,    0.0, 0.0],
[0.0, 0.0,    0.0, -0.1,    0.0,    0.0, 0.0],
[0.0, 0.0,    0.0, 0.0,    0.1,    0.0, 0.0],
[0.0, 0.0,    0.0, 0.0,    0.0,    -0.1, 0.0],
[0.0, 0.0,    0.0, 0.0,    0.0,    0.0, 0.05]]        

def get_action_for_controller(ctrl_type, step_num):
    """ Get appropriate action based on controller type
    """
    if ctrl_type == "EEImpedance": 
        action = EE_POSITIONS_LIST[step_num]
    else:
        raise ValueError("invalid control type string")
    return action

env = DemoControlEnv('dev/demo_control/demo_control_cfg.yaml', True, 'Demo Control Env')
print("Perls2 Demo Control Environment Created.")

control_types = {"1" : "EEImpedance", 
                 "2" : "JointVelocity",
                 "3" : "JointImpedance",
                 "4" : "JointTorque"}
command_dict = {"EEImpedance": EE_POSITIONS_LIST,
                  "JointVelocity": JOINT_VELOCITY_LIST,
                  "JointImpedance": JOINT_IMP_LIST,
                  "JointTorque": JOINT_TORQUE_LIST}

control_message = """Select Control Type: 
\t [1] : EE Impedance 
\t [2] : Joint Velocity
\t [3] : Joint Impedance
\t [4] : Joint Torque
>>"""

while True: 
    try:
        control_selected = input(control_message)
        if control_selected not in control_types.keys():
            raise ValueError
        break
    except ValueError:
        print(" Invalid input. please enter number from options provided")

print("Control type " + control_types[control_selected] + " selected.")

selected_control_name = control_types[control_selected]

## REMOVE ME: 
# selected_control_name = "JointImpedance"
env.robot_interface.change_controller(selected_control_name)

env.reset()
initial_ee_pose = env.robot_interface.dq
delta_list = []
# EE Impedance Test (delta)

for step, action in enumerate(command_dict[selected_control_name]):
    start = time.time()
    env.step(action)
    import matplotlib.pyplot as plt
    plt.show()
    # while (time.time() - start) < 5:
    #     pass

env.reset()
env.robot_interface.disconnect()

 
print("Demo complete.")