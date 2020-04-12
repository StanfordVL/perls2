from demo_control_env import DemoControlEnv
import numpy as np

import logging

EE_POSITIONS_LIST = [[0., 0, 0.05,], [0., 0.05, 0,], [0.05, 0, 0.0]]


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
                 "2" : "JointVelocity"}
control_message = """Select Control Type: 
\t [1] : EE Impedance 
\t [2] : Joint Velocity
\t [3] : Joint Impedance
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

env.robot_interface.change_controller(selected_control_name)

env.reset()

# EE Impedance Test (delta)
if selected_control_name == "EEImpedance": 
    for step, action in enumerate(EE_POSITIONS_LIST):
        initial_ee_pose = env.robot_interface.ee_pose
        action = action + [0, 0, 0]
        env.step(action)
        delta = np.asarray(env.robot_interface.ee_pose) -  np.asarray(initial_ee_pose)
        print("desired_delta: " + str(action))
        print("final delta: " + str(delta)) 
        input("press enter to step to next action.")   
        