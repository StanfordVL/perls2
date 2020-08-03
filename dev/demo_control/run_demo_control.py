from demo_control_env import DemoControlEnv
import numpy as np
import time 
import logging

logging.basicConfig(level=logging.INFO)
mpl_logger = logging.getLogger('matplotlib')
mpl_logger.setLevel(logging.WARNING) 

def make_ee_positions_list(steps):
  """ Randomly generate a series of ee positions to take

  Magnitude is always 0.1 in either xyz direction.
  """
  ee_list = []
  dim = 0
  for step in range(steps):
    for dim_i in range(6):
      delta = np.zeros(6)
      #dim = np.random.randint(0, 6)
      sign = np.random.choice([-1, 1])
      delta[dim_i] = sign*0.05
      ee_list.append(delta)
      # dim+=1
  return ee_list


EE_POSITIONS_LIST = make_ee_positions_list(steps=10)

JOINT_VELOCITY_LIST =[[0.1, 0.0,    0.0, 0.0,    0.0,    0.0, 0.0]] #,
# [0.0, -0.1,    0.0, 0.0,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.1, 0.0,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.0, -0.1,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.1,    0.0, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.0,    -0.1, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.0,    0.0, 0.05]]      

JOINT_IMP_LIST = [[0.05, 0.0,    0.0, 0.0,    0.0,    0.0, 0.0]]
# [0.0, -0.05,    0.0, 0.0,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.05, 0.0,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.0, -0.05,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.05,    0.0, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.0,    -0.05, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.0,    0.0, 0.05]]           

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

# Analyze results
def check_state(control_name, obs_list):
  global JOINT_IMP_LIST
  if control_name == "JointImpedance":
    print(obs_list)

    delta_list = [np.subtract(obs_list[j+1]['q'],obs_list[j]['q']) for j in range(len(obs_list) -1)]
    for j, (action, delta) in enumerate(zip(JOINT_IMP_LIST, delta_list)):
      print("error {}".format(np.subtract(action, delta)))

env = DemoControlEnv('dev/demo_control/demo_control_cfg.yaml', True, 'Demo Control Env')
print("Perls2 Demo Control Environment Created.")

control_types = {"1" : "EEImpedance", 
                 "2" : "JointVelocity",
                 "3" : "JointImpedance",
                 "4" : "JointTorque", 
                 "5" : "EEPosture"}

command_dict = {"EEImpedance": EE_POSITIONS_LIST,
                  "JointVelocity": JOINT_VELOCITY_LIST,
                  "JointImpedance": JOINT_IMP_LIST,
                  "JointTorque": JOINT_TORQUE_LIST,
                  "EEPosture": EE_POSITIONS_LIST}

control_message = """Select Control Type: 
\t [1] : EE Impedance 
\t [2] : Joint Velocity
\t [3] : Joint Impedance
\t [4] : Joint Torque
\t [5] : EE Posture
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

obs_list = []
## TODO ENABLE INITIAL RESET
# init_obs = env.reset()
# obs_list.append(init_obs)

delta_list = []

for step, action in enumerate(command_dict[selected_control_name]):
    print("in control loop")
    start = time.time()

    print(action)
    obs, _, _, _ = env.step(action)
    obs_list.append(obs)
    print("waiting")
    #import pdb; pdb.set_trace()


#env.reset()
env.robot_interface.disconnect()

check_state(selected_control_name, obs_list)
print("Demo complete.")


# import matplotlib
# matplotlib.use('TkAgg')
# import matplotlib.pyplot as plt

# step_data = np.load('dev/logs/control/step0.npz')['ee_list']
# step0_x = [pos[0] for pos in step_data]
# print(step0_x)
# plt.plot(step0_x)
# plt.show()

