import numpy as np
import time 
import logging
from RobotTeleop.robots.bullet_sawyer_robot import DummyEnv
import imageio

logging.basicConfig(level=logging.INFO)
mpl_logger = logging.getLogger('matplotlib')
mpl_logger.setLevel(logging.WARNING) 

action = [0.2, 0, 0, 0, 0, 0, 0]
env = DummyEnv('dev/posture_debug/posture_cfg.yaml',False, "PostureEnv")
steps_to_wait = 1000
video_writer = imageio.get_writer("dev/posture_debug/kp100_debug_nq_control.mp4", fps=20)
video_skip = 1
video_count = 0

Nq_list = []
gq_list = []

obs = env.get_observation()
while obs['ee_pose'][0] < 0.7:

  env.step(action)
  obs = env.get_observation()
  video_img = env.render()
  video_writer.append_data(video_img)
 
initial_q = obs['q']

for step in range(steps_to_wait):
  env.step([0]*7) 
  
  Nq_list.append(env.robot_interface.N_q)
  gq_list.append(env.robot_interface.gravity_vector)
  #print("dq {}".format(env.robot_interface.dq))
  video_img = env.render()
  video_writer.append_data(video_img)

# PLOTS
import pdb; pdb.set_trace()
# import matplotlib.pyplot as plt
# Nq_joint_list = list(map(list, zip(*Nq_list)))
# gq_joint_list = list(map(list, zip(*gq_list)))

# num_steps = steps_to_wait
# plt.figure(1)
# plt.subplot(211)
# plt.plot(range(steps_to_wait, Nq_joint_list[0]))
# plt.xlabel("Num time steps")
# plt.ylabel("Torque")
#print(np.equal(np.array(env.robot_interface.q)), np.array(env.config['sawyer']['neutral_joint_angles']))

# obs_list = []
# ## TODO ENABLE INITIAL RESET
# # init_obs = env.reset()
# # obs_list.append(init_obs)

# delta_list = []

# for step, action in enumerate(command_dict[selected_control_name]):
#     print("in control loop")
#     start = time.time()

#     print(action)
#     obs, _, _, _ = env.step(action)
#     obs_list.append(obs)
#     print("waiting")
#     #import pdb; pdb.set_trace()


# #env.reset()
# env.robot_interface.disconnect()

# check_state(selected_control_name, obs_list)
# print("Demo complete.")


# import matplotlib
# matplotlib.use('TkAgg')
# import matplotlib.pyplot as plt

# step_data = np.load('dev/logs/control/step0.npz')['ee_list']
# step0_x = [pos[0] for pos in step_data]
# print(step0_x)
# plt.plot(step0_x)
# plt.show()

