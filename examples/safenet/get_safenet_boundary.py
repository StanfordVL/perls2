"""Script to get Safenet bounds
"""

from safenet_env import SafenetEnv
from perls2.utils.yaml_config import YamlConfig
import numpy as np
import pybullet as pb
import threading 

class SafenetBoundaryEnv(SafenetEnv):
    def __init__(self,
             config='safenet_example.yaml',
             use_visualizer=True,
             name="SafenetBoundaryEnv", 
             world_type=None, 
             safenet_config=None):
        """Initialize the environment.

        Set up any variables that are necessary for the environment and your task.
        """
        self.config = YamlConfig(config)
        # Override world param in config file if arg passed. 
        if world_type is not None: 
            self.config['world']['type'] = world_type
        if safenet_config is not None:
            self.config['safenet'] = safenet_config['safenet'] 

        super().__init__(self.config, use_visualizer, name)
        if self.world.is_sim:
            self.visualize_boundaries()

        # Set to gravity compensation mode. 
        self.robot_interface.change_controller('JointTorque')
        self.MAX_STEPS = 10000
    
    def get_observation(self):
        """Get end-effector position
        """
        obs = {}
        obs['ee_position'] = self.robot_interface.ee_position

    def _exec_action(self):
        """Command gravity compensation to robot.
        """
        action = np.zeros(7)
        self.robot_interface.set_joint_torques(torques=action)

    def step(self, action, start=None):
        """Take a step.

        Args:
            action: The action to take.
        Returns:
            -observation: based on user-defined functions
            -reward: from user-defined reward function
            -done: whether the task was completed or max steps reached
            -info: info about the episode including success

        Takes a step forward similar to openAI.gym's implementation.
        """
        self._exec_action()
        self.world.step(start)
        self.num_steps = self.num_steps + 1

        termination = self._check_termination()

        # if terminated reset step count
        if termination:
            self.num_steps = 0

        reward = 0

        observation = self.get_observation()

        info = None
        return observation, reward, termination, info

if __name__ == '__main__':
    import argparse
    import threading
    import concurrent.futures
    import json
    import yaml

    parser = argparse.ArgumentParser(
        prog='get_safenet_boundary',
        description="Manually set Safenet Boundaries")
    parser.add_argument('output_fname', nargs='?', default='output/safenet.yaml', help='filepath to write safenet yaml config')
    args = parser.parse_args()

    print("#############################")
    print("SafeNet Boundary Script")
    
    env = SafenetBoundaryEnv(world_type='Real')
    print("Robot set to gravity compensation mode.")
    print("Move robot end-effector to max or min position indicated.")

    boundary_names = ["MIN X", "MAX X", "MIN Y", "MAX Y", "MIN Z", "MAX Z"]
    boundaries = {}
    for (index, boundary) in enumerate(boundary_names):
        env.step(np.zeros(7))
        print("Move end-effector to {} position".format(boundary_names[index]))
        collecting = True
        while collecting:
            input("Press [Enter] to save end-effector position for {}: ".format(boundary_names[index]))
            ee_position = env.robot_interface.ee_position.tolist()
            print("Boundary position is: {}".format(ee_position))
            response = input("Press [y] to confirm or [n] to resample.\n")
            if response in ['n', 'N']:
                pass
            elif response in ['y', 'Y']:
                print("position saved.")
                boundaries[boundary_names[index]] = ee_position
                collecting = False
            else: 
                print("Invalid selection. Choose from [y] or [n]")

    print("Boundary data collected: ")
    print(boundaries)

    yaml_fname = args.output_fname
    with open(yaml_fname, 'w') as f:
        safenet_yaml = {'safenet' : {}}
        #safenet_yaml = YamlConfig('safenet.yaml')
        safenet_yaml['safenet']['use_safenet'] = True
        safenet_yaml['safenet']['upper'] = [boundaries['MAX X'][0], boundaries['MAX Y'][1], boundaries['MAX Z'][2]]
        safenet_yaml['safenet']['lower'] = [boundaries['MIN X'][0], boundaries['MIN Y'][1], boundaries['MIN Z'][2]]

        yaml.dump(safenet_yaml, f, allow_unicode=True)

    pb_env = SafenetBoundaryEnv(world_type='Bullet', safenet_config=YamlConfig(yaml_fname))
    input("Boundaries visualized. Press Enter to exit.")




