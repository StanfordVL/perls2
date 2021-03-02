"""Script to get Safenet bounds
"""

from perls2.envs.env import Env
import numpy as np
import pybullet as pb
import threading 

class SafenetBoundaryEnv(Env):
    def __init__(self,
             cfg_path='safenet_example.yaml',
             use_visualizer=True,
             name="SafenetBoundaryEnv"):
        """Initialize the environment.

        Set up any variables that are necessary for the environment and your task.
        """
        super().__init__(cfg_path, use_visualizer, name)
        self.robot_interface.reset()
        self.init_ori = self.robot_interface.ee_orientation

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

    def step_env(env):
        """Step env in parallel thread.
        """
        env.step(np.zeros(7))

    parser = argparse.ArgumentParser(description="Manually set Safenet Boundaries")
    print("#############################")
    print("SafeNet Boundary Script")
    
    env = SafenetBoundaryEnv()
    print("Robot set to gravity compensation mode.")
    print("Move robot end-effector to max or min position indicated.")

    boundary_names = ["Minimum X", "Maximum X", "Minimum Y", "Maximum Y", "Minimum Z"]
    boundaries = {}
    for (bound_index, boundary) in enumerate(boundary_names):
        env.step(np.zeros(7))
        # Start a thread to step environment
        if env.world.is_sim:
            pb_thread = threading.Thread(target=step_env, args=env)
            pb_thread.start()

        print("Move end-effector to {} position".format(boundary_names[bound_index]))
        collecting = True
        while collecting:
            input("Press enter to save end-effector position: ")
            ee_position = env.robot_interface.ee_position
            print("Boundary position is: {}".format(ee_position))
            response = input("Press [y] to confirm or [n] to resample.")
            if response in ['n', 'N']:
                pass
            elif response in ['y', 'Y']:
                print("position saved.")
                boundaries[boundary_names[boundary_index]] = ee_position
                collecting = False
            else: 
                print("Invalid selection. ")

    pb_thread.join()
    print("Boundary data collected: ")
    print(boundaries)

    if not env.world.is_sim: 
        env.robot_interface.disconnect()



