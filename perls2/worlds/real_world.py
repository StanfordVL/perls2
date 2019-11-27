""" Parent class for Mujoco Pybullet Environments
"""

import abc
import pybullet

from perls2.worlds.world import World
from perls2.arenas.real_arena import RealArena
from perls2.robots.real_robot_interface import RealRobotInterface
from perls2.sensors.kinect_camera_interface import KinectCameraInterface

class RealWorld(World):
    """
    ypically, an env will have one robot with one camera as well as
    logic to have the robot execute a particular task.
    """

    def __init__(self,
                 config=None, 
                 use_visualizer=False,
                 name='DefaultEnv'):
        """ Initialize. 


        Parameters
        ----------
        config: dict
            A dict with config parameters. For initialization the relevant 
            parameters that should be defined are: 
             - robot:type
             - sensor:type
        """
        self.config = config

        self.name = name
        self.use_visualizer = use_visualizer
        # Connect to pybullet to compute kinematics for robots
        if self.use_visualizer:
            self._physics_id = pybullet.connect(pybullet.GUI)
        else:
            self._physics_id = pybullet.connect(pybullet.DIRECT)

        # Pybullet sim parmeters
        pybullet.setGravity(0,0,-10, physicsClientId=self._physics_id)

        # Learning parameters
        self.episode_num = 0

        # TODO: ctl_steps_per_action - this has more to do with OSC
        # Create an arena to load robot and objects
        self.arena = RealArena(self.config, self._physics_id)
        self.robot_interface = RealRobotInterface.create(
                                                 config=self.config, 
                                                 physics_id=self._physics_id, 
                                                 arm_id=self.arena.arm_id)
        self.sensor_interface = KinectCameraInterface()

        self.is_sim = False

    def reset(self):
        """Reset the environment.

        Returns:
            The observation.
        """
        # reload robot to restore body after any collisions
        pass

        

    def step(self):
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
        # TODO: What is the order of this

        # Prepare for next step by executing action 
        pass
        # self._exec_action(action)
        # self.num_steps = self.num_steps+1
        
        # # Check if max num steps reached or goal completed
        # termination = self._check_termination()
        # # if terminated reset step count 
        # if termination:
        #     self.num_steps = 0    
        
        # #print("Goal position " + str(goal_position))

        # # Collect reward, observation and info
        # reward = self.rewardFunction()
        # observation = self.get_observation()
        # info = self.info()

        # return observation, reward, termination, info


    def visualize(self, observation, action):
        """Visualize the action - that is,
        add visual markers to the world (in case of sim)
        or execute some movements (in case of real) to
        indicate the action about to be performed.

        Args:
            observation: The observation of the current step.
            action: The selected action.
        """
        pass

    def handle_exception(self, e):
        """Handle an exception.
        """
        pass

    @property
    def info(self):
        return {
                }
