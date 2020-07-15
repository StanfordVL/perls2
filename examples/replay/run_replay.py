"""BulletSawyerExample

Example showing how to save and restore states and execute recorded actions. 
This is useful for behavior cloning, and specific to pyBullet. 

This example builds off of the SimpleReachExample. To verify the replay, we compare
the ee pose, robot joint positions and object pose at each step. 

In this example, you can play around with the different ways of reseting a simulation, 
and it possible effects. The **Recommended** way is to use world.reboot() and env.initialize
to redo every part of simulation set up. 


"""
from __future__ import division

from examples.simple_reach.simple_reach_env import SimpleReachEnv

import numpy as np
import gym

import os
import sys
import logging
logging.basicConfig(level=logging.DEBUG)
import time

# This example involves pybullet specific funtionality
import pybullet as pb

from perls2.worlds.world import World
from perls2.arenas.bullet_arena import BulletArena
from perls2.robots.bullet_robot_interface import BulletRobotInterface
from perls2.sensors.bullet_camera_interface import BulletCameraInterface
from perls2.objects.bullet_object_interface import BulletObjectInterface

def get_action(observation):
    """Dummy policy to get action based on policy

    Given a delta xyz position from the end effector to the goal, return a
    vector in that direction with fixed magnitude

    Args:
        observation (3f): a vector of 3 floats corresponding to
        goal_position - current_ee_position.
    Returns:
        action (3f): vector of 3 floats corresponding to a delta end effector
            position.

  """
    # Get components from observations
    step = 1
    delta = observation
    action = step * delta/np.linalg.norm(delta)
    return delta
    #return action

class ReplayEnv(SimpleReachEnv):
    """ An extension of the simple reach environment that allows for saving and restoring the pybullet state.
    """
    def __init__(self,
                 cfg_path=None,
                 use_visualizer=False,
                 name=None):
        """Initialize.

        Parameters
        ----------
        config: dict
            A dict with config parameters
        arena:
            container for setting up the world in which robot and objects
            interact
        use_visualizer:
            Whether or not to use visualizer. NOTE: Pybullet only allows for one
            simulation to be connected to GUI. It is up to the user to manage
            this
        """
        super().__init__(cfg_path, use_visualizer, name)

        self.goal_position = [0, 0, 0]

        # for sim we are tracking an object, increase goal position to be above
        # the actual position of the object.
        self.object_interface = self.world.object_interfaces['013_apple']

        if (self.world.is_sim):
            self.update_goal_position()

        self._initial_ee_orn = []

    def reset(self):
        """Reset the environment.

        This reset function is different from the parent Env function.
        The object placement and camera intrinsics/extrinsics are
        are randomized if we are in simulation.

        Returns:
            The observation.
        """
        # Reset simulation to clear everything out, and reinitialize
        # self.world.reconnect()
        # self.world.reboot()

        # self.initialize()
        self.goal_position = [0, 0, 0]

        # for sim we are tracking an object, increase goal position to be above
        # the actual position of the object.
        self.object_interface = self.world.object_interfaces['013_apple']

        self._initial_ee_orn = []

        logging.info(
            "Environment reset - physicsClient: " + str(self._physics_id))
        self.episode_num += 1
        self.num_steps = 0
        self.world.reset()
        self.robot_interface.reset()
        self._initial_ee_orn = self.robot_interface.ee_orientation

        if (self.world.is_sim):
            if self.config['object']['random']['randomize']:
                self.object_interface.place(self.arena.randomize_obj_pos())
            else:
                self.object_interface.place(
                    self.config['object']['object_dict']['object_0']['default_position'])

            self.sensor_interface.set_view_matrix(self.arena.view_matrix)
            self.sensor_interface.set_projection_matrix(
                self.arena.projection_matrix)
            self.world._wait_until_stable()
        else:
            self.goal_position = self.arena.goal_position

        observation = self.get_observation()

        return observation


    def save_state(self):
        """ Save the environment state to the .bullet file.
            Args : None
            Returns: state_id (int) in memory identifier of state in pybullet.
        """

        state_id = pb.saveState(self.world.physics_id)
        return state_id

    def save_state_bullet(self,fileName):
        """ Save the environment state to the .bullet file.
            Args : None
            Returns: state_id (int) in memory identifier of state in pybullet.
        """

        pb.saveBullet(bulletFileName=fileName, physicsClientId=self.world.physics_id)
        return fileName

    def restore_state(self, state_id): 
        """ Restore state from in-memory state-id. 

         Args: state_id (int): in-memory identifier of state in pybullet. 
        """
        pb.restoreState(stateId=state_id, physicsClientId=self.world.physics_id)

    def restore_state_bullet(self, bullet_state):
        """ Restore bullet state from filepath. 
            
            Args: relative filepath where bullet state is stored. 

        """
        pb.restoreState(fileName=bullet_state, physicsClientId=self.world.physics_id)

    def get_observation(self): 
        """
        Return observation for the current state. 
        """

        if(self.world.is_sim):
            self.update_goal_position()
        ee_pose = np.array(self.robot_interface.ee_pose).flatten()
        q = np.array(self.robot_interface.q).flatten()
        obj_pose = np.array(self.object_interface.pose).flatten()

        delta = np.subtract(self.goal_position,ee_pose[0:3]).flatten()        
        return np.hstack((delta, ee_pose, q, obj_pose))


env = ReplayEnv('examples/replay/replay.yaml', True, None)
bullet_save_dir = 'examples/replay/bullet_states/'
# Lists for saving demonstrations
num_episodes = 5
action_list = []

states = []
demo_obs = []
demos_data = []

# Collect a demonstration.

# Wait for real robots to show episode is complete
if not env.world.is_sim:
    time.sleep(3)

step = 0



for ep in range(num_episodes):

    demo_states = []
    demo_obs = []
    demo_actions = []
    # Reset the environment
    observation = env.reset()    

    done = False
    while not done:

        # save the initial state.
        demo_obs.append(observation)
        
        bullet_file = os.path.join(bullet_save_dir, "ep{}_state_{}.bullet".format(ep, step))
        demo_states.append(env.save_state_bullet(bullet_file))

        #import pdb; pdb.set_trace()
        # Get action and step environment forward.
        action = get_action(observation[0:3])
        start = time.time()
        observation, reward, termination, info = env.step(action)
        
        demo_actions.append(action)

        # enforce policy frequency by waiting
        while ((time.time() - start) < 0.05):
            pass
        
        step += 1
        done = termination

    
    ep_data = {'states': demo_states, 
                 'obs': demo_obs,
                 'actions': demo_actions}
    
    demos_data.append(ep_data)
    print("Collected demo {}".format(ep))

# Store replay observations to compare to demonstration.

replay_env = ReplayEnv('examples/replay/replay.yaml', False, None)

replay_data  = []
for ep_num, ep_data in enumerate(demos_data):
    replay_obs = []

    print("Replaying demo {}".format(ep_num))
    # Test out different reset options here: 
    replay_env.reset()
    replay_env.restore_state_bullet(ep_data['states'][0])
    replay_obs.append(replay_env.get_observation())

    for j,action in enumerate(ep_data['actions']):
        # print("Demo   {}  step {} state:\n{} ".format(ep_num, j, ep_data['obs'][j]))
        # print("Replay {}  step {} state:\n{} ".format(ep_num, j, replay_obs[j]))

        if not np.all(np.equal(ep_data['obs'][j], replay_obs[j])):
            print("Step {} not equal".format(j))
            print("Delta : {}".format(np.subtract(ep_data['obs'][j], replay_obs[j])))
        else: 
            print("Step {} exactly equal.".format(j))
        
        start = time.time()
        observation, reward, termination, info = replay_env.step(action)
        replay_obs.append(observation)


        # enforce policy frequency by waiting
        while ((time.time() - start) < 0.05):
            pass
            step += 1
        done = termination

    replay_data.append({'obs': replay_obs})


print("num replay_obs {}".format(len(replay_obs)))
print("num demo obs {}". format(len(demo_obs)))
print("test complete.")