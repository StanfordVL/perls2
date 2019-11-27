""" Factory for making different types of worlds"""

from perls2.worlds.bullet_world import BulletWorld 
from perls2.worlds.mujoco_world import MujocoWorld
from perls2.worlds.real_world import RealWorld

def make_world(config, use_visualizer, name):
    if config['world']['type'] == 'Bullet':
        world = BulletWorld(config, use_visualizer, name)
    elif config['world']['type'] == 'Real':
        world = RealWorld(config, use_visualizer, name)
    elif config['world']['type'] == 'Mujoco':
        world = MujocoWorld(config, use_visualizer, name)
    else: 
        raise ValueError("world type must be either Bullet, Real or Mujoco")        

    return world