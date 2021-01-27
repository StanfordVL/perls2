""" Factory for making different types of worlds"""


def make_world(config, use_visualizer, name):
    if config['world']['type'] == 'Bullet':
        from perls2.worlds.bullet_world import BulletWorld
        world = BulletWorld(config, use_visualizer, name)
    elif config['world']['type'] == 'Real':
        from perls2.worlds.real_world import RealWorld
        world = RealWorld(config, use_visualizer, name)
    elif config['world']['type'] == 'Mujoco':
        from perls2.worlds.mujoco_world import MujocoWorld
        world = MujocoWorld(config, use_visualizer, name)
    else:
        raise ValueError("world type must be either Bullet, Real or Mujoco")

    return world
