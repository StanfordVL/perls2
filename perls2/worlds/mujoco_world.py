""" Parent class for Mujoco Pybullet Environments
"""

from perls2.worlds.world import World


class MujocoWorld(World):
    """Template class for Mujoco environments.
    Set up a given environment, as well as tasks in those environments.
    Typically, an env will have one robot with one camera as well as
    logic to have the robot execute a particular task.
    """

    def __init__(self,
                 config=None,
                 use_visualizer=False):
        pass
