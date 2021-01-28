"""This example shows different ways to load objects into the environment.
"""

from perls2.envs.env import Env


class LoadObjectEnv(Env):
    def __init__(self,
                 cfg_path='examples/load_objects/load_objects.yaml',
                 use_visualizer=True,
                 name=None):
        """Initialize.
        """
        super().__init__(cfg_path, use_visualizer, name)
        print("Objects in world: {}".format(self.world.object_interfaces.keys()))
        self.load_objects()
        print("Objects in world {}".format(self.world.object_interfaces.keys()))
        self.remove_objects()

    def load_objects(self):
        """ Load objects
        """
        self.world.add_object(
            path='objects/ycb/013_apple/google_16k/textured.urdf',
            name='apple2',
            pose=[3.0, 3.0, 3.0, 0.0, 0.0, 0.0, 1.0],
            scale=1.0,
            is_static=False)

    def remove_objects(self):
        self.world.remove_object('apple2')


if __name__ == '__main__':
    env = LoadObjectEnv()
