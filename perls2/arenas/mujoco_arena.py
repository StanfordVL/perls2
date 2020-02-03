"""The parent class for Arenas encapsulating robots, sensors and objects.
"""

from perls2.arenas.arena import Arena


class MujocoArena(Arena):
    """The class definition for arenas
    Arenas contain interfaces for robots, sensors and objects.
    """
    def __init__(self):
        raise NotImplementedError
