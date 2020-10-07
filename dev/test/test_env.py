"""Test Environment for development and functionality.
"""
from perls2.envs.env import Env


class TestEnv(Env):
    """Env class for testing perls2 functionality.
    """
    def __init__(self,
                 config='test_cfg.yaml',
                 use_visualizer=True,
                 name='TesterEnv'):
        """Initialize environment.
        """
        super().__init__(config, use_visualizer, name)

    def get_observation(self):
        """Return observations as dict.
        """
        obs = {}
        obs['frames'] = self.sensor_interface.frames()

        proprio = {}
        proprio['ee_pose'] = self.robot_interface.ee_pose
        proprio['ee_position'] = self.robot_interface.ee_position
        proprio['ee_orientation'] = self.robot_interface.ee_orientation
        proprio['q'] = self.robot_interface.q
        proprio['dq'] = self.robot_interface.dq
        proprio['ee_v'] = self.robot_interface.ee_v
        proprio['ee_w'] = self.robot_interface.ee_w

        obs['proprio'] = proprio

        return obs
