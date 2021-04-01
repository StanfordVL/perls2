"""Script to run Consecutive Line Demos in X, Y, Z direction with OpSpace controller.
"""

from osc_demo import OpSpaceLineXYZDemo

import argparse

parser = argparse.ArgumentParser(
    description="Run OpSpace Line Demo in XYZ directions")
parser.add_argument('--world', default=None, help='World type for the demo, uses config file if not specified', choices=['Bullet', 'Real'])
parser.add_argument('--ctrl_type',
                    default="EEImpedance",
                    help='Type of controller to test',
                    choices=["EEImpedance", "EEPosture"])
parser.add_argument('--test_fn',
                    default='set_ee_pose',
                    help='Function to test',
                    choices=['set_ee_pose', 'move_ee_delta'])

parser.add_argument('--path_length', type=float,
                    default=None, help='length in m of path')
parser.add_argument('--delta_val',
                    default=0.001, type=float,
                    help="Max step size [m] to take for demo.")
parser.add_argument('--axis',
                    default='x', type=str,
                    choices=['x', 'y', 'z'],
                    help='Axis about which end-effector rotates (from initial end-effector frame)')
parser.add_argument('--num_steps', default=100, type=int,
                    help="max steps for demo.")
parser.add_argument('--plot_pos', action="store_true",
                    help="whether to plot positions of demo.")
parser.add_argument('--plot_error', action="store_true",
                    help="whether to plot errors.")
parser.add_argument('--save', action="store_true",
                    help="whether to store data to file")
parser.add_argument('--demo_name', default=None,
                    type=str, help="Valid filename for demo.")
parser.add_argument('--save_fig', action="store_true",
                    help="whether to save pngs of plots")
parser.add_argument('--config_file', default='demos/demo_control_cfg.yaml', help='absolute filepath for config file.')

args = parser.parse_args()
kwargs = vars(args)

demo = OpSpaceLineXYZDemo(**kwargs)
demo.run()