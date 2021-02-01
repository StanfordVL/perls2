"""Script to run Joint Impedance Demo.
"""
from joint_demo import JointImpDemoSeq
import argparse

parser = argparse.ArgumentParser(
    description="Test controllers and measure errors.")
parser.add_argument('--world', default=None, help='World type for the demo, uses config file if not specified', choices=['Bullet', 'Real'])

parser.add_argument('--delta_val',
                    default=0.01, type=float,
                    help="Max step size (m or rad) to take for demo.")
parser.add_argument('--num_steps', default=50, type=int,
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
demo = JointImpDemoSeq(**kwargs)
demo.run()
