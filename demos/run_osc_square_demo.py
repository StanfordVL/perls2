"""Script to run Square with OpSpace controller.
"""

from osc_demo import OpSpaceSquareDemo
import argparse

parser = argparse.ArgumentParser(
    description="Run OpSpace Square Demo ")
parser.add_argument('--world', default=None, 
     help='World type for the demo, uses config file if not specified', choices=['Bullet', 'Real'])

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
                    help="Max step size (m) between consecutive steps in demo.")
parser.add_argument('--num_steps', default=400, type=int,
                    help="number of steps for demo.")
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
parser.add_argument('--config_file', default='demos/demo_control_cfg.yaml', help='filepath for config file relative to current directory')
parser.add_argument('--cycles', type=int, default=1, help="num times to cycle path (only for square)")

args = parser.parse_args()
kwargs = vars(args)

kwargs["demo_type"] = "Square"

demo = OpSpaceSquareDemo(**kwargs)
demo.run()