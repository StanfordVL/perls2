""" Test script for controllers.

Creates a demo based on terminal args for various controllers. Helpful for debugging
controllers and tuning gains. Capable of reading custom config files for projects.

Args:
    --ctrl_type (str): string identifying type of controller to test. Must be
        valid control type name.
    --demo_type (str): string identifying type of demo to run with controller.
        Must be valid with control type.
        The following demos are available for each controller:
        EEImpedance, EEPosture:
            - Zero, Square, Line, Rotation
        JointImpedance, JointVelocity, JointTorque:
            -Zero, Sequential
    --test_fn (str): string identifying the robot interface function to execute
        the action. Must be valid with control type.
        The following functions are available for each controller:
        EEImpedance, EEPosture:
            - 'set_ee_pose', 'move_ee_delta'
        JointImpedance:
            - 'set_joint_positions', 'set_joint_delta'
        JointVelocity
            - 'set_joint_velocities
        JointTorque:
            - 'set_joint_torques'
    --fix_pos (flag): only for move_ee_delta with Rotation demos. Fixes position.
    --fix_ori (flag): only for move_ee_delta with Line, Square demos. Fixes orientation.
    --delta_val (float): (optional) space to separate each action in the demo.
        For "Line", "Square" demos, this is the space each point in the demo is
        spaced from the start.
    --path_length (float): Applicable only for "Line" and "Square" demos,
        length of path to travel (m) for the Line, side_length (m) for square.
        Overrides delta_val
    --axis (str): 'x', 'y', 'z' axis about which the demo progresses the action.
        Applicable only for "Line" or "Rotation" demos.
    --num_steps (int): total number of steps in the demo.
    --plot_pos (flag): Use to direct demo to plot current and goal states at
        each step.
    --plot_error (flag): Use to direct demo to plot error at each step.
    --save (flag): save demo data to npz
    --save_fig (flag): save position and error plots to png files.
    --demo_name (str): file name to save demo data.


"""

import logging
import argparse

from control_demo import Demo

logging.basicConfig(level=logging.INFO)
mpl_logger = logging.getLogger('matplotlib')
mpl_logger.setLevel(logging.WARNING)

CONTROL_TYPES = {
    "1": "EEImpedance",
    "2": "JointVelocity",
    "3": "JointImpedance",
    "4": "JointTorque",
    "5": "EEPosture"}

DEMO_TYPES = {
    "0": "Zero",
    "1": "Sequential",
    "2": "Square",
    "3": "Line",
    "4": "Rotation"}

USE_ABS = {
    "0": True,
    "1": False}
TEST_FUNCS = {
    "0": 'set_ee_pose',
    "1": 'move_ee_delta'}


def prompt_menu_selection(menu_type, options):
    """ Prompt user to select option from menu.
    Args:
        menu_type (str) : type of menu to indicate what user is selecting
        options (dict): dictionary of valid options to select from.

    Returns: str indicating option selected by user, value of key from options
        dict.

    Example:
        pets_dict = {"1" : "Cat", "2": "Dog", "3": Parrot}
        pet_type = prompt_menu_selection("Pet", pets_dict)

    """
    def menu_msg(menu_type, options):
        menu_title = "Select {} Type".format(menu_type)
        menu_options = ""
        for key in options.keys():
            menu_options += ("\n\t [{}] : {}".format(key, options[key]))

        menu_options += "\n >>"

        return menu_title + menu_options

    while True:
        try:
            option_selected = input(menu_msg(menu_type, options))
            if option_selected not in options.keys():
                raise ValueError
            break
        except ValueError:
            print(" Invalid input. please enter number from options provided")

    print("{} type ".format(menu_type) + options[option_selected] +
          " selected.")
    return options[option_selected]


class ControllerTester():
    """Class that manages the demo / test.

    """
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        # Prompt the user if any of the important set up arguments are not
        # filled in.
        if self.kwargs['ctrl_type'] is None:
            self.kwargs['ctrl_type'] = prompt_menu_selection(
                "Control", CONTROL_TYPES)
        if self.kwargs['demo_type'] is None:
            self.kwargs['demo_type'] = prompt_menu_selection(
                "Demo", DEMO_TYPES)
        if self.kwargs['use_abs'] is None:
            self.kwargs['use_abs'] = prompt_menu_selection("Use Abs", USE_ABS)
        if self.kwargs['test_fn'] is None:
            self.kwargs['test_fn'] = prompt_menu_selection(
                "Function to test", TEST_FUNCS)
        self.demo = Demo.make_demo(**self.kwargs)  # noqa: F405

    def run(self):
        # Create demo based on control type and demo type.

        self.demo.run()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Test controllers and measure errors.")
    parser.add_argument('--ctrl_type',
                        default=None,
                        help='Type of controller to test')
    parser.add_argument('--use_abs',
                        action="store_true",
                        help='Use absolute positions')
    parser.add_argument('--demo_type',
                        default=None,
                        help='Type of menu to run.')
    parser.add_argument('--test_fn',
                        default='set_ee_pose',
                        help='Function to test',
                        choices=['set_ee_pose', 'move_ee_delta', 'set_joint_delta', 'set_joint_positions', 'set_joint_torques', 'set_joint_velocities'])
    parser.add_argument('--path_length', type=float,
                        default=None, help='length in m of path')
    parser.add_argument('--delta_val',
                        default=None, type=float,
                        help="Max step size (m or rad) to take for demo.")
    parser.add_argument('--axis',
                        default='x', type=str,
                        choices=['x', 'y', 'z'],
                        help='axis for demo. Position direction for Line or rotation axis for Rotation')
    parser.add_argument('--joint_num',
                        default=6, type=int,
                        help='joint index to test for Joint space demos')
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
    parser.add_argument('--fix_ori', action="store_true",
                        help="fix orientation for move_ee_delta")
    parser.add_argument('--fix_pos', action="store_true",
                        help="fix position for move_ee_delta")
    parser.add_argument('--config_file', default='dev/validation/demo_control_cfg.yaml', help='absolute filepath for config file.')
    args = parser.parse_args()
    kwargs = vars(args)
    tester = ControllerTester(**kwargs)
    tester.run()
