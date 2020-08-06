from demo_control_env import DemoControlEnv
import numpy as np
import time 
import logging
from datetime import datetime
import argparse
import matplotlib.pyplot as plt
logging.basicConfig(level=logging.INFO)
mpl_logger = logging.getLogger('matplotlib')
mpl_logger.setLevel(logging.WARNING) 

CONTROL_TYPES = {"1" : "EEImpedance", 
             "2" : "JointVelocity",
             "3" : "JointImpedance",
             "4" : "JointTorque", 
             "5" : "EEPosture"}

DEMO_TYPES = {"1" : "Sequential", 
             "2" : "Square",
             "3" : "Circle"}

# command_dict = {"EEImpedance": EE_POSITIONS_LIST,
#                   "JointVelocity": JOINT_VELOCITY_LIST,
#                   "JointImpedance": JOINT_IMP_LIST,
#                   "JointTorque": JOINT_TORQUE_LIST,
#                   "EEPosture": EE_POSITIONS_LIST}

# Action space dimensions
JOINT_IMP_DIM = 7
JOINT_VEL_DIM = 7

OSC_DIM = 6

def prompt_menu_selection(menu_type, options):
    """ Prompt user to select option from menu. 
    Args: 
        menu_type (str) : type of menu to indicate what user is selecting
        options (dict): dictionary of valid options to select from. 

    Returns: str indicating option selected by user, value of key from options dict.

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

    print("{} type ".format(menu_type) + options[option_selected] + " selected.")
    return options[option_selected]

class ControllerTester():
    """Class that manages the demo / test.

    """
    def __init__(self, ctrl_type, demo_type, **kwargs):
        self.ctrl_type = ctrl_type
        self.demo_type = demo_type
        self.demo = None
        self.use_abs = False


    def run(self): 
        if self.ctrl_type is None:
            self.ctrl_type = prompt_menu_selection("Control", CONTROL_TYPES)
        if self.demo_type is None:
            self.demo_type = prompt_menu_selection("Demo", DEMO_TYPES)
        
        # Create demo based on control type and demo type. 
        self.demo = OpSpaceDeltaDemo(self.ctrl_type, self.demo_type, False)

        initial_state = self.demo.env.reset()
        print(self.demo.env.robot_interface.ee_pose)
        self.demo.run()
        #self.demo.plot_error()
        #self.demo.plotxy()
        self.demo.save_data()

class Demo():
    """Class definition for demonstration. 
        Demonstrations are a series of actions that follow a specified pattern and are 
        of appropriate dimension for the controller. 
    """
    def __init__(self, ctrl_type, demo_type, use_abs):
        self.env = DemoControlEnv('dev/demo_control/demo_control_cfg.yaml', True, 'Demo Control Env')
        self.ctrl_type = ctrl_type
        self.env.robot_interface.change_controller(self.ctrl_type)
        self.demo_type = demo_type
        self.errors = []
        self.actions = []
        self.states = []

    def get_action_list(self):
        raise NotImplementedError

    def run(self):
        for action in self.action_list:

            self.goal_state = self.get_goal_state(action)
            self.goal_states.append(self.goal_state)

            self.env.step(action)          
            self.actions.append(action)
            new_state = self.get_state()

            self.states.append(new_state)
            self.errors.append(self.compute_error(new_state))
            print(self.errors[-1])

    def save_data(self, name=None):
        if name is None:
            name = "dev/demo_control/data_npz/{}_{}_{}.npz".format(str(time.time()), self.ctrl_type, self.demo_type)

        np.savez(name, states=self.states, errors=self.errors, actions=self.actions, goals=self.goal_states, allow_pickle=True)
        data = np.load(name)

    def get_goal_state(self, delta):
        current_state = self.get_state()
        goal_state = np.add(current_state, delta)
        return goal_state    

    def get_errors_per_dim(self):
        """Reshapes error list at a step to error list at a joint or dimension for all steps.
        """
        num_dims = len(self.errors[0])
        all_dim_errors = []
        for dim_i in range(num_dims):
            error_dim_list = [step_error[dim_i] for step_error in self.errors]

            all_dim_errors.append(error_dim_list)

        return all_dim_errors

    def plot_error(self):
        import matplotlib.pyplot as plt
        errors = self.get_errors_per_dim()
        plt.xlabel("Num steps")
        plt.ylabel("error joint 0 ")
        plt.plot(errors[0])
        plt.show()

class OpSpaceDemo(Demo):
    def __init__(self, ctrl_type, demo_type, use_abs):
        self.action_space_dim = 6
        self.delta_val = 0.01
        super().__init__(ctrl_type, demo_type, use_abs)

    def get_state(self): 
        return self.env.robot_interface.ee_pose_euler

    def get_action_list(self):
        raise NotImplementedError

class OpSpaceDeltaDemo(OpSpaceDemo):
    def __init__(self, ctrl_type, demo_type, use_abs):
        super().__init__(ctrl_type, demo_type, use_abs)
        self.init_state = self.get_state()

        if self.demo_type == "Square":
            self.path = Square([0., 0., 0.], 2)
            # Get goal states based on demo and control type. 
            self.action_list = self.get_action_list()
            self.goal_states = self.get_goal_states()
        else:
            raise ValueError("invalid demo type")

        self.num_steps = len(self.action_list)
        self.step_num = 0

    def run(self):
        import pdb; pdb.set_trace()
        for i, action in enumerate(self.action_list):

            self.env.step(action)          
            self.actions.append(action)
            new_state = self.get_state()

            self.states.append(new_state)
            self.errors.append(self.compute_error(self.goal_states[i], new_state))
            print(self.errors[-1])
            self.plotxy()

    def get_goal_states(self):
        goal_states = [self.init_state]
        for action in self.action_list:
            goal_states.append(np.add(goal_states[-1], action*self.env.config['controller']['Bullet']['EEPosture']['output_max']))
        return goal_states

    def compute_error(self, goal, current):
        return np.subtract(goal, current)
        
    def get_action_list(self):
        """ Get action list for random demo.
        
            Each action will move only one joint in a random direction 
            at a time. the magnitude of each action is determined by 
            the delta_val. 

            e.g. 
            [0, -0.05, 0, 0, 0, 0]

        """
        if self.demo_type == "Sequential":
            # Joint position delta demo. 
            ee_pose_delta_demo = []

            # Move each joint individually by delta (m for xyz, rad for angles)
            for dim_num in range(self.action_space_dim):
                delta = np.zeros(self.action_space_dim)
                delta[dim_num] = 0.01
                ee_pose_delta_demo.append(delta)

            for dim_num in range(self.action_space_dim):
                delta = np.zeros(self.action_space_dim)
                delta[dim_num] = -0.01
                ee_pose_delta_demo.append(delta)

            return ee_pose_delta_demo
        elif (self.demo_type == "Square"):
            sq_delta_actions = [np.hstack((ee_pos,[0, 0, 0])) for ee_pos in self.path.path]
            return sq_delta_actions

    def plot_error(self):
        """ Plot the error between the goal and state for each dimension.
        """
        errors = self.get_errors_per_dim()
        plt.xlabel("Num steps")
        plt.ylabel("error x ")
        plt.plot(errors[0])
        plt.show()

    def plotxy(self):
        """Plot xy positions of end effector with goal ee positions xy plane only. 
        """
        # Get list of goal x and y positions
        goal_x = [goal[0] for goal in self.goal_states]
        goal_y = [goal[1] for goal in self.goal_states]

        # get list of state x and y positions
        state_x = [state[0] for state in self.states]
        state_y = [state[1] for state in self.states]

        plt.plot(goal_x, goal_y, '*r')
        plt.plot(state_x, state_y, '*b')
        print(self.goal_states[len(self.states)])
        print(self.states[-1])

        plt.show()

class JointSpaceDemo(Demo):

    """Class definition for joint space demonstrations.

    Joint space demonstrations are 7 dof for each motor. 
    They include joint positions, velocities and torques.

    Attributes:    
        delta_val (float): magnitude of delta from current position. 
            Absolute joint space demos are also determined by adding
            this delta to current joint position. 
    """
    def __init__(self, ctrl_type, demo_type, use_abs):
        self.action_space_dim = 7
        self.delta_val = 0.01
        super().__init__(ctrl_type, demo_type, use_abs)
        # Get goal states based on demo and control type. 
        self.goal_states = []
        self.action_list = self.get_action_list()
        self.num_steps = len(self.action_list)
        self.step_num = 0

    def get_state(self): 
        return self.env.robot_interface.q[:7]

class JointSpaceDeltaDemo(JointSpaceDemo):

    def get_action_list(self):
        """ Get action list for random demo.
        
            Each action will move only one joint in a random direction 
            at a time. the magnitude of each action is determined by 
            the delta_val. 

            e.g. 
            [0, -0.05, 0, 0, 0, 0]

        """
        action_list = []

        # Joint position delta demo. 
        joint_position_delta_demo = []
        NUM_JOINTS = 7

        # Move each joint individually by +0.05 radians
        for joint_num in range(NUM_JOINTS):
            delta = np.zeros(NUM_JOINTS)
            delta[joint_num] = 0.01
            joint_position_delta_demo.append(delta)

        # Move each joint individually by -0.05 radians
        for joint_num in range(NUM_JOINTS):
            delta = np.zeros(NUM_JOINTS)
            delta[-1 - joint_num] = -0.05
            joint_position_delta_demo.append(delta)

        return joint_position_delta_demo

class JointSpaceAbsDemo(JointSpaceDeltaDemo):
    def __init__(self, ctrl_type, demo_type, use_abs, start_pos): 
        self.start_pos = start_pos
        super().__init__(ctrl_type, demo_type, use_abs)

    def get_action_list(self):
        """Add actions to joint position and update to get
        absolute positions at each step.
        """

        jp_delta_demo = super().get_action_list()
        jp_abs_demo = []

        for action in jp_abs_demo: 
            jp_abs_demo.append(self.states, jp_delta_demo)
            self.start_pos = jp_abs_demo[-1]
        return jp_abs_demo


class Path():
    """Class definition for path definition (specific to ee trajectories)
    
    A Path is a series
 
    """

    def __init__(self, shape, num_points): 
        self.shape = shape
        self.num_points = num_points
        self.path = []

SQUARE_DELTA = [[0.1, 0, 0], 
                [0.1, 0., 0,],
                [0.1, 0, 0], 
                [0., 0.1, 0],
                [0, 0.1, 0], 
                [0, 0.1, 0], 
                [-0.1, 0.0, 0], 
                [-0.1, 0.0, 0], 
                [-0.1, 0.0, 0]] 

class Square(Path):
    """Class def for square path. 

    Square path defined by side length and start point. 
    At step 4 * sidelength -1, ee is not at initial point. 
    Last step returns to initial point. 

    Square path is ordered in clockwise from origin (Bottom, Left, Top, Right)
    """ 
    def __init__(self, start_pos, side_num_pts):
        self.start_pos = start_pos
        self.side_num_pts = side_num_pts
        self.deltaxy = 1
        self.path = [[0, 0, 0]]
        self.make_path()

    def make_path(self): 
        # Bottom Side
        for pt in range(self.side_num_pts):
           self.path.append([self.deltaxy, 0.0, 0.0])
        # Left Side
        for pt in range(self.side_num_pts):
             self.path.append([0.0, self.deltaxy, 0.0])
        # Top side
        for pt in range(self.side_num_pts):
             self.path.append([-self.deltaxy, 0, 0.0])
        # Top side
        for pt in range(self.side_num_pts):
            self.path.append([0.0, -self.deltaxy, 0.0])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Test controllers and measure errors.")
    parser.add_argument('--ctrl_type', default=None, help='Type of controller to test')
    parser.add_argument('--use_abs', action="store_true", help='Use absolute positions')
    parser.add_argument('--demo_type', default=None, help='Type of menu to run.')
    args = parser.parse_args()
    kwargs = vars(args)
    tester = ControllerTester(**kwargs)
    tester.run()


# def make_ee_positions_list(steps):
#   """ Randomly generate a series of ee positions to take

#   Magnitude is always 0.1 in either xyz direction.
#   """
#     ee_list = []
#     dim = 0
#     for step in range(steps):
#     for dim_i in range(6):
#         delta = np.zeros(6)
#         #dim = np.random.randint(0, 6)
#         sign = np.random.choice([-1, 1])
#         delta[dim_i] = sign*0.05
#         ee_list.append(delta)
#         # dim+=1
#     return ee_list


# EE_POSITIONS_LIST = make_ee_positions_list(steps=10)

# JOINT_VELOCITY_LIST =[[0.1, 0.0,    0.0, 0.0,    0.0,    0.0, 0.0]] #,
# # [0.0, -0.1,    0.0, 0.0,    0.0,    0.0, 0.0],
# # [0.0, 0.0,    0.1, 0.0,    0.0,    0.0, 0.0],
# # [0.0, 0.0,    0.0, -0.1,    0.0,    0.0, 0.0],
# # [0.0, 0.0,    0.0, 0.0,    0.1,    0.0, 0.0],
# # [0.0, 0.0,    0.0, 0.0,    0.0,    -0.1, 0.0],
# # [0.0, 0.0,    0.0, 0.0,    0.0,    0.0, 0.05]]      

# JOINT_IMP_LIST = [[0.05, 0.0,    0.0, 0.0,    0.0,    0.0, 0.0]]
# # [0.0, -0.05,    0.0, 0.0,    0.0,    0.0, 0.0],
# # [0.0, 0.0,    0.05, 0.0,    0.0,    0.0, 0.0],
# # [0.0, 0.0,    0.0, -0.05,    0.0,    0.0, 0.0],
# # [0.0, 0.0,    0.0, 0.0,    0.05,    0.0, 0.0],
# # [0.0, 0.0,    0.0, 0.0,    0.0,    -0.05, 0.0],
# # [0.0, 0.0,    0.0, 0.0,    0.0,    0.0, 0.05]]           

# JOINT_TORQUE_LIST = [[0.05, 0.0,    0.0, 0.0,    0.0,    0.0, 0.0],
# [0.0, -0.1,    0.0, 0.0,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.1, 0.0,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.0, -0.1,    0.0,    0.0, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.1,    0.0, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.0,    -0.1, 0.0],
# [0.0, 0.0,    0.0, 0.0,    0.0,    0.0, 0.05]]        

# def get_action_for_controller(ctrl_type, step_num):
#     """ Get appropriate action based on controller type
#     """
#     if ctrl_type == "EEImpedance": 
#         action = EE_POSITIONS_LIST[step_num]
#     else:
#         raise ValueError("invalid control type string")
#     return action

# # Analyze results
# def check_state(control_name, obs_list):
#   global JOINT_IMP_LIST
#   if control_name == "JointImpedance":
#     print(obs_list)

#     delta_list = [np.subtract(obs_list[j+1]['q'],obs_list[j]['q']) for j in range(len(obs_list) -1)]
#     for j, (action, delta) in enumerate(zip(JOINT_IMP_LIST, delta_list)):
#       print("error {}".format(np.subtract(action, delta)))

# env = DemoControlEnv('dev/demo_control/demo_control_cfg.yaml', True, 'Demo Control Env')
# print("Perls2 Demo Control Environment Created.")



# selected_control_name = CONTROL_TYPES[control_selected]

# env.robot_interface.change_controller(selected_control_name)

# obs_list = []
# ## TODO ENABLE INITIAL RESET
# # init_obs = env.reset()
# # obs_list.append(init_obs)

# delta_list = []

# for step, action in enumerate(command_dict[selected_control_name]):
#     print("in control loop")
#     start = time.time()

#     print(action)
#     obs, _, _, _ = env.step(action)
#     obs_list.append(obs)
#     print("waiting")
#     #import pdb; pdb.set_trace()


# #env.reset()
# env.robot_interface.disconnect()

# check_state(selected_control_name, obs_list)
# print("Demo complete.")


# import matplotlib
# matplotlib.use('TkAgg')
# import matplotlib.pyplot as plt

# step_data = np.load('dev/logs/control/step0.npz')['ee_list']
# step0_x = [pos[0] for pos in step_data]
# print(step0_x)
# plt.plot(step0_x)
# plt.show()

