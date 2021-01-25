"""Class definition for a demo path.
"""
import numpy as np
import logging
logger = logging.getLogger(__name__)

class Path(object):
    """A path is a sequence of goal states for the robot to achieve that
    follow a specified pattern. 

    The path may be either in joint space or in cartesian space as end-effector
    poses.

    Attributes:
        num_pts (int): number of points in path.
        path (list): list 7f of absolute goal poses in path. Goal poses
            are specified by [x, y, z, qx, qy, qz, w] position and quaternion.
        deltas (list): list 6f deltas between poses to produce path.
    """

    def __init__(self, shape, num_pts):
        self.shape = shape
        self.num_pts = num_pts
        self.path = []

    def make_path(self):
        """Create path by sequentially adding deltas to initial state
        """
        self.path = [self.start_pose]

        for delta in self._deltas:
            new_pose = np.add(self.path[-1], delta)
            self.path.append(new_pose)


class SequentialJoint(Path):
    """ Series of joint positions in which a single joint is gradually incremented, 
    held at desired position, and then gradually decremented back to intial state. 

    i.e.(Move a single joint back and forth)

    Attributes: 
        start_pose (list): initial joint states
        delta_val (double): amount to increment/decrement 

    """
    def __init__(self, start_pose, delta_val=0.001, num_steps=30, joint_num=0):
        logger.info("Sequential Joint path")

        self.start_pose = start_pose
        self.delta_val = delta_val
        self.num_steps = num_steps
        self.joint_num = joint_num
        self._deltas = self._get_deltas()

        self.path = []
        self.make_path()

    def _get_deltas(self):
        """Return series of joint deltas where each joint is individually
            incremented, and then decremented by delta.
        """

        deltas = []
        # Increment by delta
        for _ in range(self.num_steps):
            delta = np.zeros(7)
            delta[self.joint_num] = self.delta_val
            deltas.append(delta)

        # Hold final pose
        for _ in range(self.num_steps):
            delta = np.zeros(7)
            deltas.append(delta)

        # Return to intial pose, decrementing by delta
        for _ in range(self.num_steps):
            delta = np.zeros(7)
            delta[self.joint_num] = -self.delta_val
            deltas.append(delta)

        # Hold initial pose (avoids jerky stop / immediate reset)
        for _ in range(self.num_steps):
            delta = np.zeros(7)
            deltas.append(delta)

        return deltas

    def make_path(self):
        """Create path by sequentially adding deltas to joint pose.
        """

        self.path = [self.start_pose]

        for delta in self._deltas:
            new_pose = np.add(self.path[-1], delta)
            self.path.append(new_pose)


class Line(Path):
    """Class definition for straight line in given direction.
    """
    def __init__(self, start_pose, num_pts, path_length,
                 delta_val=None, dim=0, end_pos=None):
        """ Initialize Line class

        Args:
            start_pose (list): 7f pose at start of path. Best to
                set at robot reset pose.
            num_pts (int): number of points in path.
            path_length (float): length of path in m
            delta_val (float): (optional) delta in m between
                each step. If None, end_pos must be specified.
            dim (int): direction to move for line, x = 0, y=1,
                z=2.
        """
        self.start_pose = start_pose
        self.num_pts = num_pts
        self.path_length = path_length
        if delta_val is None:
            delta_val = np.divide(self.path_length, self.num_pts)
        self.delta_val = delta_val
        self.dim = dim
        self._deltas = []
        self.get_deltas()
        self.path = []
        self.make_path()

    def get_deltas(self):

        delta = np.zeros(6)
        delta[self.dim] = self.delta_val
        self._deltas = [delta]*self.num_pts
        # self._deltas[0] = np.zeros(7)

class Square(Path):
    """Class def for square path.

    Square path defined by side length and start point.
    At step 4 * sidelength -1, ee is not at initial point.
    Last step returns to initial point.

    Square path is ordered in clockwise from origin (Bottom, Left, Top, Right)

    Attributes:
        start_pose (7f): start pose to begin square from.
        num_pts (int): number of steps to take on each side.
        delta_val (float): step size in m to take for each step.
        _deltas (list): list of delta xyz from a position to reach next position
             on path.
        path (list): list of actions to take to perform square path. Actions
            are either delta xyz from current position (if use_abs is False) or
            they are absolute positions taken by adding the deltas to start.

    """
    def __init__(self, start_pose, side_num_pts, delta_val):

        self.start_pose = start_pose
        self.num_pts = side_num_pts
        self.delta_val = delta_val
        self._deltas = []
        self.get_deltas()
        self.path = []
        self.make_path()

    def get_deltas(self):
        """ Get a series of steps from current position that produce
        a square shape. Travel starts with bottom side in positive direction,
        then proceeds clockwise (left, top, right.)

        """
        self._deltas = [[0, 0, 0, 0, 0, 0]]
        # Bottom side.
        for pt in range(self.num_pts):
            self._deltas.append([self.delta_val, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Left Side
        for pt in range(self.num_pts):
            self._deltas.append([0.0, -self.delta_val, 0.0, 0.0, 0.0, 0.0])
        # Top side
        for pt in range(self.num_pts):
            self._deltas.append([-self.delta_val, 0.0, 0.0, 0.0, 0.0, 0.0])
        # Right side
        for pt in range(self.num_pts):
            self._deltas.append([0.0, self.delta_val, 0.0, 0.0, 0.0, 0.0])


class Rotation(Path):
    """ Class definition for path that rotating end effector in place.
    Start and end orientation should be in euler angles.
    """
    def __init__(self, start_pose, num_pts,
                 rotation_rad=np.pi/4, delta_val=None, dim=2, end_ori=None):
        logger.debug("Making Rotation Path")
        self.start_pose = start_pose
        self.end_ori = end_ori
        self.num_pts = num_pts
        self.rotation_rad = rotation_rad
        if delta_val is None:
            if num_pts == 0:
                delta_val = 0
            else:
                delta_val = np.divide(rotation_rad, num_pts)
        self.dim = dim
        self.delta_val = delta_val
        self.get_deltas()
        self.path = []
        self.make_path()

    def get_deltas(self):
        """Convert euler angle rotation with magnitude delta in the direction
        specified by dim.
        """
        delta = np.zeros(3)
        delta[self.dim] = self.delta_val
        # pad with position deltas= 0
        delta = np.hstack(([0, 0, 0], delta))
        self._deltas = [delta]*self.num_pts