"""Utils for redis
"""

import numpy as np

######## Franka-Panda redis utils 3#####################3

def franka_state_to_np(state_str):
    """Helper function to convert redis states to 1d ndarray.

    Args:
        state_str (str): state from redis a string

    Returns
        ndarray
    """ 
    return np.fromstring(state_str, dtype=np.float, sep= ' ')

def franka_state_to_np_mat(state_str, shape): 
    """Helper function to convert franka-panda redis states
    to numpy ndarrays.

    Args: 
        state_str (str): state from redis. 
        shape (tuple): row, col shape of the resulting ndarray.
    
    Returns:
        ndarray with dims == shape
    """
    return np.fromstring(state_str, dtype=np.float, sep= ' ').reshape(shape)  