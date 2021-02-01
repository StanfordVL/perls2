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
    return np.fromstring(state_str, dtype=np.float, sep= ' ').reshape(shape).T  

def ndarray_to_eigen_str(value):
    """Sets the key to format compatible for franka-panda eigen
    """
    if isinstance(value, np.ndarray):
        value = str(value)
    else:
        raise ValueError("value should be ndarray")

    # remove brackets
    value = value[1:-1]
    return value
