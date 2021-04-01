"""Utils for redis
"""

import numpy as np
import struct 
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

######### redis utility functions for images #################
def convert_frame_to_encoded_bytes(frame):
    """Convert rgb, depth or ir frame to bytes array with encoded dim
    """
    height = np.shape(frame)[0]
    width = np.shape(frame)[1]

    # Encode the shape of the picture into the bytes array
    frame_np = np.array(frame).astype('uint8')
    frame_bytes = frame_np.tobytes()
    frame_shape = struct.pack('>II', height, width)
    encoded_frame = frame_shape + frame_bytes
    return encoded_frame

def convert_encoded_frame_to_np(encoded_frame, dim):
    """Convert rgb, depth or ir bytes array to numpy
    """
    h, w = struct.unpack('>II', encoded_frame[:8])

    frame_np = np.frombuffer(
        encoded_frame, dtype=np.uint8, offset=8).reshape(h, w, dim)

    return frame_np
