"""Utility functions for PyBullet
"""
import pybullet as pb

def add_debug_line(start, end, physicsClientId, color=[0,0,1], width=3.0, lifetime=0):
    """Add 3d Line to pybullet visualization.

    Wrapper for pybullet.addUserDebugLine:

    Args:
        start (list): 3f xyz start position in World Frame
        end (list): 3f xyz end position in World Frame
        color (list): 3f RGB color in range [0..1]
        width (float): line width
        physicsClientId (int): physics server unique id.
        lifetime (float): (optional) positive time in seconds after which
            line is removed. default is 0 for permanent.

    Returns:
        (int): unqiue id for line object.
    """
    return pb.addUserDebugLine(lineFromXYZ=start,
                        lineToXYZ=end,
                        lineColorRGB=color,
                        lineWidth=width,
                        physicsClientId=physicsClientId,
                        lifeTime=lifetime)

def add_debug_text(text, position, physicsClientId, color=[0,0,1], size=14, lifetime=0):
    """Add 3d Text to pybullet visualization

    Wrapper for pb.addUserDebugText

    Args:
        text (str): text as string
        position (list): 3f xyz position of text in world coordinates.
        physicsClientId (int): physics server unique id.
        color (list): 3f rgb [0..1]
        size (float): font size of text.
        lifetime (float): (optional) positive time in seconds after which
            line is removed. default is 0 for permanent.

    Returns:
        text_id (int): unique id for text object.
    """
    return pb.addUserDebugText(text=text,
                        textPosition=position,
                        physicsClientId=physicsClientId,
                        textColorRGB=color,
                        textSize=size,
                        lifeTime=lifetime)

def remove_all_debug(physicsClientId):
    """Remove all debug parameters.

    Args:
        physicsClientId (int): unique id for physics server.
    Returns:
        None
    """
    pb.removeAllUserDebugItems(physicsClientId)

def remove_user_debug(itemId, physicsClientId):
    """Remove specific debug parameter.

    Args:
        itemId : unique id for debug object
        physicsClientId: unique id for physics server.
    """
    pb.removeUserDebugItem(itemId, physicsClientId)
