from obj_parts import ObjParts


def get_geometry(delta):
    """
    Return the geometry, where:
    0: Sphere
    1: Cylinder
    2: Box

    Parameters:
    -----------
        delta: float
    Returns:
    -----------
        geometry: int
    """
    if (delta >= 0 and delta <= 0.1 ):
		return 0
    elif (delta > 0.1 and delta <= 0.4):
        return 1
    else:
        return 2

def check_similarity(var1, var2, error):
    """
    Check the simulatiry between two numbers, considering a error margin.

    Parameters:
    -----------
        var1: float
        var2: float
        error: float
    Returns:
    -----------
        similarity: boolean
    """
    if((var1 <= (var2 + error)) and (var1 >= (var2 - error))):
        return True
    else:
        return False

def check_simple_geometry(parts):
    error = 0.008
    size = len(parts)
    for i in range(size):
        if i + 1 == size:
            return True
        elif (not check_similarity(parts[i].largura,parts[i+1].largura,error) or (get_geometry(parts[i].delta) != get_geometry(parts[i+1].delta)) ):
            return False
    
    return True

def get_grasp(parts, g_w):
    """
    Get the best region for grasping. With no part is viable, return -2.

    Parameters:
    -----------
        parts: ObjParts array N
        g_w: float
    Returns:
    -----------
        indice: int
    """
    for i in range(len(parts)):
        if(parts[i].largura < g_w and parts[i].curvatura > 0.016):
            return i
    return -2