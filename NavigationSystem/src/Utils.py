import numpy as np

def get_rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array(((c,-s), (s,c)))

"""
Local coordinate: 
    Car coordinate. X direction is its way direction.
Global coordinate:
    Buttom line is the X direction.

Transform from local to global coordinate, which mean
to rotate the basis with positive theta (x direction 
in car coordinate).

Input:
    current_state: car pose in global coordinate.
    point: 1d or 2d nparry, in local coordinate.
        format: [[x, y],
                 [x, y]....]
    scale: Zoom up the point in local coordinate.
Return:
    point: 1d or 2d nparray in global coordinate.
"""
def trans_local_to_global(current_state, point, scale=1):
    RM = get_rotation_matrix(current_state[2])
    # Broacasting plus
    new_p = current_state[0:2] + np.matmul(RM, point.T).T * scale
    return np.squeeze(new_p)


def trans_global_to_local(current_state, point, scale=1):
    vec = point - current_state[0:2]
    RM = get_rotation_matrix(-current_state[2])
    new_p = np.matmul(RM, vec.T).T * scale
    return np.squeeze(new_p)
