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
    kk = np.matmul(RM, point.T).T
    new_p = current_state[0:2] + np.matmul(RM, point.T).T * scale
    return np.squeeze(new_p)


def trans_global_to_local(current_state, point, scale=1):
    vec = point - current_state[0:2]
    RM = get_rotation_matrix(-current_state[2])
    new_p = np.matmul(RM, vec.T).T * scale
    return np.squeeze(new_p)


"""
  local 
coordinate axis 
 Ly         Gy         Lx
  \         |          /
   \        |         /
    \       |   .____/____Point G after rotate the global coordinate axis counter 
     \      |       /             clockwise with theta (radian),
      \     |      /      this point's coordinate is (A,B) in local coordinate                                
       \    |     /                                  (E,F) in global coordinate
        \   |    /       
         \  |   /       ._______Point G ,this point's cooedinate is (A,B) in global coordinate                                            
          \ |  /                                                    (C,D) in local coordinate
           \|_/_theta________________________Gx, global coordinate axis

we know 
    / cos(theta) -sin(theta) \  a global to local rotation matrix, which rotates  
   |                         |  the global coordinate axis in counter clockwise direction with 
   \  sin(theta) cos(theta) /   theta (radian) to the local coordinate axis.
   
    / cos(-theta) -sin(-theta) \  a local to global rotation matrix, which rotates  
   |                           |  the local coordinate axis in clockwise direction with 
   \  sin(-theta) cos(-theta) /   theta (radian) to the global coordinate axis.

### Note: theta is the angle between local x-axis and global x_axis, a positive
          value, so we need to put -theta into clockwise rotation matrix and
          +theta into counter clockwise rotation matrix.
          
we also know 
     / E  \    / cos(theta) -sin(theta) \ / A  \    
    |     | = |                         ||     |   
    \  F /    \  sin(theta) cos(theta) / \  B /
    
    which can be explained in a situation like, here we have a point(A,B) in 
local coordinate axis, and we want to know what is its x and y value in global 
coordinate axis, so we multiply the rotation matrix by (A,B), a rotation matrix
which rotates the global coordinate axis in counter clockwise direction with 
theta (radian), this is actually what the transfer_local_to_global function does.

    In contrast, here we have a point(A,B) in global coordinate axis, and we want 
to know what is its x and y value in local coordinate axis, so we just need to 
multuply the rotation matrix by (A,B), a rotation matrix which rotates the local coord-
inate axis in clockwise direction with theta (radian), this is what the 
transfer_global_to_local function does.

Conclusion:
    
transfer_local_to_global function:      
    Having a local coordinate and want a global coordinate at the same point.
=> multuply a global to local rotation matrix by local coordinate.
    
transfer_global_to_local function:
    Having a global coordinate and want a local coordinate at the same point.
=> multuply a local to global rotation matrix by global coordinate.

"""
