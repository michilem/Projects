import quaternion
import numpy as np
def ecompass(a, m):
    # Implementation of electric compass

    # Arguments: 
    # a: accelerometer measurement
    # m: magnetometer measurement

    # Return:
    # ori: oritation represented in quaternion

    # Initialization of rotation matrix
    R = np.zeros((3,3))
    
    # TODO: Follow the instruction in section 1 to calculate each row of the rotation matrix

    # TODO: convert the rotation matrix to quaternion and retuen the quaternion
    v1 = np.cross(np.cross(a, m),  a)
    v2 = np.cross(a, m)
    v3 = a
    R[:, 0] = v1
    R[:, 1] = v2
    R[:, 2] = v3
    return quaternion.from_rotation_matrix(R)