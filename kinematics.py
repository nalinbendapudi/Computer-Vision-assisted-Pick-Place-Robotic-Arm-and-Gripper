import numpy as np
import math
#expm is a matrix exponential function
from scipy.linalg import expm

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""

LS = np.array([113.69, 101.4, 112.85, 119.17])

# Checks if a matrix is a valid rotation matrix.
def _isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def _rotationMatrixToEulerAngles(R) :
    assert(_isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def matrix_to_trans_euler(matrix):
    assert(matrix.shape==(4,4))
    return list(np.append(matrix[:3, 3].reshape(-1), _rotationMatrixToEulerAngles(matrix[:3,:3])))

def dh2matrix(joint_angles, link_id):
    '''
    INPUT: link id
    OUTPUT: transformation matrix
    '''
    assert(len(joint_angles)>=link_id)
    DH_TABLE = np.array([
        [joint_angles[0],   LS[0],      0,      -np.pi/2],
        [joint_angles[1]-np.pi/2,   0    ,  LS[1],             0],
        [joint_angles[2]+np.pi/2,   0    ,      0,       np.pi/2],
        [joint_angles[3],   LS[2],      0,      -np.pi/2],
        [joint_angles[4],   0    ,      0,       np.pi/2],
        [joint_angles[5],   LS[3],      0,             0],
    ])
    theta, d, a, alpha = DH_TABLE[link_id]
    m =  np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                    [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                    [0            , np.sin(alpha)               , np.cos(alpha)              , d              ],
                    [0            , 0                           , 0                          , 1              ]])
    if not (_isRotationMatrix(m[:3,:3])):
        raise AssertionError(str(link_id) + " matrix is invalid")
    return m


def get_transformation(joint_angles, target = 6, base = 0):
    rtn = np.eye(4)
    for link_id in np.arange(base, target):
        rtn = np.matmul(rtn, dh2matrix(joint_angles, link_id))

        if not _isRotationMatrix(rtn[:3,:3]):
            raise AssertionError("Product matrix is invalid")
    return rtn



def IK(pose, angle_limits):
    # pose is the final 4X4 homogeneous matrix of last frame. This is a numpy array
    R = pose[:3,:3]
    X = pose[0][3]
    Y = pose[1][3]
    Z = pose[2][3]

    # end effector position for the first three joints
    X_c = X - LS[3]*R[0][2]
    Y_c = Y - LS[3]*R[1][2]
    Z_c = Z - LS[3]*R[2][2]

    # parameters for computing the configuration of first three joints
    r = np.sqrt(X_c**2 + Y_c**2)      # x in 2R arm case
    s = Z_c - LS[0]                   # y in 2R arm case

    v1 = np.arctan2(Y_c,X_c)

    # theta 1 has two solutions    
    for i in [v1, v1 + np.pi]:
        theta1 = i
        
        acos_theta_3 = (r**2+s**2-LS[1]**2-LS[2]**2)/(2*LS[1]*LS[2])
        if acos_theta_3 < -1 or acos_theta_3 > 1:
            continue
        v3 = np.arccos(acos_theta_3)

        # theta 3 has two solutions
        for j in [v3, -v3]:
            theta3 = j

            # the formulas for computing theta 2 are different for different theta 1 
            if theta1 == v1:
                sign = 1.0
            else:
                sign = -1.0

            theta2 = sign * (np.pi/2 - np.arctan2(s,r) - sign * (np.arctan2(LS[2]*np.sin(theta3) , LS[1]+LS[2]*np.cos(theta3))))

            R_01 = np.array([[np.cos(theta1),-np.sin(theta1),0.],[np.sin(theta1), np.cos(theta1),0.],[0., 0., 1.]])
            R_12 = np.array([[np.cos(theta2),0.,np.sin(theta2)],[0.,1.,0.],[-np.sin(theta2),0.,np.cos(theta2)]])
            R_23 = np.array([[np.cos(theta3),0.,np.sin(theta3)],[0.,1.,0.],[-np.sin(theta3),0.,np.cos(theta3)]])

            R_02 = np.matmul(R_01,R_12)
            R_03 = np.matmul(R_02,R_23)

            # get the orientation difference between the end effector of the first three joints and all six joints
            R_36 = np.matmul(R_03.T,R)

            theta4 = np.arctan2(R_36[1][2], R_36[0][2])

            theta6 = np.arctan2(R_36[2][1], -R_36[2][0])

            sin_theta5 = R_36[0][2]/np.cos(theta4)

            cos_theta5 = R_36[2][2]

            theta5 = np.arctan2(sin_theta5, cos_theta5)


            cfg = [theta1, theta2, theta3, theta4, theta5, theta6]

            viable = 1

            # check if the configuration exceed joint anlge limits
            for i in range(len(cfg)) :
                if cfg[i] < angle_limits[i,0] or cfg[i] > angle_limits[i,1]:
                    viable = 0
                    break

            if viable:
                return cfg
                    
    return None


def get_T_from_angle(axis, angle):
    if axis == 0:
        rotation = np.array([[1.0, 0.0, 0.0],[0.0, np.cos(angle), -np.sin(angle)],[0.0, np.sin(angle), np.cos(angle)]])
    elif axis == 1:
        rotation = np.array([[np.cos(angle), 0.0, np.sin(angle)],[0.0, 1.0, 0.0],[-np.sin(angle), 0.0, np.cos(angle)]])
    elif axis == 2:
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0.0],[np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]])
    return rotation









def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    pass

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pass