import numpy as np
import math
#expm is a matrix exponential function
from scipy.linalg import expm

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""

LS = np.array([122.43, 110.02, 121.7, 118.48])

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
 
    # import pdb;pdb.set_trace()
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
        [joint_angles[1],   0    ,  LS[1],             0],
        [joint_angles[2],   0    ,      0,       np.pi/2],
        [joint_angles[3],   LS[2],      0,      -np.pi/2],
        [joint_angles[4],   0    ,      0,       np.pi/2],
        [joint_angles[5],   LS[3],      0,             0],
    ])
    a, alpha, d, theta = DH_TABLE[link_id]
    m =  np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                    [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                    [0            , np.sin(alpha)               , np.cos(alpha)              , d              ],
                    [0            , 0                           , 0                          , 1              ]])
    if not (_isRotationMatrix(m[:3,:3])):
        raise AssertionError(str(link_id) + " matrix is invalid")
    return m


def get_transformation(joint_angles, target = 6, base = 0):
    rtn = np.eye(4)
    for link_id in np.arange(base, target): # [0,1,2,3,4,5]
        rtn = np.matmul(dh2matrix(joint_angles, link_id), rtn)

        if not _isRotationMatrix(rtn[:3,:3]):
            raise AssertionError("Product matrix is invalid")
    return rtn

def FK_dh(joint_angles, link):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    pass

def FK_pox(joint_angles):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    using product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the 
    desired link

    note: phi is the euler angle about y in the base frame

    """
    pass

def IK(pose):
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    pass


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




def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass