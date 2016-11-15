from PyCommon.modules.pyVirtualPhysics import *
import numpy as np
import math


def pyVec3_2_Vec3(pyV):
    return Vec3(pyV[0], pyV[1], pyV[2])

def pyVec_2_Vec3_extend(pyV):
    if len(pyV) == 1:
        return Vec3()

def Vec3_2_pyVec3(vpVec3):
    return np.array((vpVec3[0], vpVec3[1], vpVec3[2]))

def pyVec3_2_Axis(pyV):
    return Axis(pyV[0], pyV[1], pyV[2])

def Axis_2_pyVec3(vpAxis):
    return Vec3_2_pyVec3(vpAxis)

def se3_2_pyVec6(vpse3):
    return np.array((vpse3[0],vpse3[1],vpse3[2],vpse3[3],vpse3[4],vpse3[5]))

def pyVec6_2_se3(pyV):
    return se3(pyV[0], pyV[1], pyV[2], pyV[3], pyV[4], pyV[5])

def dse3_2_pyVec(vpdse3):
    return se3_2_pyVec6(vpdse3)

def pyVec6_2_dse3(pyV):
    return dse3(pyV[0], pyV[1], pyV[2], pyV[3], pyV[4], pyV[5])

def SE3_2_pySO3(T):
    pyR = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            pyR[j, i] = T[3*i + j]

    return pyR


def SE3_2_pySE3(T):
    pyV = np.zeros((3, 1))
    pyV[0, 0] = T[9]
    pyV[1, 0] = T[10]
    pyV[2, 0] = T[11]

    pyVV = np.zeros(4)
    pyVV[3] = 1.

    return np.vstack((np.hstack((SE3_2_pySO3(T), pyV)), pyVV))


def pySO3_2_SE3(pyR):
    T = SE3(pyR[0, 0], pyR[1,0], pyR[2,0], pyR[0, 1], pyR[1,1], pyR[2,1], pyR[0,2], pyR[1,2], pyR[2,2])

    return T


def pySE3_2_SE3(pyT):
    T = SE3(pyT[0, 0],pyT[1, 0],pyT[2, 0],pyT[0, 1],pyT[1, 1],pyT[2, 1],pyT[0, 2],pyT[1, 2],pyT[2, 2],pyT[0, 3]    , pyT[1, 3], pyT[2, 3])

    return T


def getSE3FromVectors(vec1, vec2):
    v1 = Normalize(vec1)
    v2 = Normalize(vec2)

    rot_axis = Normalize(Cross(v1, v2))
    inner = Inner(v1, v2)
    theta = math.acos(inner)

    if (rot_axis[0] == 0.) and (rot_axis[1] == 0.) and (rot_axis[2] == 0.):
        rot_axis = Vec3(0., 1., 0.)
    elif inner < -1.0 + LIE_EPS:
        rand = np.random.uniform(0., 1., 3)
        rand_vec = Vec3(rand[0], rand[1], rand[2])
        rot_axis = Normalize(Cross(v1, Normalize(rand_vec)))

    x = rot_axis[0]
    y = rot_axis[1]
    z = rot_axis[2]

    c = inner
    s = math.sin(theta)
    R = SE3(c + (1.0-c)*x*x,    (1.0-c)*x*y - s*z,    (1-c)*x*z + s*y,
            (1.0-c)*x*y + s*z,    c + (1.0-c)*y*y,    (1.0-c)*y*z - s*x,
            (1.0-c)*z*x - s*y,    (1.0-c)*z*y + s*x,    c + (1.0-c)*z*z)

    return Inv(R)
