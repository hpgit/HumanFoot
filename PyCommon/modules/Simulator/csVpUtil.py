from PyCommon.modules.pyVirtualPhysics import *
import numpy as np
import numpy.linalg as npl
import math


def pyVec3_2_Vec3(pyV):
    """

    :param pyV:
    :return: Vec3
    """
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
    """

    :param pyR:
    :return: SE3
    """
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

def genangvel_2_angvel(genangvel, r):
    """

    :param genangvel: np.array
    :param r: np.array
    :return: np.array
    """
    m_rQ = r
    m_rDq = genangvel
    t = npl.norm(m_rQ)
    alpha = 0
    beta = 0
    gamma = 0
    t2 = t * t

    if t < BJOINT_EPS:
        alpha = SCALAR_1_6 - SCALAR_1_120 * t2
        beta = SCALAR_1 - SCALAR_1_6 * t2
        gamma = SCALAR_1_2 - SCALAR_1_24 * t2
    else:
        beta = math.sin(t) / t
        alpha = (SCALAR_1 - beta) / t2
        gamma = (SCALAR_1 - math.cos(t)) / t2

    V = (alpha * np.dot(m_rQ, m_rDq)) * m_rQ + beta * m_rDq + gamma * np.cross(m_rDq, m_rQ)
    return V

def angvel_2_genangvel(angvel, r):
    """

    :param angvel: np.array
    :param r: np.array
    :return: np.array
    """
    m_rQ = r
    W = angvel
    t = npl.norm(m_rQ)
    delta = 0
    zeta = 0
    t2 = t * t

    if t < BJOINT_EPS:
        delta = SCALAR_1_12 + SCALAR_1_720 * t2
        zeta = SCALAR_1 - SCALAR_1_12 * t2
    else:
        zeta = SCALAR_1_2 * t * (SCALAR_1 + math.cos(t)) / math.sin(t)
        delta = (SCALAR_1 - zeta) / t2

    genangvel = (delta * np.dot(m_rQ, W)) * m_rQ + zeta * W + SCALAR_1_2 * np.cross(m_rQ, W)
    # genangvel = (delta * np.dot(m_rQ, W)) * m_rQ
    # genangvel += zeta*W
    # genangvel += SCALAR_1_2 *np.cross(m_rQ, W)
    return genangvel

def angacc_2_genangacc(angacc, r, dr, angvel):
    """

    :param angacc: np.array
    :param r: np.array
    :param dr: np.array
    :param angvel: np.array
    :return: np.array
    """
    m_rQ = r
    m_rDq = dr
    V = angvel
    W = V
    DW = angacc
    t = np.linalg.norm(m_rQ)
    qw = np.dot(m_rQ, W)
    t2 = t * t
    beta = 0
    gamma = 0
    delta = 0
    zeta = 0
    d_delta = 0
    d_eta = 0

    if t < BJOINT_EPS:
        SCALAR_1_7560 = 0.000132275132275132275132
        SCALAR_1_360 = 0.00277777777777777777778

        delta = SCALAR_1_12 + SCALAR_1_720 * t2
        zeta = SCALAR_1 - SCALAR_1_12 * t2

        d_delta = (SCALAR_1_360 + SCALAR_1_7560 * t2) * qw
        d_eta = -(SCALAR_1_6 + SCALAR_1_180 * t2) * qw
    else:
        beta = math.sin(t) / t
        gamma = (SCALAR_1 - math.cos(t)) / t2

        zeta = SCALAR_1_2 * beta / gamma
        delta = (SCALAR_1 - zeta) / t2

        d_eta = SCALAR_1_2 * (beta - SCALAR_1) / gamma / t2 * qw
        d_delta = -(d_eta + SCALAR_2 * (SCALAR_1 - zeta) / t2 * qw) / t2

    m_rDdq = (d_delta * qw + delta * (np.dot(m_rQ, DW) + np.dot(m_rDq, W))) * m_rQ \
             + (delta * qw) * m_rDq \
             + d_eta * W + zeta * DW \
             + SCALAR_1_2 * (np.cross(m_rQ, DW) + np.cross(m_rDq, W))
    return m_rDdq

def genangacc_2_angacc(genangacc, r, dr, angvel):
    m_rQ = r
    m_rDq = dr
    m_rDdq = genangacc
    t = npl.norm(m_rQ)
    alpha = 0
    beta = 0
    gamma = 0
    d_alpha = 0
    d_beta = 0
    d_gamma = 0
    q_dq = np.dot(m_rQ, m_rDq)
    t2 = t * t

    if t < BJOINT_EPS:
        alpha = SCALAR_1_6 - SCALAR_1_120 * t2
        beta = SCALAR_1 - SCALAR_1_6 * t2
        gamma = SCALAR_1_2 - SCALAR_1_24 * t2

        d_alpha = (SCALAR_1_1260 * t2 - SCALAR_1_60) * q_dq
        d_beta = (SCALAR_1_30 * t2 - SCALAR_1_3) * q_dq
        d_gamma = (SCALAR_1_180 * t2 - SCALAR_1_12) * q_dq
    else:
        beta = math.sin(t) / t
        alpha = (SCALAR_1 - beta) / t2
        gamma = (SCALAR_1 - math.cos(t)) / t2

        d_alpha = (gamma - SCALAR_3 * alpha) / t2 * q_dq
        d_beta = (alpha - gamma) * q_dq
        d_gamma = (beta - SCALAR_2 * gamma) / t2 * q_dq

    angacc = (d_alpha * q_dq + alpha * (SquareSum(m_rDq) + Inner(m_rQ, m_rDdq))) * m_rQ \
            + (alpha * q_dq + d_beta) * m_rDq + beta * m_rDdq \
            + Cross(d_gamma * m_rDq + gamma * m_rDdq, m_rQ)
    return angacc