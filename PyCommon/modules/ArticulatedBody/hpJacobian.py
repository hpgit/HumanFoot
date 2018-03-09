import numpy as np
import math
from PyCommon.modules.Math import mmMath as mm

JACOBIAN_EPS = 1e-6
SCALAR_1 = 1.
SCALAR_1_2 = .5
SCALAR_1_6 = 1./6.
SCALAR_1_24 = 1./24.
SCALAR_1_120 = 1./120.


def compute_jacobian(J, q, joint_dofs, joint_positions, effector_positions, ancestor_mask,
                    joint_linear_first=True, effector_linear_first=True):
    '''

    :type J: np.ndarray
    :param q:
    :param joint_dofs:
    :param joint_positions:
    :param effector_positions:
    :param ancestor_mask:
    :param joint_linear_first:
    :param effector_linear_first:
    :return:
    '''
    # ancestor_mask[i][j] => whether i'th effector of joint affected by j'th joint
    assert len(joint_dofs) == len(joint_positions)
    assert len(effector_positions) > 0
    dof_offset = 0

    for effector_idx in range(len(effector_positions)):
        effector_position = effector_positions[effector_idx]
        for joint_idx in range(len(joint_dofs)):
            dof = joint_dofs[joint_idx]
            joint_position = joint_positions[joint_idx]
            _J = None  # type: np.ndarray
            if dof == 6:
                _J = compute_joint_jacobian_6dof(q, joint_position, effector_position)
            elif dof == 3:
                _J = compute_joint_jacobian_3dof(q, joint_position, effector_position)
            else:
                raise NotImplementedError
            J[6*effector_idx:6*effector_idx+6, dof_offset:dof_offset+dof] = _J

            dof_offset = dof_offset + dof


def compute_basic_jacobian_3dof(q):
    t = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2])
    t2 = t * t
    alpha, beta, gamma = 0., 0., 0.
    if t < JACOBIAN_EPS:
        alpha = SCALAR_1_6 - SCALAR_1_120 * t2
        beta = SCALAR_1 - SCALAR_1_6 * t2
        gamma = SCALAR_1_2 - SCALAR_1_24 * t2
    else:
        beta = math.sin(t) / t
        alpha = (SCALAR_1 - beta) / t2
        gamma = (SCALAR_1 - math.cos(t)) / t2

    return alpha * mm.getDyadMatrixForm(q) + beta * np.eye(3) - gamma * mm.getCrossMatrixForm(q)


def compute_joint_jacobian_3dof(q, joint_orientation, joint_position, effector_position):
    '''

    :rtype: np.ndarray
    :param np.ndarray q:
    :param np.ndarray joint_orientation:
    :param np.ndarray joint_position:
    :param np.ndarray effector_position:
    :return:
    '''
    # jw is represented in body frame
    jw = compute_basic_jacobian_3dof(q)
    jw_global = np.dot(joint_orientation, jw)
    p = effector_position - joint_position

    return np.dot(-mm.getCrossMatrixForm(p), jw_global)


def compute_joint_jacobian_6dof(q, joint_orientation, joint_position, effector_position):
    j = np.eye(6)
    j[3:, 3:] = compute_joint_jacobian_3dof(q[3:], joint_orientation, joint_position, effector_position)
    return j

