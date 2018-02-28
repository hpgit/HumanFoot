import numpy as np
from PyCommon.modules.Math import mmMath as mm


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
            if dof == 6:
                _J = compute_joint_jacobian_6dof(q, joint_position, effector_position)
            elif dof == 3:
                _J = compute_joint_jacobian_3dof(q, joint_position, effector_position)
                pass
            else:
                raise NotImplementedError

            dof_offset = dof_offset + dof


def compute_basic_jacobian_3dof(q):
    # TODO: compute_basic_jacobian_3dof
    return np.eye(3)


def compute_joint_jacobian_3dof(q, joint_orientation,
                                joint_position, effector_position):
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
    pass

