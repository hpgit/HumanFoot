import pydart2 as pydart
import numpy as np
from math import sin, cos

from PyCommon.modules.Math import mmMath as mm

BJOINT_EPS = 1e-6

SCALAR_1 = 1.
SCALAR_1_2 = 1./2.
SCALAR_1_6 = 1./6.
SCALAR_1_24 = 1./24.
SCALAR_1_120 = 1./120.


ANGULAR_FIRST = False


def get_bjoint_jacobian(m_rQ):
    t = np.linalg.norm(m_rQ)
    t2 = t * t

    if t < BJOINT_EPS:
        alpha = SCALAR_1_6 - SCALAR_1_120 * t2
        beta = SCALAR_1 - SCALAR_1_6 * t2
        gamma = SCALAR_1_2 - SCALAR_1_24 * t2
    else:
        beta = sin(t) / t
        alpha = (SCALAR_1 - beta) / t2
        gamma = (SCALAR_1 - cos(t)) / t2

    return alpha * mm.getDyadMatrixForm(m_rQ) + beta * np.eye(3) - gamma * mm.getCrossMatrixForm(m_rQ)


def compute_jacobian(skel, idx_or_name, eff_global_pos):
    """

    :param skel:
    :type skel: pydart.Skeleton
    :param idx_or_name:
    :type idx_or_name: str | int
    :param eff_global_pos:
    :type eff_global_pos: np.ndarray
    :return:
    """
    parent_joint = skel.body(idx_or_name).parent_joint
    ancestors = list()
    while parent_joint is not None:
        ancestors.append(parent_joint.id)
        parent_joint = parent_joint.parent_bodynode.parent_joint if parent_joint.parent_bodynode is not None else None

    J = np.zeros((6, skel.num_dofs()))

    # root joint
    if True:
        J[0, 3] = 1.
        J[1, 4] = 1.
        J[2, 5] = 1.

        joint_frame = skel.joint(0).get_world_frame_after_transform()
        offset = eff_global_pos - joint_frame[3, :3]
        _Jw = np.dot(joint_frame[:3, :3], get_bjoint_jacobian(mm.logSO3(joint_frame[:3, :3])))
        _Jv = -np.dot(mm.getCrossMatrixForm(offset), _Jw)
        if ANGULAR_FIRST:
            J[3:6, 0:3] = _Jv
            J[0:3, 0:3] = _Jw
        else:
            J[0:3, 0:3] = _Jv
            J[3:6, 0:3] = _Jw

    # internal joint
    for i in range(skel.num_joints()):
        if i in ancestors and skel.joint(i).num_dofs() == 3:
            joint = skel.joint(i)
            joint_frame = joint.get_world_frame_after_transform()
            offset = eff_global_pos - joint_frame[3, :3]
            dof_start_index = joint.dofs[0].index_in_skeleton()
            _Jw = np.dot(joint_frame[:3, :3], get_bjoint_jacobian(skel.q[dof_start_index:dof_start_index+3]))
            _Jv = -np.dot(mm.getCrossMatrixForm(offset), _Jw)
            if ANGULAR_FIRST:
                J[3:6, dof_start_index:dof_start_index+3] = _Jv
                J[0:3, dof_start_index:dof_start_index+3] = _Jw
            else:
                J[0:3, dof_start_index:dof_start_index+3] = _Jv
                J[3:6, dof_start_index:dof_start_index+3] = _Jw
        else:
            raise NotImplementedError("only ball joint")





