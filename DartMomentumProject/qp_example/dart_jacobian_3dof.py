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

# ANGULAR_FIRST = False


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


def compute_jacobian(skel, idx_or_name, eff_global_pos, ANGULAR_FIRST=False):
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

    return J


def compute_J_dJdq(skel, ANGULAR_FIRST=False):
    """

    :param skel:
    :type skel: pydart.Skeleton
    :return:
    """
    body_num = skel.num_bodynodes()
    joint_num = skel.num_joints()
    active_joint_index = list()
    for i in range(joint_num):
        if len(skel.joint(i).dofs) > 0:
            active_joint_index.append(i)

    ndofs = skel.num_dofs()

    J = np.zeros((6*body_num, ndofs))
    dJdq = np.zeros(6*body_num)

    ##############################################
    # pre-processing
    ##############################################
    # get joint frames
    joint_frames = list()  # type: list[np.ndarray]
    for joint_idx in active_joint_index:
        joint = skel.joint(joint_idx)  # type: pydart.Joint
        joint_frames.append(joint.get_world_frame_after_transform())

    # get ancestors
    ancestors = list()  # type: list[list[int]]
    for body_idx in range(body_num):
        parent_joint = skel.body(body_idx).parent_joint
        body_ancestors = list()
        while parent_joint is not None:
            body_ancestors.append(parent_joint.id)
            parent_joint = parent_joint.parent_bodynode.parent_joint if parent_joint.parent_bodynode is not None else None

        ancestors.append(body_ancestors)

    ##############################################
    # root joint
    ##############################################
    # jacobian
    _Jw = joint_frames[0][:3, :3]
    for body_idx in range(body_num):
        eff_pos = skel.body(body_idx).to_world()
        offset = eff_pos - joint_frames[0][3, :3]
        _Jv = -np.dot(mm.getCrossMatrixForm(offset), _Jw)

        J[6*body_idx+0:6*body_idx+3, 0:3] = np.eye(3)
        if ANGULAR_FIRST:
            J[6*body_idx+3:6*body_idx+6, 3:6] = _Jv
            J[6*body_idx+0:6*body_idx+3, 3:6] = _Jw
        else:
            J[6*body_idx+0:6*body_idx+3, 3:6] = _Jv
            J[6*body_idx+3:6*body_idx+6, 3:6] = _Jw

    # jacobian derivative
    joint_global_pos = joint_frames[0][3, :3]
    #TODO: check global velocity
    joint_global_velocity = skel.body(0).world_linear_velocity(skel.joint(0).transform_from_child_body_node()[3, :3])
    joint_global_ang_vel = skel.body(0).world_angular_velocity()

    _dJdqw = np.zeros(3)
    for body_idx in range(body_num):
        eff_pos = skel.body(body_idx).to_world()
        offset = eff_pos - joint_global_pos
        eff_vel = skel.body(body_idx).world_linear_velocity()
        offset_velocity = eff_vel - joint_global_velocity
        _dJdqv = -mm.cross(offset, _dJdqw) - mm.cross(offset_velocity, joint_global_ang_vel)

        if ANGULAR_FIRST:
            dJdq[6*body_idx+3:6*body_idx+6] += _dJdqv
            dJdq[6*body_idx+0:6*body_idx+3] += _dJdqw
        else:
            dJdq[6*body_idx+0:6*body_idx+3] += _dJdqv
            dJdq[6*body_idx+3:6*body_idx+6] += _dJdqw

    ##############################################
    # internal joint
    ##############################################
    for joint_idx in active_joint_index[1:]:
        joint = skel.joint(joint_idx)  # type: pydart.Joint
        parent_joint_index = joint.parent_bodynode.parent_joint.id
        dof_start_index = joint.dofs[0].index_in_skeleton()
        joint_global_pos = joint_frames[joint_idx][3, :3]
        #TODO: check global velocity
        joint_global_velocity = joint.child_bodynode.world_linear_velocity(joint.transform_from_child_body_node()[3, :3])
        joint_global_ang_vel = joint.child_bodynode.world_angular_velocity() - joint.parent_bodynode.world_angular_velocity()

        _Jw = joint_frames[joint_idx][:3, :3]
        _dJdqw = mm.cross(joint.parent_bodynode.world_angular_velocity(), joint_global_ang_vel)

        for body_idx in range(1, body_num):
            if joint_idx in ancestors[body_idx]:
                eff_pos = skel.body(body_idx).to_world()
                eff_vel = skel.body(body_idx).world_linear_velocity()

                # jacobian
                offset = eff_pos - joint_global_pos
                _Jv = -np.dot(mm.getCrossMatrixForm(offset), _Jw)

                if ANGULAR_FIRST:
                    J[6*body_idx+3:6*body_idx+6, dof_start_index+0:dof_start_index+3] = _Jv
                    J[6*body_idx+0:6*body_idx+3, dof_start_index+0:dof_start_index+3] = _Jw
                else:
                    J[6*body_idx+0:6*body_idx+3, dof_start_index+0:dof_start_index+3] = _Jv
                    J[6*body_idx+3:6*body_idx+6, dof_start_index+0:dof_start_index+3] = _Jw

                # jacobian derivatives
                offset_velocity = eff_vel - joint_global_velocity
                _dJdqv = -mm.cross(offset, _dJdqw) - mm.cross(offset_velocity, joint_global_ang_vel)

                if ANGULAR_FIRST:
                    dJdq[6*body_idx+3:6*body_idx+6] += _dJdqv
                    dJdq[6*body_idx+0:6*body_idx+3] += _dJdqw
                else:
                    dJdq[6*body_idx+0:6*body_idx+3] += _dJdqv
                    dJdq[6*body_idx+3:6*body_idx+6] += _dJdqw

    return (J, dJdq)
