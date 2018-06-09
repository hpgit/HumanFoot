import numpy as np
import math
from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.Motion import ysMotion as ym


def footAdjust(posture_ori, touch_body_indices, SEGMENT_FOOT_MAG, SEGMENT_FOOT_RAD, baseHeight=0.):
    """

    :param posture_ori:
    :type posture_ori: ym.JointPosture
    :param touch_body_indices:
    :type touch_body_indices: list[str]
    :param SEGMENT_FOOT_MAG:
    :type SEGMENT_FOOT_MAG: float
    :param SEGMENT_FOOT_RAD:
    :type SEGMENT_FOOT_RAD: float
    :param baseHeight:
    :type baseHeight: float
    :return:
    """
    idDic = dict()
    for i in range(posture_ori.skeleton.getJointNum()):
        idDic[posture_ori.skeleton.getJointName(i)] = i
    nameDic = dict()
    for i in range(posture_ori.skeleton.getJointNum()):
        nameDic[i] = posture_ori.skeleton.getJointName(i)
    # specified
    foot_name = "LeftFoot"
    side_touch_body_indices = [touch_body_idx for touch_body_idx in touch_body_indices if foot_name in nameDic[touch_body_idx]]
    foot_joint_pos = posture_ori.getJointPositionGlobal(idDic[foot_name])
    seg_joint_ori = [posture_ori.getJointOrientationGlobal(touch_body_idx) for touch_body_idx in side_touch_body_indices]
    seg_joint_pos = []
    for touch_body_idx in side_touch_body_indices:
        if nameDic[touch_body_idx] == foot_name+'_foot_0_0':
            seg_joint_pos.append(posture_ori.getJointPositionGlobal(touch_body_idx) - SEGMENT_FOOT_MAG*np.dot(seg_joint_ori, -1.936*mm.unitY()))
        else:
            seg_joint_pos.append(posture_ori.getJointPositionGlobal(touch_body_idx))

    if len(side_touch_body_indices) == 0:
        pass
    elif len(side_touch_body_indices) == 1:
        seg_idx = side_touch_body_indices[0]

        # seg joint y pos to 0
        ankle_to_joint_vec = seg_joint_pos[0] - foot_joint_pos
        joint_vec_rot_axis, temp_angle = mm.getRotAxisAngleFromVectors(ankle_to_joint_vec, -mm.unitY())
        joint_y_to_zero_angle = temp_angle - math.acos((foot_joint_pos[1] - SEGMENT_FOOT_RAD - baseHeight)/np.linalg.norm(ankle_to_joint_vec))
        posture_ori.mulJointOrientationGlobal(idDic[foot_name], mm.exp(joint_vec_rot_axis, joint_y_to_zero_angle))

        ###############################################################
        #TODO:
        ###############################################################
        # rotate seg to parallel ground
        seg_ori = posture_ori.getJointOrientationGlobal(seg_idx)
        posture_ori.mulJointOrientationGlobal(seg_idx, seg_ori.T)

    elif len(side_touch_body_indices) == 2:
        seg_idx = [side_touch_body_indices[0], side_touch_body_indices[1]]

        ankle_to_joint_vecs = [seg_joint_pos[i] - foot_joint_pos for i in range(len(seg_idx))]
        ankle_to_joint_vec = .5*(ankle_to_joint_vecs[0] + ankle_to_joint_vecs[1])
        joint_vec_rot_axis, temp_angle = mm.getRotAxisAngleFromVectors(ankle_to_joint_vec, -mm.unitY())
        joint_y_to_zero_angle = temp_angle - math.acos((foot_joint_pos[1] - SEGMENT_FOOT_RAD - baseHeight)/np.linalg.norm(ankle_to_joint_vec))
        posture_ori.mulJointOrientationGlobal(idDic[foot_name], mm.exp(joint_vec_rot_axis, joint_y_to_zero_angle))






    seg_contact_pos = []
    for touch_body_idx in side_touch_body_indices:
        ith_seg_contact_pos = []
        if nameDic[touch_body_idx] == foot_name+'_foot_0_0_0':
            ith_seg_contact_pos.append(seg_joint_pos)
            ith_seg_contact_pos.append(seg_joint_pos + np.dot(seg_joint_ori, mm.unitZ()))
            ith_seg_contact_pos.append(seg_joint_pos + np.dot(seg_joint_ori, mm.unitX()))
        if nameDic[touch_body_idx] == foot_name+'_foot_0_1_0':
            ith_seg_contact_pos.append(seg_joint_pos + np.dot(seg_joint_ori, np.array([0., 0., 0.])))
            pass
        if nameDic[touch_body_idx] == foot_name+'_foot_0_0':
            pass
        if nameDic[touch_body_idx] == foot_name+'_foot_1_0':
            pass


    # unspecified
    pass
