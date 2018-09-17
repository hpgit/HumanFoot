import numpy as np

from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Math import mmMath as mm
from copy import deepcopy


def get_plane_align_transform(posture_to_be_aligned, posture_base):
    """
    aligning posture in XZ plane
    :type posture_base: ym.JointPosture
    :type posture_to_be_aligned:  ym.JointPosture
    :return:
    """
    base_transform = posture_base.globalTs[0]
    before_transform = posture_to_be_aligned.globalTs[0]

    align_translation = mm.T2p(base_transform) - mm.T2p(before_transform)
    align_translation[1] = 0.

    base_direction = np.dot(mm.T2R(base_transform), mm.unitZ())
    before_direction = np.dot(mm.T2R(before_transform), mm.unitZ())

    base_direction[1] = 0.
    before_direction[1] = 0.

    align_rotation = mm.getSO3FromVectors(before_direction, base_direction)

    return mm.getSE3FromSO3andVec3(align_rotation, align_translation)


def align_posture(posture_to_align, posture_base):
    """

    :type posture_to_align: ym.JointPosture
    :type posture_base: ym.JointPosture
    :return:
    """
    align_transform = get_plane_align_transform(posture_to_align, posture_base)
    posture_to_align.translateByOffset(mm.T2p(align_transform))
    posture_to_align.rotateByOffset(mm.T2R(align_transform))


def align_motion(motion_to_align, posture_base, align_frame=None):
    """

    :type motion_to_align: ym.JointMotion
    :type posture_base: ym.JointPosture
    :type align_frame: int
    :return:
    """
    if align_frame is None:
        align_frame = 0
    align_transform = get_plane_align_transform(motion_to_align[align_frame], posture_base)
    motion_to_align.translateByOffset(mm.T2p(align_transform))
    motion_to_align.rotateTrajectory(mm.T2R(align_transform), motion_to_align[align_frame].rootPos)


def blend_posture(posture_from, posture_to, t):
    """

    :type posture_from: ym.JointPosture
    :type posture_to: ym.JointPosture
    :type t: float
    :return:
    """
    assert 0. <= t <= 1.
    posture_new = deepcopy(posture_from)
    #
    # new_localRs = [np.eye(3) for i in range(len(posture_from.localRs))]
    # for i in range(len(posture_from.localRs)):
    #     assert posture_from.skeleton.getElementName(i) == posture_to.skeleton.getElementName(i)

    q_from = posture_from.get_q()
    q_to = posture_to.get_q()
    posture_new.set_q(ym.dart_q_slerp(t, q_from, q_to))

    return posture_new


def blend(prev_motion, next_motion):
    pass


if __name__ == '__main__':
    from fltk import *
    from PyCommon.modules.Resource import ysMotionLoader as yf
    from PyCommon.modules.GUI import ysSimpleViewer as ysv
    from PyCommon.modules.Renderer import ysRenderer as yr

    bvhFilePath = '../samples/walk_left_90degree.bvh'
    motion0 = yf.readBvhFile(bvhFilePath)[104:]
    bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
    motion1 = yf.readBvhFile(bvhFilePath, 0.01)[104:]

    temp_motion = deepcopy(motion1)
    # align_motion(temp_motion, motion0[0], 104)
    for i in range(20):
        temp_motion[i] = blend_posture(motion0[0], motion1[0], i/20.)

    viewer = ysv.SimpleViewer()
    viewer.record(False)
    viewer.doc.addRenderer('motion0', yr.JointMotionRenderer(motion0, (0,0,255), yr.LINK_LINE))
    viewer.doc.addObject('motion0', motion0)
    viewer.doc.addRenderer('motion1', yr.JointMotionRenderer(motion1, (0,255,0), yr.LINK_LINE))
    viewer.doc.addObject('motion1', motion1)
    viewer.doc.addRenderer('stitched_motion', yr.JointMotionRenderer(temp_motion, (255,255,0), yr.LINK_LINE))
    viewer.doc.addObject('stitched_motion', temp_motion)

    viewer.startTimer(1/30.)

    viewer.show()

    Fl.run()
