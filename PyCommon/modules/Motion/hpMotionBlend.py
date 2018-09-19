import numpy as np

from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.Math import ysFunctionGraph as yfg
from copy import deepcopy


def get_plane_align_transform_by_posture(posture_to_be_aligned, posture_base):
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
    align_transform = get_plane_align_transform_by_posture(posture_to_align, posture_base)
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
    align_transform = get_plane_align_transform_by_posture(motion_to_align[align_frame], posture_base)
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
    # posture_new = deepcopy(posture_from)
    posture_new = ym.JointPosture(posture_from.skeleton)
    posture_new.globalTs = deepcopy(posture_from.globalTs)
    posture_new.localRs = deepcopy(posture_from.localRs)
    posture_new.rootPos = deepcopy(posture_from.rootPos)

    # TODO: different skeleton?
    #
    # new_localRs = [np.eye(3) for i in range(len(posture_from.localRs))]
    # for i in range(len(posture_from.localRs)):
    #     assert posture_from.skeleton.getElementName(i) == posture_to.skeleton.getElementName(i)

    q_from = posture_from.get_q()
    q_to = posture_to.get_q()
    posture_new.set_q(ym.dart_q_slerp(t, q_from, q_to))

    return posture_new



def blend_motion(prev_motion, next_motion, prev_blend_start, prev_blend_end, next_blend_start, next_blend_end):
    """

    :type prev_motion: ym.JointMotion
    :type next_motion: ym.JointMotion
    :type prev_blend_start: int
    :type prev_blend_end: int
    :type next_blend_start: int
    :type next_blend_end: int
    :return:
    """
    # assume that fps is same
    # 1. time scaling
    prev_frame_num = prev_blend_end - prev_blend_start + 1
    next_frame_num = next_blend_end - next_blend_start + 1
    blend_frame_num = (prev_frame_num + next_frame_num) // 2

    motion_new = deepcopy(prev_motion[prev_blend_start:prev_blend_end+1]) if prev_frame_num > next_frame_num \
        else deepcopy(next_motion[next_blend_start:next_blend_end+1])
    motion_new = prev_motion[:prev_blend_start] + motion_new[0:blend_frame_num] + next_motion[next_blend_end+1:]

    # 2. blending frame by frame
    for i in range(blend_frame_num):
        # uniform sampling
        # T = i / blend_frame_num

        # non-uniform sampling
        t = i / blend_frame_num
        T = yfg.H1(t) + next_frame_num/prev_frame_num * yfg.H2(t) + prev_frame_num/next_frame_num * yfg.H3(t)
        prev_q = prev_motion.get_q_by_time((prev_blend_start + T*prev_frame_num)/prev_motion.fps)
        next_q = next_motion.get_q_by_time((next_blend_start + T*next_frame_num)/next_motion.fps)
        s = .5 - .5*math.cos(math.pi*T)
        motion_new[prev_blend_start+i].set_q(ym.dart_q_slerp(s, prev_q, next_q))

    # TODO:
    # 3. fix foot skating

    return motion_new


if __name__ == '__main__':
    import math
    from fltk import *
    from PyCommon.modules.Resource import ysMotionLoader as yf
    from PyCommon.modules.GUI import ysSimpleViewer as ysv
    from PyCommon.modules.Renderer import ysRenderer as yr

    def test_blend_posture():
        bvhFilePath = '../samples/walk_left_90degree.bvh'
        motion0 = yf.readBvhFile(bvhFilePath)[104:]
        bvhFilePath = '../samples/walk_left_90degree.bvh'
        motion1 = yf.readBvhFile(bvhFilePath)[0:]

        align_motion(motion1, motion0[0], 0)
        temp_motion = deepcopy(motion1)
        for i in range(21):
            temp_motion[i] = blend_posture(motion0[0], motion1[0], .5 - .5*math.cos(i/20.*math.pi))
            # temp_motion[i] = blend_posture(motion0[0], motion1[0], i/20.)
        for i in range(21, len(temp_motion)):
            temp_motion[i] = temp_motion[20]

        viewer = ysv.SimpleViewer(rect=(0, 0, 1280, 900))
        viewer.record(False)
        viewer.doc.addRenderer('motion0', yr.JointMotionRenderer(motion0, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion0', motion0)
        viewer.doc.addRenderer('motion1', yr.JointMotionRenderer(motion1, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion1', motion1)
        viewer.doc.addRenderer('stitched_motion', yr.JointMotionRenderer(temp_motion, (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('stitched_motion', temp_motion)

        def pre_callback(frame):
            motion0.goToFrame(0)
            motion1.goToFrame(0)

        viewer.setPreFrameCallback_Always(pre_callback)

        viewer.startTimer(1/30.)
        viewer.show()

        Fl.run()

    def test_blend_motion():
        # bvhFilePath = '../samples/wd2_WalkForwardVFast00.bvh'
        bvhFilePath = '../samples/wd2_WalkSameSame00.bvh'
        motion0 = yf.readBvhFile(bvhFilePath, 0.01)
        bvhFilePath = '../samples/wd2_WalkSukiko00.bvh'
        motion1 = yf.readBvhFile(bvhFilePath, 0.01)

        # VFast to Sukiko
        # align_motion(motion1, motion0[54], 121)
        # temp_motion = blend_motion(motion0, motion1, 54, 82, 121, 180)
        # SameSame to Sukiko
        # align_motion(motion1, motion0[76], 121)
        # temp_motion = blend_motion(motion0, motion1, 76, 116, 121, 180)

        # Sukiko to SameSame
        align_motion(motion0, motion1[121], 76)
        temp_motion = blend_motion(motion1, motion0, 121, 180, 76, 116)

        viewer = ysv.SimpleViewer(rect=(0, 0, 1280, 900))
        viewer.record(False)
        viewer.doc.addRenderer('motion0', yr.JointMotionRenderer(motion0, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion0', motion0)
        # viewer.doc.setRendererVisible('motion0', False)
        viewer.doc.addRenderer('motion1', yr.JointMotionRenderer(motion1, (0,255,0), yr.LINK_LINE))
        viewer.doc.addObject('motion1', motion1)
        # viewer.doc.setRendererVisible('motion1', False)
        viewer.doc.addRenderer('stitched_motion', yr.JointMotionRenderer(temp_motion, (255,255,0), yr.LINK_LINE))
        viewer.doc.addObject('stitched_motion', temp_motion)
        # viewer.doc.setRendererVisible('stitched_motion', True)

        def pre_callback(frame):
            # VFast to Sukiko
            # motion0.goToFrame(54)
            # motion1.goToFrame(180)
            # SameSame to Sukiko
            # motion0.goToFrame(76)
            # motion1.goToFrame(180)

            # Sukiko to SameSame
            motion0.goToFrame(116)
            motion1.goToFrame(121)
        motion0.goToFrame(116)
        motion1.goToFrame(121)

        viewer.setPreFrameCallback_Always(pre_callback)

        viewer.startTimer(1/30.)
        viewer.show()

        Fl.run()

    def test_motion():
        # bvhFilePath = '../samples/walk_left_90degree.bvh'
        # bvhFilePath = '../samples/wd2_cross_walk_0d_01.bvh'
        # bvhFilePath = '../samples/wd2_cross_walk_0d_fast_21.bvh'

        bvhFilePath = '../samples/wd2_WalkAzuma01.bvh'
        # bvhFilePath = '../samples/wd2_WalkBackward00.bvh'
        # bvhFilePath = '../samples/wd2_WalkForwardNormal00.bvh'
        # bvhFilePath = '../samples/wd2_WalkForwardSlow01.bvh'
        # bvhFilePath = '../samples/wd2_WalkForwardVFast00.bvh'
        # bvhFilePath = '../samples/wd2_WalkHandWav00.bvh'
        # bvhFilePath = '../samples/wd2_WalkSameSame01.bvh'
        # bvhFilePath = '../samples/wd2_WalkSoldier00.bvh'
        motion0 = yf.readBvhFile(bvhFilePath)

        viewer = ysv.SimpleViewer(rect=(0, 0, 1280, 900))
        viewer.record(False)
        viewer.doc.addRenderer('motion0', yr.JointMotionRenderer(motion0, (0,0,255), yr.LINK_LINE))
        viewer.doc.addObject('motion0', motion0)

        viewer.startTimer(1/30.)
        viewer.show()

        Fl.run()
        pass

    # test_blend_posture()
    # test_blend_motion()
    test_motion()
