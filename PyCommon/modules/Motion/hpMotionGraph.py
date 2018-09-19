import numpy as np

from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Motion import hpMotionBlend as hmb
from PyCommon.modules.Math import mmMath as mm
from annoy import AnnoyIndex


def joint_poses_btw_posture_in_plane_by_joint_pos(posture, posture_base):
    """

    :type posture: ym.JointPosture
    :type posture_base: ym.JointPosture
    :return:
    """

    align_transform = hmb.get_plane_align_transform_by_posture(posture, posture_base)
    position = [mm.affine_pos(align_transform, posture.getPosition(i)) for i in range(1, posture.skeleton.getElementNum())]
    position.insert(0, np.array((posture.getPosition(0)[1], )))

    return np.concatenate(position)


BLEND_FRAME = 10


class MotionTransition(object):
    def __init__(self):
        self.motion_to = 0
        self.motion_to_idx = 0
        self.motion_from = 0
        self.motion_from_idx = 0


class MotionGraph(object):
    def __init__(self):
        self.is_built = False
        self.transition = []  # type: list[MotionTransition]
        self.motions = []  # type: list[ym.JointMotion]

    def add_transition(self, _transition):
        self.transition.append(_transition)

    def generate_motion(self, start_motion_idx, start_motion_time_offset, motion_time):
        pass

    def add_motion(self):
        pass

    def build(self):
        self.init()
        self.find_nn()
        self.prune_contact()
        self.prune_likelihood()
        self.prune_local_maxima()
        self.prune_dead_end()

        self.is_built = True

    def init(self):
        """
        init transition data structure
        :return:
        """
        pass

    def find_nn(self):
        pass

    def prune_contact(self):
        pass

    def prune_likelihood(self):
        pass

    def prune_local_maxima(self):
        pass

    def prune_dead_end(self):
        pass

