from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Motion import hpMotionBlend as hmb


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

