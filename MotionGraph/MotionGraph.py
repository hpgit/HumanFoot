
class MotionTransition(object):
    def __init__(self):
        self._to = 0

    @property
    def to(self):
        return self.to

    @to.setter
    def to(self, value):
        self._to = value

class MotionGraph(object):
    def __init__(self):
        self.is_built = False
        self.transition = False

    def generate_motion(self, start_motion_idx, start_motion_time_offset, motion_time):
        pass


