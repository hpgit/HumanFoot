from PyCommon.modules.Motion.ysMotion import *
from copy import deepcopy


class PmLinearMotion(JointMotion):
    def __add__(self, nextMotion):
        """

        :param nextMotion:
        :type nextMotion: PmLinearMotionMotion
        :return: PmLinearMotion
        """
        if isinstance(nextMotion, PmLinearMotion):
            motion = PmLinearMotion(self.data)
            motion.data.extend(nextMotion.data)
            motion.fps = self.fps
            motion.resourceName = self.resourceName
            return motion
        else:
            raise TypeError

    def getSize(self):
        return len(self)

    def getPosture(self, i):
        """

        :param i:
        :return:
        :rtype: PmPosture
        """
        return self[i]

    def getMask(self):
        mask = 0x00
        for i in range(self.getSize()):
            mask |= self.getPosture(i).getMask()

        return mask


class PmPosture(JointPosture):
    def __init__(self, skeleton):
        super(PmPosture, self).__init__(skeleton)
        self.skeleton = skeleton  # type: PmHuman

        # pmqm
        self.mask = 0x00
        self.mask |= 0x01 << 0
        self.mask |= 0x01 << 5
        self.mask |= 0x01 << 6
        self.mask |= 0x01 << 12
        self.mask |= 0x01 << 13
        self.mask |= 0x01 << 14
        self.mask |= 0x01 << 15
        self.mask |= 0x01 << 16
        self.mask |= 0x01 << 17
        self.mask |= 0x01 << 18
        self.mask |= 0x01 << 19
        self.mask |= 0x01 << 20
        self.mask |= 0x01 << 21
        self.mask |= 0x01 << 22
        self.mask |= 0x01 << 23

    def getGlobalTransf(self, i):
        return self.globalTs[i]

    def getGlobalPosition(self, i):
        return mm.T2p(self.globalTs[i])

    def getMask(self):
        return self.mask


class PmHuman(JointSkeleton):
    pass

