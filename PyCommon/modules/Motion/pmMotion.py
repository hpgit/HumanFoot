from PyCommon.modules.Motion.ysMotion import *
from PyCommon.modules.Motion.pmConstants import *
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

    def getPositionVelocity(self, i, v):
        """

        :param i:
        :type i: int
        :param v:
        :type v: PmVector
        :return:
        """
        raise NotImplementedError


class PmPosture(JointPosture):
    def __init__(self, skeleton):
        super(PmPosture, self).__init__(skeleton)
        self.skeleton = skeleton  # type: PmHuman

        # pmqm
        self.mask = PmHumanEnum.UNDEFINED
        self.mask |= MaskBit(PmHumanEnum.PELVIS)
        self.mask |= MaskBit(PmHumanEnum.CHEST)
        self.mask |= MaskBit(PmHumanEnum.NECK)
        self.mask |= MaskBit(PmHumanEnum.UPPER_RIGHT_ARM)
        self.mask |= MaskBit(PmHumanEnum.UPPER_LEFT_ARM)
        self.mask |= MaskBit(PmHumanEnum.LOWER_RIGHT_ARM)
        self.mask |= MaskBit(PmHumanEnum.LOWER_LEFT_ARM)
        self.mask |= MaskBit(PmHumanEnum.UPPER_RIGHT_LEG)
        self.mask |= MaskBit(PmHumanEnum.UPPER_LEFT_LEG)
        self.mask |= MaskBit(PmHumanEnum.LOWER_RIGHT_LEG)
        self.mask |= MaskBit(PmHumanEnum.LOWER_LEFT_LEG)
        self.mask |= MaskBit(PmHumanEnum.RIGHT_FOOT)
        self.mask |= MaskBit(PmHumanEnum.LEFT_FOOT)
        self.mask |= MaskBit(PmHumanEnum.RIGHT_TOE)
        self.mask |= MaskBit(PmHumanEnum.LEFT_TOE)

    def getGlobalTransf(self, i):
        return self.globalTs[i]

    def getGlobalPosition(self, i):
        return mm.T2p(self.globalTs[i])

    def getMask(self):
        return self.mask


class PmHuman(JointSkeleton):
    pass


class PmVectorArray:
    def __init__(self, joint_num):
        self.data = []  # type: list[PmVector]
        self.joint_num = joint_num

    def setSize(self, size):
        for i in range(size):
            self.data.append(PmVector(self.joint_num))

    def setVector(self, j, v):
        self.data[j] = deepcopy(v)


class PmVector:
    def __init__(self, joint_num):
        # pmqm
        self.mask = PmHumanEnum.UNDEFINED
        self.mask |= MaskBit(PmHumanEnum.PELVIS)
        self.mask |= MaskBit(PmHumanEnum.CHEST)
        self.mask |= MaskBit(PmHumanEnum.NECK)
        self.mask |= MaskBit(PmHumanEnum.UPPER_RIGHT_ARM)
        self.mask |= MaskBit(PmHumanEnum.UPPER_LEFT_ARM)
        self.mask |= MaskBit(PmHumanEnum.LOWER_RIGHT_ARM)
        self.mask |= MaskBit(PmHumanEnum.LOWER_LEFT_ARM)
        self.mask |= MaskBit(PmHumanEnum.UPPER_RIGHT_LEG)
        self.mask |= MaskBit(PmHumanEnum.UPPER_LEFT_LEG)
        self.mask |= MaskBit(PmHumanEnum.LOWER_RIGHT_LEG)
        self.mask |= MaskBit(PmHumanEnum.LOWER_LEFT_LEG)
        self.mask |= MaskBit(PmHumanEnum.RIGHT_FOOT)
        self.mask |= MaskBit(PmHumanEnum.LEFT_FOOT)
        self.mask |= MaskBit(PmHumanEnum.RIGHT_TOE)
        self.mask |= MaskBit(PmHumanEnum.LEFT_TOE)

        self.linear = np.zeros(3)
        self.angular = [np.zeros(3) for i in range(joint_num)]

    def getMask(self):
        return self.mask

    def positionDifference(self, p1, p2):
        """

        :param p1:
        :type p1: PmPosture
        :param p2:
        :type p2: PmPosture
        :return:
        """
        self.mask = p1.getMask() & p2.getMask()

        t1 = mm.PlaneProject(p1.getTransf(0))
        t2 = mm.PlaneProject(p2.getTransf(0))

        calib = mm.invertSE3(t2) * t1

        self.setLinearVector(np.zeros(3))

        for i in range(PM_HUMAN_NUM_LINKS):
            if self.getMask() & MaskBit(i):
                a1 = (np.zeros(3) + p1.getGlobalTranslation(i))
                a2 = (np.zeros(3) + p2.getGlobalTranslation(i)) * calib

                self.setAngularVector(i, a2 - a1)

        return self

