import numpy as np
from copy import deepcopy
from enum import Enum, auto
import random

from PyCommon.modules.Math.mmMath import PlaneProject


class BufferType(Enum):
    FC_P_WARP = auto()
    FC_PV_WARP = auto()
    FC_PVC_WARP = auto()
    FC_NO_WARP = auto()


class ControlSchemeType(Enum):
    FC_RANDOM_SELECTION = auto()
    FC_PROBABILITY_SELECTION = auto()
    FC_LINEAR = auto()
    FC_SEARCH = auto()


class FlowState:
    def __init__(self):
        self.node = 0
        self.weight = 0.
        self.depth = 0
        self.branch = 0
        self.location = np.eye(4)
        self.distance = 0.
        self.path = list()  # type: list[int]

    def __gt__(self, other):
        assert isinstance(other, FlowState)
        return self.weight > other.weight

    def copy(self, b):
        """

        :param b:
        :type b: FlowState
        :return:
        """
        self.node = b.node
        self.weight = b.weight
        self.depth = b.depth
        self.branch = b.branch
        self.location = deepcopy(b.location)
        self.distance = b.distance
        self.path = deepcopy(b.path)


class FlowGenerate:
    def __init__(self):
        self.graph = None  # type: FlowGraph
        self.time = 0
        self.human = None  # type: PmHuman

        # current state
        self.posture = None  # type: PmPosture
        self.velocity = None  # type: PmVector

        # warp buffer
        self.buffer = None  # type: PmLinearMotion
        self.buffer_len = 0
        self.warpCount = 0

        self.alignment_t = np.eye(4)

        # constraints
        self.leftFootContact = False
        self.rightFootContact = False

        self.leftFootFrame = 0
        self.rightFootFrame = 0

        self.leftFootTransf = np.eye(4)
        self.rightFootTransf = np.eye(4)

        # control scheme
        self.buffer_type = BufferType.FC_P_WARP
        self.control_scheme = ControlSchemeType.FC_RANDOM_SELECTION

        self.targetPosture = None  # type: PmPosture

    def init(self, h, f):
        self.graph = f
        self.human = h
        self.time = 0

        self.warpCount = self.buffer_len

        self.posture.setBody( h )
        self.buffer.setBody( h )

        self.control_scheme = ControlSchemeType.FC_PROBABILITY_SELECTION

    def step(self):
        t_old = self.getTime()
        self.setTime(self.getNextFrame(t_old))

        p = self.updateWarpBuffer(t_old, self.getTime())

        self.updateFoorConstraints(p)
        self.updatePosture(p)

    def updateWarpBuffer(self, t_old, t_new):
        if t_new != t_old + 1:
            self.makeBuffer(self.getPosture(), t_old, t_new)
            self.resetWarpCount()

        warp_count = self.tickWarpCount()

        #TODO:
        # is p member variable?
        p = None

        if warp_count < self.getBufferLength():
            p = self.getBuffer(warp_count)
        elif self.graph.local_coordinate:
            p = self.graph.getFrame(t_new)
            self.graph.getNextPosture(self.getPosture(), p, t_new)
        else:
            p = self.graph.getFrame(t_new)

        return p

    def updateFootConstraints(self, p):
        c = self.graph.getConstraint(self.getTime())

        if self.buffer_type == BufferType.FC_PVC_WARP:
            if self.leftFootContact:
                self.leftFootFrame += 1
                self.leftFootContact = c.isConstrained(LEFT_FOOT) and self.graph.getConstraint(self.leftFootFrame).isConstrained(LEFT_FOOT)
            elif c.isConstrained(LEFT_FOOT):
                self.leftFootContact = True
                self.leftFootFrame = self.getTime()

                t1 = p.getGlobalTrnasf(LEFT_FOOT)
                t2 = self.graph.getFrame(self.leftFootFrame).getGlobalTransf(LEFT_FOOT)

                t1 = PlaneProject(t1)
                t2 = PlaneProject(t2)

                self.leftFootTransf = t2.inverse() * t1

            if self.rightFootContact:
                self.rightFootFrame += 1
                self.rightFootContact = c.isConstrained(RIGHT_FOOT) and self.graph.getConstraint(self.rightFootFrame).isConstrained(RIGHT_FOOT)
            elif c.isConstrained(RIGHT_FOOT):
                self.rightFootContact = True
                self.rightFootFrame = self.getTime()

                t1 = p.getGlobalTrnasf(RIGHT_FOOT)
                t2 = self.graph.getFrame(self.rightFootFrame).getGlobalTransf(RIGHT_FOOT)

                t1 = PlaneProject(t1)
                t2 = PlaneProject(t2)

                self.leftFootTransf = t2.inverse() * t1

    def setTime(self, t):
        self.time = t

    def getTime(self):
        return self.time

    def getHuman(self):
        return self.human

    def getPosture(self):
        return self.posture

    def setVelocity(self):
        return self.velocity

    def updatePosture(self, p):
        self.velocity.diffrence(p, self.posture)
        self.posture = p

    def getNextFrame(self, t_cur):
        if self.control_scheme == ControlSchemeType.FC_RANDOM_SELECTION:
            # random selection without probability
            d = random.randrange(0, self.graph.flow_graph[t_cur].num)
            e = self.graph.flow_graph[t_cur].entity
            i = 0
            while i < d:
                i = i+1
                e = e.next
            return e.id

        elif self.control_scheme == ControlSchemeType.FC_PROBABILITY_SELECTION:
            # random selection with probability
            r = random.random()
            sum = 0.
            i = 0

            for _i in range(self.graph.getSize()):
                sum += self.graph.getValue(t_cur, _i)
                if r < sum:
                    i = _i
                    break

            if i == self.graph.getSize():
                i -= 1

            return i

        elif self.control_scheme == ControlSchemeType.FC_LINEAR:
            # sequential playback
            return t_cur + 1

        elif self.control_scheme == ControlSchemeType.FC_SEARCH:
            i = self.searchPath(t_cur, self.getPosture(), self.targetPosture)
            return i

    def getControlScheme(self):
        return self.control_scheme

    def getBufferType(self):
        return self.buffer_type

    def setControlScheme(self, t):
        self.control_scheme = t

    def setBufferType(self, t):
        self.buffer_type = t

    def setStartPosture(self, t):
        self.time = t

        self.leftFootContact = self.graph.getConstraint(t).isConstrained( LEFT_FOOT)
        self.rightFootContact = self.graph.getConstraint(t).isConstrained( RIGHT_FOOT)

        self.leftFootFrame = t
        self.rightFootFrame = t

        self.alignment_t = np.eye(4)

        self.updatePosture( self.graph.getFrame(self.time) )
        self.updatePosture( self.graph.getFrame(self.time) )

        self.leftFootTransf = self.alignment_t
        self.rightFootTransf = self.alignment_t

        self.removeBuffer()

    def applyTransf(self, t):
        self.posture.applyTrnasf(t)

    def getWarpCount(self):
        return self.warpCount

    def tickWarpCount(self):
        if self.warpCount < self.buffer_len:
            self.warpCount += 1
        return self.warpCount

    def resetWarpCount(self):
        self.warpCount = 0

    def removeBuffer(self):
        self.warpCount = self.buffer_len

    def makeBuffer(self, cur_pst, cur_frame, start_frame):
        calib = np.eye(4)
        pst = static PmPosture()

        t1 = cur_pst.getGlobalTransf(PELVIS)
        t2 = self.graph.getFrame(start_frame-1).getGlobalTransf(PELVIS)

        t1 = PlaneProject(t1)
        t2 = PlaneProject(t2)

        if self.graph.local_coordinate:
            calib = t2.inverse() * t1

            if self.graph.angle_fixed:
                calib = translate_transf(calib.getTranslation())
        else:
            calib = np.eye(4)

        lfc = self.leftFootContact
        lff = self.leftFootFrame - 1
        lfd = False

        rfc = self.rightFootContact
        rff = self.rightFootFrame - 1
        rfd = False

        lft = self.leftFootTransf.copy()
        rft = self.rightFootTransf.copy()

        for i in range(self.buffer_len):
            pst = self.graph.getFrame(start_frame + i - 1)
            pst.applyTransf(calib)
            self.buffer.setPosture(i, pst)

            if self.buffer_type == BufferType.FC_PVC_WARP:
                c = self.graph.getConstraint(start_frame + i - 1)

                if lfc:
                    lff += 1
                    lfc = c.isConstrained(LEFT_FOOT) and self.graph.getConstraint(lff).isConstrained(LEFT_FOOT)
                elif c.isConstrained(LEFT_FOOT):
                    lfc = True
                    lff = start_frame + i - 1

                    t1 = pst.getGlobalTransf(LEFT_FOOT)
                    t2 = self.graph.getFrame(lff).getGlobalTransf(LEFT_FOOT)

                    t1 = PlaneProject(t1)
                    t2 = PlaneProject(t2)

                    lft = t2.inverse() * t1

                if rfc:
                    rff += 1
                    rfc = c.isConstrained(RIGHT_FOOT) and self.graph.getConstraint(rff).isConstrained(RIGHT_FOOT)
                elif c.isConstrained(RIGHT_FOOT):
                    rfc = True
                    rff = start_frame + i - 1

                    t1 = pst.getGlobalTransf(RIGHT_FOOT)
                    t2 = self.graph.getFrame(rff).getGlobalTransf(RIGHT_FOOT)

                    t1 = PlaneProject(t1)
                    t2 = PlaneProject(t2)

                    rft = t2.inverse() * t1

                if i > 0 and not c.isConstrained(LEFT_FOOT):
                    lfd = True
                if i > 0 and not c.isConstrained(RIGHT_FOOT):
                    lfc = True

                cf = self.buffer.constraints[i]
                cf.reset()

                if lfc:
                    if not lfd:
                        t2 = self.graph.getFrame(lff).getGlobalTransf(LEFT_FOOT)
                        cf.push(LEFT_FOOT, t2 * lft)
                    else:
                        cf.push(LEFT_FOOT, pst.getGlobalTransf(LEFT_FOOT))

                if rfc:
                    if not rfd:
                        t2 = self.graph.getFrame(rff).getGlobalTransf(RIGHT_FOOT)
                        cf.push(RIGHT_FOOT, t2 * rft)
                    else:
                        cf.push(RIGHT_FOOT, pst.getGlobalTransf(RIGHT_FOOT))

        if self.buffer_type != BufferType.FC_NO_WARP:
            self.buffer.directManip(0, cur_pst, None, self.buffer.getSize(), 1, False, False)

        if self.buffer_type == BufferType.FC_PVC_WARP:
            bound = 2

            for i in range(PM_HUMAN_NUM_LINKS):
                j = 0
                while j < bound:
                    if self.buffer.constraints[j].isConstrained(i):
                        t1 = self.buffer.getPosture(j).getGlobalTransf(i)
                        t2 = self.buffer.constraints[j].getTransf(i)

                        t1 = PlaneProject(t1)
                        t2 = PlaneProject(t2)

                        t1 = t2.inverse() * t1

                        while j < self.buffer.getSize():
                            if not self.buffer.constraints[j].isConstrained(i)
                                break
                            self.buffer.constraints[j].applyTransf(i, t1)

                        break

            self.buffer.directManip(0, cur_pst, self.buffer.constraints, self.buffer.getSize(), 3, True, False)

    def getBufferLength(self):
        return self.buffer_len

    def setBufferLength(self, l):
        self.buffer_len = l
        self.buffer.extend(self.buffer_len)

    def getBuffer(self, i=None):
        if i is None:
            return self.buffer
        else:
            return self.buffer.getPosture(i)

    def getFrames(self, m, start_posture, n, index):
        self.setStartPosture(index[0])
        self.updatePosture(start_posture)
        self.updatePosture(start_posture)  # to initialize the velocity

        m.setSize(n)
        m.setPosture(0, start_posture)

        for i in range(1, n-1):
            self.setTime(index[i])

            p = static pmPosture
            p = self.updateWarpBuffer(index[i-1], self.getTime())

            self.updateFootConstraints(p)
            self.updatePosture(p)

            c = PmConstrint()
            c.reset()
            if self.leftFootContact:
                c.push(LEFT_FOOT, self.leftFootTransf)
            if self.rightFootContact:
                c.push(RIGHT_FOOT, self.rightFootTransf)

            m.setPosture(i, self.posture)
            m.setConstraint(i, c)


    def getActionSequences(self, m, a, numActions, index):
        numFrames = 0
        for i in range(numActions):
            numFrames += a[index[i]].getLength()

        findex = [0] * numFrames

        f_count = 0
        for i in range(numActions):
            for j in range(a[index[i]].getLength()):
                findex[f_count] = a[index[i]].start + j
                f_count += 1

        self.getFrames(m, self.graph.getFrame(findex[0]), numFrames, findex)

    # generate motion incrementally
    def generateMotion(self, n, seed, start_posture, m):
        self.setStartPosture(seed)
        self.updatePosture(start_posture)

        m.setSize(n)
        m.setPosture(0, self.posture)

        for i in range(1, n):
            self.step()

            c = PmConstraint()
            c.reset()
            if self.leftFootContact:
                c.push(LEFT_FOOT, self.leftFootTransf)
            if self.rightFootContact:
                c.push(RIGHT_FOOT, self.rightFootTransf)

            m.setPosture(i, self.posture)
            m.setConstraint(i, c)

    def searchPath(self, int, posture, posture):
        pass




































