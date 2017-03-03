import numpy as np
from numpy.linalg import inv
import sys
if ".." not in sys.path:
    sys.path.append("..")
from PyCommon.modules import pydart2 as pydart
from PyCommon.modules.Math import mmMath as mm
import math

class PDController:
    """
    :type h : float
    :type skel : pydart.Skeleton
    :type qhat : np.array
    """
    def __init__(self, skel, h):
        self.h = h
        self.skel = skel
        ndofs = self.skel.ndofs
        # self.qhat = self.skel.q
        Kt = 1000.
        Dt = 2.*(Kt**.5)
        self.Kp = np.diagflat([0.0] * 6 + [400.0] * (ndofs - 6))
        self.Kd = np.diagflat([0.0] * 6 + [20.0] * (ndofs - 6))
        # self.Kp = np.diagflat([0.0] * 6 + [Kt] * (ndofs - 6))
        # self.Kd = np.diagflat([0.0] * 6 + [Dt] * (ndofs - 6))
        self.preoffset = 0.0

        self.Rs = None
        self.vel = None

    def setKpKd(self, index, Kp, Kd):
        self.Kp[index, index] = Kp
        self.Kd[index, index] = Kd

    def compute(self):
        skel = self.skel
        deltaq = self.calcDeltaq()

        invM = inv(skel.M + self.Kd * self.h)
        # p = -self.Kp.dot(skel.q + skel.dq * self.h - self.qhat)
        p = -self.Kp.dot(-deltaq + skel.dq * self.h)
        d = -self.Kd.dot(skel.dq)
        qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
        tau = p + d - self.Kd.dot(qddot) * self.h

        '''
        # Check the balance
        COP = skel.body('h_heel_left').to_world([0.05, 0, 0])
        offset = skel.C[0] - COP[0]

        # Adjust the target pose
        k1 = 200.0 if 0.0 < offset and offset < 0.1 else 2000.0
        k2 = 100.0
        kd = 10.0 if 0.0 < offset and offset < 0.1 else 100.0
        q_delta1 = np.array([-k1, -k2, -k1, -k2]) * offset
        q_delta2 = np.ones(4) * kd * (self.preoffset - offset)
        tau[np.array([17, 25, 19, 26])] += (q_delta1 + q_delta2)
        self.preoffset = offset
        #'''

        # Make sure the first six are zero
        tau[:6] = 0
        # return qddot
        # return p+d
        return tau

    def setTartgetPose(self, Rs):
        self.Rs = Rs

    def setTargetVel(self, vel):
        self.vel = vel

    def calcDeltaq(self):
        deltaq = np.zeros(self.skel.q.shape)
        if self.Rs is not None:
            p_r0 = self.Rs[0][0]
            p0 = self.skel.q[3:6]

            th_r0 = self.Rs[0][1]
            th0 = mm.exp(self.skel.q[:3])

            deltaq[:6] = np.hstack((mm.logSO3(np.dot(th0.transpose(), th_r0)), p_r0 - p0))
            # TODO:
            # apply variety dofs

            dofOffset = 6
            for i in range(1, len(self.skel.joints)):
                # for i in range(1, len(self.Rs)):
                joint = self.skel.joints[i]
                if joint.num_dofs() == 3:
                    deltaq[dofOffset:dofOffset+3] = mm.logSO3(np.dot(joint.get_local_transform()[:3, :3].transpose(), self.Rs[i]))
                elif joint.num_dofs() == 2:
                    targetAngle1 = math.atan2(-self.Rs[i][1,2], self.Rs[i][2,2])
                    targetAngle2 = math.atan2(-self.Rs[i][0,1], self.Rs[i][0,0])
                    deltaq[dofOffset:dofOffset+2] = np.array([targetAngle1, targetAngle2])
                elif joint.num_dofs() == 1:
                    deltaq[dofOffset] = math.atan2(self.Rs[i][2, 1], self.Rs[i][1,1])
                dofOffset += joint.num_dofs()

            # a_des0 = kt*(p_r0 - p0) + dt*(- v0) #+ a_r0
            # ddth_des0 = kt*(mm.logSO3(np.dot(th0.transpose(), th_r0))) + dt*(- dth0) #+ ddth_r0

        return deltaq

    def calcDeltadq(self):
        deltadq = np.zeros(self.skel.dq.shape)
        if self.vel is not None:

            dth0
            deltadq[:6] = np.hstack(())
