import numpy as np
from numpy.linalg import inv
# import pydart
from PyCommon.modules import pydart2 as pydart
from PyCommon.modules.Math import mmMath as mm


class PDController:
    """
    :type h : float
    :type skel : pydart.Skeleton
    :type qhat : np.array
    """
    def __init__(self, model, skel, h, Kp, Kd):
        self.h = h
        self.skel = skel
        self.model = model
        ndofs = self.skel.ndofs
        # self.qhat = self.skel.q
        # self.Kp = np.diagflat([0.0] * 6 + [Kp] * (ndofs - 6))
        # self.Kd = np.diagflat([0.0] * 6 + [Kd] * (ndofs - 6))
        self.Kp = np.diagflat([Kp] * ndofs)
        self.Kd = np.diagflat([Kd] * ndofs)
        self.preoffset = 0.0

    def setKpKd(self, Kp, Kd):
        ndofs = self.skel.ndofs
        self.Kp = np.diagflat([Kp] * ndofs)
        self.Kd = np.diagflat([Kd] * ndofs)

    def compute(self, positions_hat, weightMap=None):
        skel = self.skel

        # invM = inv(skel.M + self.Kd * self.h)
        # p = -self.Kp.dot(skel.q + skel.dq * self.h - qhat)
        # d = -self.Kd.dot(skel.dq)
        # qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
        # tau = p + d - self.Kd.dot(qddot) * self.h

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
        th_r = positions_hat
        ddth_des = [None] * len(th_r)
        th = self.model.getDOFPositions()
        dth = self.model.getDOFVelocities()
        p_r0 = th_r[0][0]
        p0 = th[0][0]
        v0 = dth[0][:3]

        th_r0 = th_r[0][1]
        th0 = th[0][1]
        dth0 = dth[0][3:]

        kt = self.Kp[0, 0]
        dt = self.Kd[0, 0]

        if weightMap is not None:
            kt = Kt * weightMap[0]
            dt = Dt * (weightMap[0]**.5)
            # dt = 0.
        a_des0 = kt*(p_r0 - p0) - dt * v0
        ddth_des0 = kt*(mm.logSO3(np.dot(th0.transpose(), th_r0))) - dt*dth0
        ddth_des[0] = np.concatenate((a_des0, ddth_des0))

        for i in range(1, len(th_r)):
            if weightMap is not None:
                kt = Kt * weightMap[i]
                dt = Dt * (weightMap[i]**.5)
                # dt = 0.
            ddth_des[i] = kt*(mm.logSO3(np.dot(th[i].transpose(), th_r[i]))) - dt*dth[i]
            # ddth_des[i] = kt*(mm.logSO3(np.dot(th[i].transpose(), th_r[i]))) + dt*( - dth[i]) #+ ddth_r[i]

        return np.concatenate(ddth_des)
