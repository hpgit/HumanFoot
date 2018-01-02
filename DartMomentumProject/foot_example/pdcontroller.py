import numpy as np
from numpy.linalg import inv
# import pydart
from PyCommon.modules import pydart2 as pydart


class PDController:
    """
    :type h : float
    :type skel : pydart.Skeleton
    :type qhat : np.array
    """
    def __init__(self, skel, h, Kp, Kd):
        self.h = h
        self.skel = skel
        ndofs = self.skel.ndofs
        # self.qhat = self.skel.q
        self.Kp = np.diagflat([0.0] * 6 + [Kp] * (ndofs - 6))
        self.Kd = np.diagflat([0.0] * 6 + [Kd] * (ndofs - 6))
        self.preoffset = 0.0

    def set_gain(self, Kp, Kd):
        ndofs = self.skel.ndofs
        self.Kp = np.diagflat([0.0] * 6 + [Kp] * (ndofs - 6))
        self.Kd = np.diagflat([0.0] * 6 + [Kd] * (ndofs - 6))

    def compute(self, qhat):
        skel = self.skel

        invM = inv(skel.M + self.Kd * self.h)
        p = -self.Kp.dot(skel.q + skel.dq * self.h - qhat)
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
        p = -self.Kp.dot(skel.q - qhat)
        d = -self.Kd.dot(skel.dq)
        tau = p + d
        tau[:6] = 0
        return tau
