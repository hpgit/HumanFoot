import numpy as np


class PDController:
    """
    :type h : float
    :type skel : pydart.Skeleton
    :type qhat : np.array
    """
    def __init__(self, skel, h, Kp, Kd):
        self.h = h
        self.skel = skel
        self.Kp, self.Kd = Kp, Kd
        ndofs = self.skel.ndofs
        self.Kp = np.diagflat([Kp] * ndofs)
        self.Kd = np.diagflat([Kd] * ndofs)
        self.preoffset = 0.0

    def setKpKd(self, Kp, Kd):
        ndofs = self.skel.ndofs
        self.Kp = np.diagflat([Kp] * ndofs)
        self.Kd = np.diagflat([Kd] * ndofs)

    def compute_flat(self, qhat, robustPD=True):
        skel = self.skel
        q = skel.q
        dq = skel.dq

        if robustPD:
            invM = np.linalg.inv(skel.M + self.Kd * self.h)
            p = -self.Kp.dot(-skel.position_differences(q, qhat) + dq * self.h)
            d = -self.Kd.dot(dq)
            qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
            tau = p + d - self.Kd.dot(qddot) * self.h
        else:
            tau = self.Kp * skel.position_differences(q, qhat) - self.Kd.dot(skel.dq)
        tau[0:6] = np.zeros(6)

        return tau
