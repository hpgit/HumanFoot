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
        self.preoffset = 0.0
        self.Kp_matrix = np.diagflat([0.0] * 6 + [Kp] * (skel.ndofs - 6))
        self.Kd_matrix = np.diagflat([0.0] * 6 + [Kd] * (skel.ndofs - 6))

    def setKpKd(self, Kp, Kd):
        self.Kp, self.Kd = Kp, Kd

    def compute_flat(self, qhat, robustPD=True):
        skel = self.skel
        q = skel.q
        dq = skel.dq

        if robustPD:
            invM = np.linalg.inv(skel.M + self.Kd_matrix * self.h)
            p = self.Kp_matrix.dot(skel.position_differences(qhat, q) - dq * self.h)
            d = -self.Kd_matrix.dot(dq)
            qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
            tau = p + d - self.Kd_matrix.dot(qddot) * self.h
        else:
            tau = self.Kp_matrix.dot(skel.position_differences(qhat, q)) - self.Kd_matrix.dot(skel.dq)

        tau[0:6] = np.zeros(6)
        return tau

