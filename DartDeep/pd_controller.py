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

    def setKpKd(self, Kp, Kd):
        self.Kp, self.Kd = Kp, Kd

    def compute_flat(self, qhat, robustPD=True):
        print('haha0')
        skel = self.skel
        print('haha1')
        q = skel.q
        print('haha2')
        dq = skel.dq
        print('haha3')

        if robustPD:
            print('haha4')
            invM = np.linalg.inv(skel.M + self.Kd * np.eye(skel.ndofs) * self.h)
            print('haha5')
            p = self.Kp * (skel.position_differences(qhat, q) - dq * self.h)
            print('haha6')
            d = -self.Kd * dq
            print('haha7')
            qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
            print('haha8')
            tau = p + d - self.Kd * qddot * self.h
            print('haha9')
        else:
            tau = self.Kp * skel.position_differences(qhat, q) - self.Kd * skel.dq

        tau[0:6] = np.zeros(6)
        print('haha10')
        return tau

