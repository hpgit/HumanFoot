import numpy as np

class PDController:
    """
    :type h : float
    :type skel : pydart.Skeleton
    :type qhat : np.array
    """
    def __init__(self, skel, h, Kp, Kd, weightMap=None):
        self.h = h
        self.skel = skel
        # ndofs = self.skel.ndofs
        # self.qhat = self.skel.q
        # self.Kp = np.diagflat([0.0] * 6 + [Kp] * (ndofs - 6))
        # self.Kd = np.diagflat([0.0] * 6 + [Kd] * (ndofs - 6))
        self.setKpKd(Kp, Kd, weightMap)
        self.preoffset = 0.0

    def setKpKd(self, Kp, Kd, weightMap=None):
        ndofs = self.skel.ndofs

        if weightMap is None:
            self.Kp = np.diagflat([Kp] * ndofs)
            self.Kd = np.diagflat([Kd] * ndofs)
        else:
            Kp_list = [Kp] * ndofs
            Kd_list = [Kd] * ndofs
            for j in range(self.skel.num_joints()):
                joint = self.skel.joint(j)  # type: pydart.Joint
                for d in range(joint.num_dofs()):
                    dof = joint.dofs[d]  # type:  pydart.Dof
                    Kp_list[dof.index_in_skeleton()] = Kp * weightMap[joint.name[2:]]
                    Kd_list[dof.index_in_skeleton()] = Kd * weightMap[joint.name[2:]]
            self.Kp = np.diagflat(Kp_list)
            self.Kd = np.diagflat(Kd_list)

    def compute_flat(self, qhat, robustPD=True):
        skel = self.skel

        if robustPD:
            invM = np.linalg.inv(skel.M + self.Kd * self.h)
            p = -self.Kp.dot(skel.q + skel.dq * self.h - qhat)
            d = -self.Kd.dot(skel.dq)
            qddot = invM.dot(-skel.c + p + d + skel.constraint_forces())
            tau = p + d - self.Kd.dot(qddot) * self.h
        else:
            tau = self.Kp.dot(qhat - skel.q) - self.Kd.dot(skel.dq)
        tau[0:6] = np.zeros(6)

        return tau

