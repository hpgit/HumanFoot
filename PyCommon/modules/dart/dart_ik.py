import pydart2 as pydart
import numpy as np
from scipy.optimize import minimize

from PyCommon.modules.Math import mmMath as mm


class IkConstraint(object):
    def __init__(self):
        pass

    def f(self):
        raise NotImplementedError

    def g(self):
        raise NotImplementedError

    def update(self):
        raise NotImplementedError


class IkOriConstraint(IkConstraint):
    def __init__(self, body, des, weight):
        """

        :type body: pydart.BodyNode
        :param des:
        :param weight:
        """
        super().__init__()
        self.body = body
        self.des = mm.seq2Vec3(mm.logSO3(des))
        self.weight = weight
        self.cur = mm.seq2Vec3(mm.logSO3(self.body.world_transform()[:3, :3]))

    def f(self):
        return self.weight * 0.5 * mm.length(self.des - self.cur) ** 2

    def g(self):
        return np.dot(self.cur - self.des, self.body.angular_jacobian())[6:]

    def update(self):
        self.cur = mm.seq2Vec3(mm.logSO3(self.body.world_transform()[:3, :3]))


class IkPosConstraint(IkConstraint):
    def __init__(self, body, offset, des, weight):
        """

        :type body: pydart.BodyNode
        :param des:
        :param weight:
        """
        super().__init__()
        self.body = body
        self.offset = offset
        self.des = des
        self.weight = weight
        self.cur = self.body.to_world(self.offset)

    def f(self):
        return self.weight * 0.5 * np.linalg.norm(self.des - self.cur) ** 2

    def g(self):
        return np.dot(self.cur - self.des, self.body.linear_jacobian(self.offset))[6:]

    def update(self):
        self.cur = self.body.to_world(self.offset)


class DartIk(object):
    def __init__(self, skel):
        """

        :type skel: pydart.Skeleton
        """
        self.skel = skel
        self.constraints = list()  # type: list[IkConstraint]

    def add_joint_pos_const(self, body_name_or_idx, pos, weight=1.):
        joint = self.skel.joint(body_name_or_idx)
        offset = joint.transform_from_parent_body_node()[:3, 3]
        self.constraints.append(IkPosConstraint(joint.parent_bodynode, offset, pos, weight))

    def add_position_const(self, body_name_or_idx, pos, offset, weight=1.):
        body = self.skel.body(body_name_or_idx)  # type: pydart.BodyNode
        self.constraints.append(IkPosConstraint(body, offset, pos, weight))

    def add_orientation_const(self, body_name_or_idx, ori, weight=1.):
        body = self.skel.body(body_name_or_idx)  # type: pydart.BodyNode
        self.constraints.append(IkOriConstraint(body, ori, weight))

    def clean_constraints(self):
        del self.constraints[:]

    def update_skel(self, x):
        q = self.skel.positions()
        q[6:] = x
        self.skel.set_positions(q)

    def f(self, x):
        self.update_skel(x)

        f = 0.
        for constraint in self.constraints:
            constraint.update()
            f += constraint.f()
        return f

    def g(self, x):
        self.update_skel(x)
        g = np.zeros_like(x)
        for constraint in self.constraints:
            constraint.update()
            g += constraint.g()

        return g

    def solve(self):
        # res = minimize(self.f,
        #                x0=self.skel.positions()[6:],
        #                method="SLSQP")
        res = minimize(self.f,
                       x0=self.skel.positions()[6:],
                       jac=self.g,
                       method="SLSQP")
        # res = minimize(self.f,
        #                x0=self.skel.positions()[6:],
        #                jac=self.g,
        #                method="SLSQP")
        # print(res)
        self.update_skel(res['x'])
