import pydart2
import numpy as np
import PyCommon.modules.Util.ysPythonEx as ype
import copy
from PyCommon.modules.Math import mmMath as mm

from PyCommon.modules.ArticulatedBody import ysReferencePoints as yrp
from PyCommon.modules.ArticulatedBody import ysMomentum as ymt
import PyCommon.modules.Optimization.ysAnalyticConstrainedOpt as yac


def getTrackingWeightDart(DOFs, skeleton, weightMap, rootPositionWeight=0.):
    weights = [1.]*skeleton.num_joints()
    for name, weight in weightMap.items():
        index = skeleton.body('h'+name[1:]).index_in_skeleton()
        weights[index] = weight

    totalDOF = 0
    for dof in DOFs:
        totalDOF += dof

    weights_ext = [None]*totalDOF
    ype.repeatListElements(weights, weights_ext, DOFs)
    weights_ext[0:3] = [rootPositionWeight, rootPositionWeight, rootPositionWeight]

    return weights_ext


def addTrackingTerms(problem, totalDOF, weight, jointWeights, ddth_des_flat):
    # minimize | Wt(ddth_des - ddth) |^2
    problem.addObjective_matrix(np.diag( [jointWeights[i] for i in range(len(jointWeights))] ), np.array([jointWeights[i]*ddth_des_flat[i] for i in range(len(jointWeights))]), weight )


def addLinearTerms(problem, totalDOF, weight, dL_des, R, r_bias):
    # minimize | dL_des - (R*ddth + r_bias) |^2
    problem.addObjective_matrix(R, dL_des - r_bias, weight)


def addAngularTerms(problem, totalDOF, weight, dH_des, S, s_bias):
    # minimize | dH_des - (S*ddth + s_bias) |^2
    problem.addObjective_matrix(S, dH_des - s_bias, weight)


def addEndEffectorTerms(problem, totalDOF, weight, J, dJ, dth, ddP_des):
    # minimize | ddP_des - (J*ddth + dJ*dth)|^2
    problem.addObjective_matrix(J, ddP_des - np.dot(dJ, dth), weight)


def addSoftPointConstraintTerms(problem, totalDOF, weight, ddP_des, Q, q_bias):
    # minimize | ddP_des - (Q*ddth + q_bias) |^2
    problem.addObjective_matrix(Q, ddP_des - q_bias, weight)


def setConstraint(problem, totalDOF, J, dJ, dth_flat, a_sup):
    # subject to J_sup*ddth + dJ_sup*dth_flat = a_sup
    problem.setConstraint_matrix(J, a_sup - np.dot(dJ, dth_flat))


def addConstraint(problem, totalDOF, J, dJ, dth_flat, a_sup):
    # subject to J_sup*ddth + dJ_sup*dth_flat = a_sup
    problem.addConstraint_matrix(J, a_sup - np.dot(dJ, dth_flat))


class DartMomentumBalanceController:
    def __init__(self, skeleton, ref_skeleton, weight_map, up_vec_in_each_link):
        """

        :param skeleton:
        :type skeleton: pydart2.Skeleton
        :param ref_skeleton:
        :type ref_skeleton: pydart2.Skeleton
        :param Kt:
        :param Kl:
        :param Kh:
        :param Bl:
        :param Bh:
        :param kt_sup:
        """
        Kt = 25.
        Dt = 2.*(Kt**.5)
        Kl = 100.
        Dl = 2.*(Kt**.5)
        Kh = 100.
        Dh = 2.*(Kt**.5)
        Bt = 1.
        Bl = 0.1
        Bh = 0.13
        kt_sup = 22.

        self.skel = skeleton
        self.ref_skel = ref_skeleton
        self.Kt = Kt
        self.Kl = Kl
        self.Kh = Kh
        self.Bt = 1.
        self.Bl = Bl
        self.Bh = Bh
        self.kt_sup = kt_sup
        self.Dt = 2.*(Kt**.5)
        self.Dl = (Kl**.5)
        self.Dh = (Kh**.5)
        self.dt_sup = 2.*(kt_sup**.5)

        self.ndofs = self.skel.ndofs
        self.Kp = np.diagflat([self.Kt] * self.ndofs)
        self.Kd = np.diagflat([self.Dt] * self.ndofs)

        self.DOFs = [joint.num_dofs() for joint in self.skel.joints]

        self.weight_map = weight_map
        self.w = getTrackingWeightDart(self.DOFs, self.skel, weight_map)

        self.num_bodies = self.skel.num_bodynodes()

        self.CP_old = None  # type: np.ndarray
        self.linkMasses = [self.skel.body(i).mass() for i in range(self.num_bodies)]
        self.totalMass = self.skel.M
        self.TO = ymt.make_TO(self.linkMasses)
        self.dTO = ymt.make_dTO(len(self.linkMasses))
        self.problem = yac.LSE(self.ndofs, 12)

        self.supL = self.skel.body('h_blade_left')
        self.supR = self.skel.body('h_blade_right')

        self.ref_supL = self.ref_skel.body('h_blade_left')
        self.ref_supR = self.ref_skel.body('h_blade_right')

        self.up_vec_in_each_link = up_vec_in_each_link

    def set_parameters(self, Kt, Kl, Kh, Bl, Bh, kt_sup):
        self.Kt = Kt
        self.Kl = Kl
        self.Kh = Kh
        self.Bt = 1.
        self.Bl = Bl
        self.Bh = Bh
        self.kt_sup = kt_sup
        self.Dt = 2.*(Kt**.5)
        self.Dl = (Kl**.5)
        self.Dh = (Kh**.5)
        self.dt_sup = 2.*(kt_sup**.5)

    def solve(self, q_ref, contact_ids, des_com_offset, r_idx, l_idx, CP, leg_names):
        """

        :param q_ref: desired posture
        :param contact_des_ids: indicies of contact bodies
        :return:
        """
        ddq_des = self.Kp.dot(q_ref - self.skel.q) - self.Kd.dot(self.skel.dq)

        #################################################
        # jacobian
        #################################################

        # contact_ids = list()  # temp idx for balancing
        # contact_ids.extend(contact_des_ids)

        contact_body_ori = list(self.skel.body(i).world_transform()[:3, :3] for i in range(len(contact_ids)))
        contact_body_pos = list(self.skel.body(i).world_transform()[:3, 3] for i in range(len(contact_ids)))
        contact_body_vel = list(self.skel.body(i).world_linear_velocity() for i in range(len(contact_ids)))
        contact_body_angvel = list(self.skel.body(i).world_angular_velocity() for i in range(len(contact_ids)))

        ref_body_ori = list(self.ref_skel.body(i).world_transform()[:3, :3] for i in range(len(contact_ids)))
        ref_body_pos = list(self.ref_skel.body(i).world_transform()[:3, 3] for i in range(len(contact_ids)))

        is_contact = [1] * len(contact_ids)
        contact_right = len(set(contact_ids).intersection(r_idx)) > 0
        contact_left = len(set(contact_ids).intersection(l_idx)) > 0

        contMotionOffset = self.skel.body(0).to_world() - self.ref_skel.body(0).to_world()

        linkFrames = [self.skel.body(i).world_transform() for i in range(self.num_bodies)]
        linkPositions = [self.skel.body(i).to_world() for i in range(self.num_bodies)]
        linkVelocities = [self.skel.body(i).world_linear_velocity() for i in range(self.num_bodies)]
        linkAngVelocities = [self.skel.body(i).world_angular_velocity() for i in range(self.num_bodies)]
        linkInertias = [np.dot(linkFrames[i][:3, :3], np.dot(self.skel.body(i).inertia(), linkFrames[i][:3, :3].T)) for i in range(self.num_bodies)]

        # CM = yrp.getCM(linkPositions, linkMasses, totalMass)
        # dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)
        CM = self.skel.C
        dCM = self.skel.dC
        CM_plane = copy.copy(CM)
        CM_plane[1] = 0.
        dCM_plane = copy.copy(dCM)
        dCM_plane[1] = 0.

        P = ymt.getPureInertiaMatrix(self.TO, self.linkMasses, linkPositions, CM, linkInertias)
        dP = ymt.getPureInertiaMatrixDerivative(self.dTO, self.linkMasses, linkVelocities, dCM, linkAngVelocities, linkInertias)

        # calculate jacobian
        Jsys = np.empty((6*self.num_bodies, self.skel.ndofs))
        dJsys = np.empty_like(Jsys)
        for i in range(self.num_bodies):
            Jsys[6*i:6*i+6, :] = self.skel.body(i).world_jacobian()[range(-3, 3), :]
            dJsys[6*i:6*i+6, :] = self.skel.body(i).world_jacobian_classic_deriv()[range(-3, 3), :]

        J_contacts = []  # type: list[np.ndarray]
        dJ_contacts = []  # type: list[np.ndarray]
        for contact_id in contact_ids:
            J_contacts.append(Jsys[6*contact_id:6*contact_id + 6, :])
            dJ_contacts.append(dJsys[6*contact_id:6*contact_id + 6])

        # calculate footCenter
        footCenter = sum(contact_body_pos) / len(contact_body_pos) if len(contact_body_pos) > 0 \
            else .5 * (self.supL.to_world() + self.supR.to_world())
        footCenter[1] = 0.
        # if len(contact_body_pos) > 2:
        #     hull = ConvexHull(contact_body_pos)

        footCenter_ref = sum(ref_body_pos) / len(ref_body_pos) if len(ref_body_pos) > 0 \
            else .5 * (self.ref_supL.to_world() + self.ref_supR.to_world())
        footCenter_ref = footCenter_ref + contMotionOffset
        # if len(ref_body_pos) > 2:
        #     hull = ConvexHull(ref_body_pos)
        footCenter_ref[1] = 0.

        # linear momentum
        # CM_ref_plane = footCenter
        # CM_ref_plane = footCenter_ref
        CM_ref = footCenter + np.asarray(des_com_offset)
        dL_des_plane = self.Kl * self.skel.mass() * (CM_ref - CM) - self.Dl * self.skel.mass() * dCM
        # dL_des_plane = Kl * totalMass * (CM_ref_plane - CM_plane) - Dl * totalMass * dCM_plane
        # dL_des_plane[1] = 0.
        # print('dCM_plane : ', np.linalg.norm(dCM_plane))

        # angular momentum
        CP_ref = footCenter
        # CP_ref = footCenter_ref
        # bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
        # CP = yrp.getCP(contactPositions, contactForces)
        if self.CP_old is None or CP is None:
            dCP = None
        else:
            dCP = (CP - self.CP_old)/(1/30.)
        self.CP_old = CP

        if CP is not None and dCP is not None:
            ddCP_des = self.Kh*(CP_ref - CP) - self.Dh * dCP
            dCP_des = dCP + ddCP_des * (1/30.)
            CP_des = CP + dCP*(1/30.) + .5*ddCP_des*((1/30.)**2)
            dH_des = np.cross((CP_des - CM), (dL_des_plane + self.skel.mass() * mm.s2v(self.skel.world.gravity())))
        else:
            dH_des = None

        # convex hull
        # contact_pos_2d = np.asarray([np.array([contactPosition[0], contactPosition[2]]) for contactPosition in contactPositions])
        # p = np.array([CM_plane[0], CM_plane[2]])
        # hull = None  # type: Delaunay
        # if contact_pos_2d.shape[0] > 0:
        #     hull = Delaunay(contact_pos_2d)
        #     print(hull.find_simplex(p) >= 0)

        # set up equality constraint
        a_oris = list(map(mm.logSO3, [np.dot(np.dot(ref_body_ori[i], mm.getSO3FromVectors(np.dot(ref_body_ori[i], self.up_vec_in_each_link[contact_ids[i]]), mm.unitY())), contact_body_ori[i].T) for i in range(len(contact_body_ori))]))
        body_qs = list(map(mm.logSO3, contact_body_ori))
        body_angs = [np.dot(contact_body_ori[i], contact_body_angvel[i]) for i in range(len(contact_body_ori))]
        body_dqs = [mm.vel2qd(body_angs[i], body_qs[i]) for i in range(len(body_angs))]
        # a_oris = [np.dot(contact_body_ori[i], mm.qdd2accel(body_ddqs[i], body_dqs[i], body_qs[i])) for i in range(len(contact_body_ori))]

        KT_SUP = np.diag([self.kt_sup/10., self.kt_sup, self.kt_sup/10.])
        # KT_SUP = np.diag([kt_sup, kt_sup, kt_sup])

        a_sups = [np.append(np.dot(KT_SUP, (ref_body_pos[i] - contact_body_pos[i] + contMotionOffset)) - self.dt_sup * contact_body_vel[i],
                            self.kt_sup*a_oris[i] - self.dt_sup * contact_body_angvel[i]) for i in range(len(a_oris))]

        # momentum matrix
        RS = np.dot(P, Jsys)
        R, S = np.vsplit(RS, 2)

        rs = np.dot((np.dot(dP, Jsys) + np.dot(P, dJsys)), self.skel.dq)
        # rs = np.dot(dP, np.dot(Jsys, self.skel.dq)) + np.dot(P, dJsys)
        r_bias, s_bias = np.hsplit(rs, 2)

        #######################################################
        # optimization
        #######################################################
        if contact_left and not contact_right:
            self.weight_map['j_thigh_right'] = .8
            self.weight_map['j_shin_right'] = .8
            self.weight_map['j_heel_right'] = .8
        else:
            self.weight_map['j_thigh_right'] = .1
            self.weight_map['j_shin_right'] = .25
            self.weight_map['j_heel_right'] = .2

        if contact_right and not contact_left:
            self.weight_map['j_thigh_left'] = .8
            self.weight_map['j_shin_left'] = .8
            self.weight_map['j_heel_left'] = .8
        else:
            self.weight_map['j_thigh_left'] = .1
            self.weight_map['j_shin_left'] = .25
            self.weight_map['j_heel_left'] = .2

        # if contact_left and not contact_right:
        #     self.weight_map['RightUpLeg'] = .8
        #     self.weight_map['RightLeg'] = .8
        #     self.weight_map['RightFoot'] = .8
        # else:
        #     self.weight_map['RightUpLeg'] = .1
        #     self.weight_map['RightLeg'] = .25
        #     self.weight_map['RightFoot'] = .2
        #
        # if contact_right and not contact_left:
        #     self.weight_map['LeftUpLeg'] = .8
        #     self.weight_map['LeftLeg'] = .8
        #     self.weight_map['LeftFoot'] = .8
        # else:
        #     self.weight_map['LeftUpLeg'] = .1
        #     self.weight_map['LeftLeg'] = .25
        #     self.weight_map['LeftFoot'] = .2

        w = getTrackingWeightDart(self.DOFs, self.skel, self.weight_map)

        addTrackingTerms(self.problem, self.ndofs, self.Bt, w, ddq_des)
        if dH_des is not None:
            addLinearTerms(self.problem, self.ndofs, self.Bl, dL_des_plane, R, r_bias)
            addAngularTerms(self.problem, self.ndofs, self.Bh, dH_des, S, s_bias)

            if True:
                for c_idx in range(len(contact_ids)):
                    addConstraint(self.problem, self.ndofs, J_contacts[c_idx], dJ_contacts[c_idx], self.skel.dq, a_sups[c_idx])

        r = self.problem.solve()
        self.problem.clear()
        ddth_sol_flat = np.asarray(r['x'])

        return ddth_sol_flat
