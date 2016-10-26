# import Optimization.csLCPLemkeSolver as lcp
# import Optimization.csLCPDantzigSolver as lcpD
from cvxopt import matrix as cvxMatrix
from cvxopt import solvers as cvxSolvers

# from openopt import LCP as openLCP

from PyCommon.modules.ArticulatedBody import ysJacobian as yjc

from PyCommon.modules.Util import ysPythonEx as ype
from PyCommon.modules.Math import mmMath as mm

from PyCommon.modules.VirtualPhysics import LieGroup as VPL

import numpy as np
import numpy.linalg as npl
import math

# import scipy.optimize as spopt
from PyCommon.modules.Optimization import csQPOASES as qpos

import time
from copy import deepcopy

# import cvxpy as cvx


def makeFrictionCone(skeleton, world, model, bodyIDsToCheck, numFrictionBases):
    cVpBodyIds, cPositions, cPositionsLocal, cVelocities = world.getContactPoints(bodyIDsToCheck)
    N = None
    D = None
    E = None

    cNum = len(cVpBodyIds)
    if cNum == 0:
        return len(cVpBodyIds), cVpBodyIds, cPositions, cPositionsLocal, cVelocities, None, None, None, None, None
    d = [None]*numFrictionBases

    DOFs = model.getDOFs()
    Jic = yjc.makeEmptyJacobian(DOFs, 1)

    jointPositions = model.getJointPositionsGlobal()
    jointPositions[0] = model.getBodyPositionGlobal(0)

    # jointAxeses = model.getDOFAxeses()
    # body0Ori = model.getBodyOrientationGlobal(0)
    # for i in range(3):
    #     jointAxeses[0][i] = body0Ori.T[i]
    #     jointAxeses[0][i+3] = body0Ori.T[i]
    jointAxeses = model.getBodyRootDOFAxeses()

    totalDOF = model.getTotalDOF()
    qdot_0 = ype.makeFlatList(totalDOF)
    # ype.flatten(model.getDOFVelocitiesLocal(), qdot_0)
    # bodyGenVelLocal = model.getBodyGenVelLocal(0)
    #
    # for i in range(3):
    #     qdot_0[i] = bodyGenVelLocal[i+3]
    #     qdot_0[i+3] = bodyGenVelLocal[i]
    ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)

    for vpidx in range(len(cVpBodyIds)):
        bodyidx = model.id2index(cVpBodyIds[vpidx])
        contactJointMasks = [yjc.getLinkJointMask(skeleton, bodyidx)]
        # yjc.computeJacobian2(Jic, DOFs, jointPositions, jointAxeses, [cPositions[vpidx]], contactJointMasks)
        yjc.computeLocalRootJacobian(Jic, DOFs, jointPositions, jointAxeses, [cPositions[vpidx]], contactJointMasks)
        n = np.array([[0., 1., 0., 0., 0., 0.]]).T
        JTn = Jic.T.dot(n)
        if N is None:
            JTN = JTn.copy()
            N = n.copy()
        else:
            JTN = np.hstack((JTN, JTn))
            N = np.hstack((N, n))
        cVel = cVelocities[vpidx]
        offsetAngle = np.arctan2(cVel[2], cVel[0])
        offsetAngle = 0.
        for i in range(numFrictionBases):
            d[i] = np.array([[math.cos(offsetAngle + 2.*math.pi*i/numFrictionBases),
                              0.,
                              math.sin(offsetAngle + 2.*math.pi*i/numFrictionBases)
                              , 0., 0., 0.
                              ]]).T

        for i in range(numFrictionBases):
            JTd = Jic.T.dot(d[i])
            if D is None:
                JTD = JTd.copy()
                D = d[i].copy()
            else:
                JTD = np.hstack((JTD, JTd))
                D = np.hstack((D, d[i]))

    E = np.zeros((cNum*numFrictionBases, cNum))
    for cIdx in range(cNum):
        for fcIdx in range(numFrictionBases):
            E[cIdx*numFrictionBases + fcIdx][cIdx] = 1.

    return len(cVpBodyIds), cVpBodyIds, cPositions, cPositionsLocal, cVelocities, JTN, JTD, E, N, D


def repairForces(forces, contactPositions):
    for idx in range(0, len(forces)):
        force = forces[idx]
        if force[1] < 0.:
           force[0] = 0.
           force[2] = 0.
           force[1] = 0.
        #    force[1] = -contactPositions[idx][1]*2000.
        # elif force[1] > 10000.:
        #    ratio = 10000./force[1]
        #    force *= ratio
        # if force[1]*force[1] < force[2]*force[2] + force[0]*force[0] :
        #    norm = math.sqrt(force[0] * force[0] + force[2]*force[2])
        #    force[0] /= norm
        #    force[2] /= norm
        pass


def normalizeMatrix(A, b):
    for i in range(A.shape[0]):
        n = npl.norm(A[0])
        A[0] /= n
        b[0] /= n


def setTimeStamp(timeStamp, timeIndex, prevTime):
    if timeIndex == 0:
        prevTime = time.time()
    if len(timeStamp) < timeIndex + 1:
        timeStamp.append(0.)
    curTime = time.time()
    timeStamp[timeIndex] += curTime - prevTime
    prevTime = curTime
    timeIndex += 1
    return timeStamp, timeIndex, prevTime


def getLCPMatrix(world, model, invM, invMc, mu, tau, contactNum, contactPositions, JTN, JTD, E, factor=1.):
    totalDOF = model.getTotalDOF()

    h = world.GetTimeStep()
    invh = 1./h
    mus = mu * np.eye(contactNum)
    temp_NM = JTN.T.dot(invM)
    temp_DM = JTD.T.dot(invM)

    # pdb.set_trace()

    # A =[ A11,  A12, 0]
    #   [ A21,  A22, E]
    #   [ mus, -E.T, 0]

    A11 = h*temp_NM.dot(JTN)
    A12 = h*temp_NM.dot(JTD)
    A21 = h*temp_DM.dot(JTN)
    A22 = h*temp_DM.dot(JTD)

    A = factor * np.concatenate(
            (
                np.concatenate((A11, A12,  np.zeros((A11.shape[0], E.shape[1]))),  axis=1),
                np.concatenate((A21, A22,  E),                                     axis=1),
                0.001*h * np.concatenate((mus, -E.T, np.zeros((mus.shape[0], E.shape[1]))),  axis=1),
            ), axis=0
    )
    # A = A + 0.1*np.eye(A.shape[0])

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getDOFVelocitiesLocal(), qdot_0)
    # bodyGenVelLocal = model.getBodyGenVelLocal(0)
    # for i in range(3):
    #     qdot_0[i] = bodyGenVelLocal[i+3]
    #     qdot_0[i+3] = bodyGenVelLocal[i]
    ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)

    qdot_0 = np.asarray(qdot_0)
    if tau is None:
        tau = np.zeros(np.shape(qdot_0))

    # non-penentration condition
    # b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)

    # improved non-penentration condition : add position condition
    penDepth = 0.003
    bPenDepth = np.zeros(A11.shape[0])
    for i in range(contactNum):
        if abs(contactPositions[i][1]) > penDepth:
            bPenDepth[i] = contactPositions[i][1] + penDepth

    b1 = JTN.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau) + 0.05 * invh * bPenDepth
    b2 = JTD.T.dot(qdot_0 - h*invMc) + h*temp_DM.dot(tau)
    b3 = np.zeros(mus.shape[0])
    b = np.hstack((np.hstack((b1, b2)), b3)) * factor
    return A, b


def getLCPMatrixHD(world, model, invM, invMc, mu, ddth, contactNum, contactPositions, JTN, JTD, E, factor=1.):
    totalDOF = model.getTotalDOF()

    h = world.GetTimeStep()
    invh = 1./h
    mus = mu * np.eye(contactNum)

    M_small = np.dot(invM[:6, 6:], npl.inv(invM[6:, 6:]))
    M_tilde = invM[:6, :] - np.dot(M_small, invM[6:, :])

    # temp_NMtilde = np.dot(JTN.T, np.concatenate((M_tilde, np.zeros((JTN.shape[0]-M_tilde.shape[0], M_tilde.shape[1]))), axis=0))
    # temp_DMtilde = np.dot(JTD.T, np.concatenate((M_tilde, np.zeros((JTD.shape[0]-M_tilde.shape[0], M_tilde.shape[1]))), axis=0))

    temp_NMtilde = np.dot(JTN[:6, :].T, M_tilde)
    temp_DMtilde = np.dot(JTD[:6, :].T, M_tilde)

    A11 = h*temp_NMtilde.dot(JTN)
    A12 = h*temp_NMtilde.dot(JTD)
    A21 = h*temp_DMtilde.dot(JTN)
    A22 = h*temp_DMtilde.dot(JTD)

    A = factor * np.concatenate(
        (
            np.concatenate((A11, A12,  np.zeros((A11.shape[0], E.shape[1]))),  axis=1),
            np.concatenate((A21, A22,  E),                                     axis=1),
            0.001*h * np.concatenate((mus, -E.T, np.zeros((mus.shape[0], E.shape[1]))),  axis=1),
        ), axis=0
    )
    # A = A + 0.1*np.eye(A.shape[0])

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getDOFVelocitiesLocal(), qdot_0)
    # bodyGenVelLocal = model.getBodyGenVelLocal(0)
    # for i in range(3):
    #     qdot_0[i] = bodyGenVelLocal[i+3]
    #     qdot_0[i+3] = bodyGenVelLocal[i]
    ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)

    qdot_0 = np.asarray(qdot_0)
    if ddth is None:
        ddth = np.zeros(np.shape(qdot_0))

    # non-penentration condition
    # b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)

    # improved non-penentration condition : add position condition
    penDepth = 0.003
    bPenDepth = np.zeros(A11.shape[0])
    for i in range(contactNum):
        if abs(contactPositions[i][1]) > penDepth:
            bPenDepth[i] = contactPositions[i][1] + penDepth


    M = npl.inv(invM)

    b1 = JTN.T.dot(qdot_0) \
         - h*np.dot(temp_NMtilde, np.dot(M, invMc)) \
         + h*np.dot(JTN.T, np.dot(np.concatenate((M_small, np.eye(M_small.shape[1])), axis=0), ddth)) \
         + 0.05 * invh * bPenDepth

    b2 = JTD.T.dot(qdot_0) \
         - h*np.dot(temp_DMtilde, np.dot(M, invMc)) \
         + h*np.dot(JTD.T, np.dot(np.concatenate((M_small, np.eye(M_small.shape[1])), axis=0), ddth))

    b3 = np.zeros(mus.shape[0])

    b = np.hstack((np.hstack((b1, b2)), b3)) * factor
    return A, b


def getLCPMatrixGenHD(world, model, invM, invMc, mu, ddth, tau, contactNum, contactPositions, contactVelocities, JTN, JTD, E, factor=1., hdAccMask=None):

    if hdAccMask is None:
        hdAccMask = [True]*invM.shape[0]
        hdAccMask[:6] = [False]*6

    rearr_idx = np.array([i for i,x in enumerate(hdAccMask) if not x]+[i for i,x in enumerate(hdAccMask) if x])

    # for torque term, invM has to be rearranged both column and row
    invMtorReArr = (invM[:, rearr_idx])[rearr_idx, :]

    # JTN and JTD have to be rearranged row
    JTNreArr = JTN[rearr_idx, :]
    JTDreArr = JTD[rearr_idx, :]
    invMcReArr = invMc[rearr_idx]
    ddthReArr = np.array(ddth)[rearr_idx]
    tauReArr = np.array(tau)[rearr_idx]

    # TODO:

    totalTorDOF = len(hdAccMask) - sum(hdAccMask)

    totalDOF = model.getTotalDOF()

    h = world.GetTimeStep()
    invh = 1./h
    mus = mu * np.eye(contactNum)

    M_small = np.dot(invMtorReArr[:totalTorDOF, totalTorDOF:], npl.inv(invMtorReArr[totalTorDOF:, totalTorDOF:]))

    M_tilde = invMtorReArr[:totalTorDOF, :]            - np.dot(M_small, invMtorReArr[totalTorDOF:, :])
    M_schur = invMtorReArr[:totalTorDOF, :totalTorDOF] - np.dot(M_small, invMtorReArr[totalTorDOF:, :totalTorDOF])

    temp_NMtilde = np.dot(JTNreArr[:totalTorDOF].T, M_tilde)
    temp_DMtilde = np.dot(JTDreArr[:totalTorDOF].T, M_tilde)

    A11 = h*temp_NMtilde.dot(JTNreArr)
    A12 = h*temp_NMtilde.dot(JTDreArr)
    A21 = h*temp_DMtilde.dot(JTNreArr)
    A22 = h*temp_DMtilde.dot(JTDreArr)

    A = factor * np.concatenate(
        (
            np.concatenate((A11, A12,  np.zeros((A11.shape[0], E.shape[1]))),  axis=1),
            np.concatenate((A21, A22,  E),                                     axis=1),
            h * np.concatenate((mus, -E.T, np.zeros((mus.shape[0], E.shape[1]))),  axis=1),
        ), axis=0
    )
    # A = A + 0.1*np.eye(A.shape[0])

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)

    qdot_0 = np.asarray(qdot_0)
    if ddth is None:
        ddth = np.zeros(np.shape(qdot_0))

    # non-penentration condition
    # b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)

    # improved non-penentration condition : add position condition
    penDepth = 0.003
    # penDepth = 0.005
    bPenDepth = np.zeros(A11.shape[0])
    for i in range(contactNum):
        if abs(contactPositions[i][1]) > penDepth:
            bPenDepth[i] = contactPositions[i][1] + penDepth

    # additional friction
    fricVel = 0.01
    bFricVel = np.zeros(A21.shape[0])
    for i in range(contactNum):
        vel = fricVel * mm.normalize2(np.array([contactVelocities[i][0], 0, contactVelocities[i][2]]))
        if abs(contactVelocities[i][0]*contactVelocities[i][0]+contactVelocities[i][2]*contactVelocities[i][2]) > fricVel*fricVel:
            for j in range(8):
                dBasis = np.array([[math.cos(2.*math.pi*j/8.), 0., math.sin(2.*math.pi*j/8.)]])
                bFricVel[8*i + j] = np.dot(dBasis, vel)



    b1 = JTN.T.dot(qdot_0) \
         + h*np.dot(JTNreArr.T, np.hstack((np.dot(M_small, ddthReArr[totalTorDOF:]), ddthReArr[totalTorDOF:]))) \
         + h*np.dot(JTN[:totalTorDOF].T, np.dot(M_schur, tauReArr[:totalTorDOF]) - invMcReArr[:totalTorDOF] + np.dot(M_small, invMcReArr[totalTorDOF:])) \
         + 0.05* invh * bPenDepth

    b2 = JTD.T.dot(qdot_0) \
         + h*np.dot(JTDreArr.T, np.hstack((np.dot(M_small, ddthReArr[totalTorDOF:]), ddthReArr[totalTorDOF:]))) \
         + h*np.dot(JTD[:totalTorDOF].T, np.dot(M_schur, tauReArr[:totalTorDOF]) - invMcReArr[:totalTorDOF] + np.dot(M_small, invMcReArr[totalTorDOF:])) \
         + 0.0 * invh * bFricVel

    b3 = np.zeros(mus.shape[0])

    b = np.hstack((np.hstack((b1, b2)), b3)) * factor
    return A, b



def calcLCPForces(motion, world, model, bodyIDsToCheck, mu, tau=None, numFrictionBases=8, solver='qp'):
    timeStamp = []
    timeIndex = 0
    prevTime = time.time()

    # model = VpControlModel
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, contactVelocities, JTN, JTD, E, N, D\
        = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)

    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None, None
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)

    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF, totalDOF))
    invMc = np.zeros(totalDOF)

    model.getInverseEquationOfMotion(invM, invMc)

    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)

    # pdb.set_trace()
    # A =[ A11,  A12, 0]
    #   [ A21,  A22, E]
    #   [ mus, -E.T, 0]

    factor = 1.
    A, b = getLCPMatrix(world, model, invM, invMc, mu, tau, contactNum, contactPositions, JTN, JTD, E, factor)

    # lo = np.zeros(A.shape[0])
    lo = 0.*np.ones(A.shape[0])
    hi = 1000000. * np.ones(A.shape[0])
    x = 0.*np.ones(A.shape[0])

    # normalizeMatrix(A, b)
    # print A[0]

    if solver == 'bulletLCP':
        # solve using bullet LCP solver
        lcpSolver = lcp.LemkeSolver()
        # lcpSolver = lcpD.DantzigSolver()
        lcpSolver.solve(A.shape[0], A, b, x, lo, hi)

    if solver == 'openOptLCP':
        # solve using openOpt LCP solver
        # p = openLCP(A, b)
        # r = p.solve('lcpsolve')
        # f_opt, x_opt = r.ff, r.xf
        # w, x = x_opt[x_opt.size/2:], x_opt[:x_opt.size/2]
        pass

    if solver == 'nqp':
        # solve using cvxopt Nonlinear Optimization with linear constraint
        Acp = cvxMatrix(A)
        bcp = cvxMatrix(b)
        Hcp = cvxMatrix(A+A.T)
        Gcp = cvxMatrix(np.vstack((-A, -np.eye(A.shape[0]))))
        hcp = cvxMatrix(np.hstack((b.T, np.zeros(A.shape[0]))))

        def F(xin=None, z=None):
            if xin is None:
                return 0, cvxMatrix(1., (A.shape[1], 1))
            for j in range(len(np.array(xin))):
                if xin[j] < 0.:
                    return None, None
            f = xin.T*(Acp*xin+bcp)
            # TODO:
            # check!!!
            Df = Hcp*xin + bcp
            if z is None:
                return f, Df.T
            H = Hcp
            return f, Df.T, H
        solution = cvxSolvers.cp(F, Gcp, hcp)
        xcp = np.array(solution['x']).flatten()
        x = xcp.copy()

    if solver == 'qp':
        # solve using cvxopt QP
        # if True:
        try:
            Aqp = cvxMatrix(A+A.T)
            bqp = cvxMatrix(b)
            Gqp = cvxMatrix(np.vstack((-A, -np.eye(A.shape[0]))))
            hqp = cvxMatrix(np.hstack((b.T, np.zeros(A.shape[0]))))
            timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)
            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiters'] = 100
            # cvxSolvers.options['kktreg'] = 1e-6
            # cvxSolvers.options['refinement'] = 10
            solution = cvxSolvers.qp(Aqp, bqp, Gqp, hqp)
            xqp = np.array(solution['x']).flatten()
            # xqp = np.array(cvxSolvers.qp(Aqp, bqp, Gqp, hqp)['x']).flatten()
            x = xqp.copy()
            # print x.shape[0]
            # print "x: ", x
            # zqp = np.dot(A,x)+b
            # print "z: ", zqp
            # print "El: ", np.dot(E, x[contactNum + numFrictionBases*contactNum:])
            # print "Ep: ", np.dot(E.T, x[contactNum:contactNum + numFrictionBases*contactNum])
            # print "force value: ", np.dot(x, zqp)
        except Exception, e:
            # print e
            pass

    if solver == 'qpOASES':
        # solve using qpOASES
        QQ = A+A.T
        pp = b
        # GG = np.vstack((A, np.eye(A.shape[0])))
        # hh = np.hstack((-b.T, np.zeros(A.shape[0])))
        GG = A.copy()
        hh = -b

        # bp::list qp(const object &H, const object &g, const object &A, const object &lb, const object &ub, const object &lbA, const object ubA, int nWSR)
        lb = [0.]*A.shape[0]
        xqpos = qpos.qp(QQ, pp, GG, lb, None, hh, None, 1000, False, "NONE")
        x = np.array(xqpos)
        zqp = np.dot(A,x)+b
        print np.dot(x, zqp)
        # print xqpos
        # x = xqpos.copy()
        pass


    normalForce = x[:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    # tangenForce = np.zeros_like(x[contactNum:contactNum + numFrictionBases*contactNum])
    minTangenVel = x[contactNum + numFrictionBases*contactNum:]
    # print minTangenVel
    print (np.dot(A,x)+b)[contactNum:contactNum+numFrictionBases*contactNum]



    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]
        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
        # print force
        forces.append(force)

    # repairForces(forces, contactPositions)
    # print forces
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)
    return bodyIDs, contactPositions, contactPositionsLocal, forces, timeStamp


def calcLCPForcesIter(motion, world, model, bodyIDsToCheck, mu, tau=None, numFrictionBases=8, solver='qp'):
    timeStamp = []
    timeIndex = 0
    prevTime = time.time()

    # model = VpControlModel
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, contactVelocities, JTN, JTD, E, N, D \
        = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)

    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None, None
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)

    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF, totalDOF))
    invMc = np.zeros(totalDOF)
    model.getInverseEquationOfMotion(invM, invMc)
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)

    # pdb.set_trace()
    # A =[ A11,  A12, 0]
    #   [ A21,  A22, E]
    #   [ mus, -E.T, 0]

    factor = 1.
    A, b = getLCPMatrix(world, model, invM, invMc, mu, tau, contactNum, contactPositions, JTN, JTD, E, factor)

    x = 100.*np.ones(A.shape[0])

    if solver == 'qp':
        # solve using cvxopt QP
        try:
            Aqp = cvxMatrix(A+A.T)
            bqp = cvxMatrix(b)
            Gqp = cvxMatrix(np.vstack((-A, -np.eye(A.shape[0]))))
            hqp = cvxMatrix(np.hstack((b.T, np.zeros(A.shape[0]))))
            timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)
            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiters'] = 100
            solution = cvxSolvers.qp(Aqp, bqp, Gqp, hqp)
            xqp = np.array(solution['x']).flatten()
            x = xqp.copy()
            # print x
        except Exception, e:
            # print e
            pass

    tangent_idx = contactNum + numFrictionBases*contactNum
    tanvel_idx = contactNum + numFrictionBases*contactNum + contactNum

    mask_eps = 0.00001
    # x_mask = [False] * len(x)
    x_mask = range(tanvel_idx)


    for i in range(contactNum):
        # normal force activated
        if x[i] > mask_eps:
            # x_mask[i] = True
            x_mask.remove(i)

    for i in range(tangent_idx, tanvel_idx):
        # initially, all tangential velocity activated
        # x_mask[i] = True
        x_mask.remove(i)

    for i in range(contactNum):
        maxFricIdx = np.argmax(x[contactNum+i*numFrictionBases:contactNum+(i+1)*numFrictionBases])
        if x[contactNum + i*numFrictionBases + maxFricIdx] > mask_eps:
            # major tangential force direction activated
            # x_mask[contactNum + i*numFrictionBases + maxFricIdx] = True
            x_mask.remove(contactNum + i*numFrictionBases + maxFricIdx)
        else:
            # if tangential force deactivated, tangential velocity deactivated
            # x_mask[tangent_idx + i] = False
            x_mask.append(tangent_idx + i)

    # print contactNum
    # print x_mask
    A2 = np.delete(A, x_mask, axis=0)
    A2 = np.delete(A2, x_mask, axis=1)
    b2 = np.delete(b, x_mask)
    x2 = np.zeros_like(b2)

    if solver == 'qp':
        # solve using cvxopt QP
        try:
            Aqp2 = cvxMatrix(A2+A2.T)
            bqp2 = cvxMatrix(b2)
            Gqp2 = cvxMatrix(np.vstack((-A2, -np.eye(A2.shape[0]))))
            hqp2 = cvxMatrix(np.hstack((b2.T, np.zeros(A2.shape[0]))))
            timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)
            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiters'] = 100
            solution2 = cvxSolvers.qp(Aqp2, bqp2, Gqp2, hqp2)
            xqp2 = np.array(solution2['x']).flatten()
            x2 = xqp2.copy()

        except Exception, e:
            # print e
            pass

    # normalForce = x[:contactNum]
    # tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    # minTangenVel = x[contactNum + numFrictionBases*contactNum:]

    x_final = []
    jj = 0
    for i in range(tanvel_idx):

        if i in x_mask:
            x_final.append(0.)
        else:
            x_final.append(x2[jj])
            jj += 1

    # print x_final

    normalForce = np.array(x_final[:contactNum])
    tangenForce = np.array(x_final[contactNum:contactNum + numFrictionBases*contactNum])
    minTangenVel = np.array(x_final[contactNum + numFrictionBases*contactNum:])




    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]
        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
        forces.append(force)

    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)
    return bodyIDs, contactPositions, contactPositionsLocal, forces, timeStamp


def calcLCPForcesVert(motion, world, model, bodyIDsToCheck, mu, tau=None, numFrictionBases=8, solver='qp'):
    timeStamp = []
    timeIndex = 0
    prevTime = time.time()

    # model = VpControlModel
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, contactVelocities, JTN, JTD, E, N, D \
        = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)

    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None, None
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)

    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF, totalDOF))
    invMc = np.zeros(totalDOF)
    model.getInverseEquationOfMotion(invM, invMc)
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)

    # pdb.set_trace()
    # A =[ A11,  A12, 0]
    #   [ A21,  A22, E]
    #   [ mus, -E.T, 0]

    totalDOF = model.getTotalDOF()

    h = world.GetTimeStep()
    invh = 1./h
    temp_NM = JTN.T.dot(invM)

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)
    qdot_0 = np.asarray(qdot_0)
    if tau is None:
        tau = np.zeros(np.shape(qdot_0))

    factor = 1.
    A = h*temp_NM.dot(JTN)
    b = JTN.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau) #+ 0.5*invh*bPenDepth


    # lo = np.zeros(A.shape[0])
    lo = 0.*np.ones(A.shape[0])
    hi = 1000000. * np.ones(A.shape[0])
    x = 100.*np.ones(A.shape[0])

    # normalizeMatrix(A, b)
    # print A[0]

    if solver == 'bulletLCP':
        # solve using bullet LCP solver
        lcpSolver = lcp.LemkeSolver()
        # lcpSolver = lcpD.DantzigSolver()
        lcpSolver.solve(A.shape[0], A, b, x, lo, hi)

    if solver == 'openOptLCP':
        # solve using openOpt LCP solver
        # p = openLCP(A, b)
        # r = p.solve('lcpsolve')
        # f_opt, x_opt = r.ff, r.xf
        # w, x = x_opt[x_opt.size/2:], x_opt[:x_opt.size/2]
        pass

    if solver == 'qp':
        # solve using cvxopt QP
        # if True:
        try:
            Aqp = cvxMatrix(A+A.T)
            bqp = cvxMatrix(b)
            Gqp = cvxMatrix(np.vstack((-A, -np.eye(A.shape[0]))))
            hqp = cvxMatrix(np.hstack((b.T, np.zeros(A.shape[0]))))
            timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)
            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiters'] = 100
            # cvxSolvers.options['kktreg'] = 1e-6
            # cvxSolvers.options['refinement'] = 10
            solution = cvxSolvers.qp(Aqp, bqp, Gqp, hqp)
            xqp = np.array(solution['x']).flatten()
            # xqp = np.array(cvxSolvers.qp(Aqp, bqp, Gqp, hqp)['x']).flatten()
            x = xqp.copy()
            print x.shape[0]
            print x
            zqp = np.dot(A,x)+b
            print "force value: ", np.dot(x, zqp)
        except Exception, e:
            # print e
            pass

    if solver == 'qpOASES':
        # solve using qpOASES
        QQ = A+A.T
        pp = b
        # GG = np.vstack((A, np.eye(A.shape[0])))
        # hh = np.hstack((-b.T, np.zeros(A.shape[0])))
        GG = A.copy()
        hh = -b

        # bp::list qp(const object &H, const object &g, const object &A, const object &lb, const object &ub, const object &lbA, const object ubA, int nWSR)
        lb = [0.]*A.shape[0]
        xqpos = qpos.qp(QQ, pp, GG, lb, None, hh, None, 1000)
        x = np.array(xqpos)
        zqp = np.dot(A,x)+b
        print np.dot(x, zqp)
        # print xqpos
        # x = xqpos.copy()
        pass


    normalForce = x[:contactNum]
    # tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    # minTangenVel = x[contactNum + numFrictionBases*contactNum:]

    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]
        '''
        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
        '''
        # print force
        forces.append(force)

    # repairForces(forces, contactPositions)
    # print forces
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)
    return bodyIDs, contactPositions, contactPositionsLocal, forces, timeStamp


def calcLCPForcesHD(motion, world, model, bodyIDsToCheck, mu, ddth, tau, numFrictionBases=8, solver='qp', hdAccMask=None):
    timeStamp = []
    timeIndex = 0
    prevTime = time.time()

    # model = VpControlModel
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, contactVelocities, JTN, JTD, E, N, D \
        = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)

    if contactNum == 0:
    # if contactNum <= 2:
        return [], [], [], None, None
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)

    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF, totalDOF))
    invMc = np.zeros(totalDOF)

    model.getInverseEquationOfMotion(invM, invMc)

    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)

    # pdb.set_trace()
    # A =[ A11,  A12, 0]
    #   [ A21,  A22, E]
    #   [ mus, -E.T, 0]

    factor = 1.
    # A, b = getLCPMatrixHD(world, model, invM, invMc, mu, ddth[6:], contactNum, contactPositions, JTN, JTD, E, factor)
    A, b = getLCPMatrixGenHD(world, model, invM, invMc, mu, ddth, tau, contactNum, contactPositions, contactVelocities, JTN, JTD, E, factor, hdAccMask)

    # lo = np.zeros(A.shape[0])
    lo = 0.*np.ones(A.shape[0])
    hi = 1000000. * np.ones(A.shape[0])
    x = 0.*np.ones(A.shape[0])

    # normalizeMatrix(A, b)
    # print A[0]

    if solver == 'bulletLCP':
        # solve using bullet LCP solver
        lcpSolver = lcp.LemkeSolver()
        # lcpSolver = lcpD.DantzigSolver()
        lcpSolver.solve(A.shape[0], A, b, x, lo, hi)

    if solver == 'openOptLCP':
        # solve using openOpt LCP solver
        # p = openLCP(A, b)
        # r = p.solve('lcpsolve')
        # f_opt, x_opt = r.ff, r.xf
        # w, x = x_opt[x_opt.size/2:], x_opt[:x_opt.size/2]
        pass

    if solver == 'nqp':
        # solve using cvxopt Nonlinear Optimization with linear constraint
        Acp = cvxMatrix(A)
        bcp = cvxMatrix(b)
        Hcp = cvxMatrix(A+A.T)
        Gcp = cvxMatrix(np.vstack((-A, -np.eye(A.shape[0]))))
        hcp = cvxMatrix(np.hstack((b.T, np.zeros(A.shape[0]))))

        def F(xin=None, z=None):
            if xin is None:
                return 0, cvxMatrix(1., (A.shape[1], 1))
            for j in range(len(np.array(xin))):
                if xin[j] < 0.:
                    return None, None
            f = xin.T*(Acp*xin+bcp)
            # TODO:
            # check!!!
            Df = Hcp*xin + bcp
            if z is None:
                return f, Df.T
            H = Hcp
            return f, Df.T, H
        solution = cvxSolvers.cp(F, Gcp, hcp)
        xcp = np.array(solution['x']).flatten()
        x = xcp.copy()

    if solver == 'qp':
        # solve using cvxopt QP
        # if True:
        try:
            Aqp = cvxMatrix(A+A.T)
            bqp = cvxMatrix(b)
            Gqp = cvxMatrix(np.vstack((-A, -np.eye(A.shape[0]))))
            hqp = cvxMatrix(np.hstack((b.T, np.zeros(A.shape[0]))))
            timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)
            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiters'] = 100
            # cvxSolvers.options['kktreg'] = 1e-6
            # cvxSolvers.options['refinement'] = 10
            solution = cvxSolvers.qp(Aqp, bqp, Gqp, hqp)
            xqp = np.array(solution['x']).flatten()
            # xqp = np.array(cvxSolvers.qp(Aqp, bqp, Gqp, hqp)['x']).flatten()
            x = xqp.copy()
            # print x.shape[0]
            # print "x: ", x
            # zqp = np.dot(A,x)+b
            # print "z: ", zqp
            # print "El: ", np.dot(E, x[contactNum + numFrictionBases*contactNum:])
            # print "Ep: ", np.dot(E.T, x[contactNum:contactNum + numFrictionBases*contactNum])
            # print "force value: ", np.dot(x, zqp)
        except Exception, e:
            # print e
            pass

    if solver == 'qpOASES':
        # solve using qpOASES
        QQ = A+A.T
        pp = b
        # GG = np.vstack((A, np.eye(A.shape[0])))
        # hh = np.hstack((-b.T, np.zeros(A.shape[0])))
        GG = A.copy()
        hh = -b

        # bp::list qp(const object &H, const object &g, const object &A, const object &lb, const object &ub, const object &lbA, const object ubA, int nWSR)
        lb = [0.]*A.shape[0]
        xqpos = qpos.qp(QQ, pp, GG, lb, None, hh, None, 1000, False, "NONE")
        x = np.array(xqpos)
        zqp = np.dot(A,x)+b
        print np.dot(x, zqp)
        # print xqpos
        # x = xqpos.copy()
        pass


    normalForce = x[:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    # tangenForce = np.zeros_like(x[contactNum:contactNum + numFrictionBases*contactNum])
    minTangenVel = x[contactNum + numFrictionBases*contactNum:]



    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]

        # if(tangentialRelVel < _lockingVel)
        #     frictionForce *= tangentialRelVel/_lockingVel;
        tangenVel = deepcopy(contactVelocities[cIdx])
        tangenVel[1] = 0.
        tangenRatio = npl.norm(tangenVel)/0.02


        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            if tangenRatio > 1.:
                force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
            else:
                force += tangenForce[cIdx*numFrictionBases + fcIdx] * d * tangenRatio
        # print force
        forces.append(force)

    # repairForces(forces, contactPositions)
    # print forces
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)
    return bodyIDs, contactPositions, contactPositionsLocal, forces, timeStamp

def calcLCPControl(motion, world, model, bodyIDsToCheck, mu, totalForce, weights, tau0=None, numFrictionBases=8):
    # tau0 = None
    # model = VpControlModel
    # numFrictionBases = 8
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, contactVelocities, JTN, JTD, E, N, D \
        = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)
    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None, None

    wLCP = weights[0]
    wTorque = weights[1]
    wForce = weights[2]

    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF, totalDOF))
    invMc = np.zeros(totalDOF)
    model.getInverseEquationOfMotion(invM, invMc)

    # Jc = np.zeros(())
    # N = np.zeros(())
    # D = np.zeros(())
    # E = np.zeros(())

    M = npl.inv(invM)
    c = np.dot(M, invMc)

    Mtmp = np.dot(M, M.T)+np.eye(M.shape[0])
    # Mtmp[:6, :6] -= np.eye(6)

    pinvM = npl.inv(Mtmp)
    pinvM0 = np.dot(M.T, pinvM)
    pinvM1 = -pinvM

    h = world.GetTimeStep()
    invh = 1./h
    mus = mu * np.eye(contactNum)
    temp_NM = JTN.T.dot(pinvM0)
    temp_DM = JTD.T.dot(pinvM0)

    A11 = h*temp_NM.dot(JTN)
    A12 = h*temp_NM.dot(JTD)
    A21 = h*temp_DM.dot(JTN)
    A22 = h*temp_DM.dot(JTD)

    factor = 1.
    # A, b = getLCPMatrix(world, model, pinvM0, c, mu, tau0, contactNum, contactPositions, JTN, JTD, E, factor)

    A1 = np.concatenate((A11, A12,  np.zeros((A11.shape[0], E.shape[1]))),   axis=1)
    A2 = np.concatenate((A21, A22,  E),                                      axis=1)
    A3 = np.concatenate((mus, -E.T, np.zeros((mus.shape[0], E.shape[1]))),  axis=1)
    A = np.concatenate((A1,
                        A2,
                        A3), axis=0) * factor
    # A = 0.01 * np.eye(A.shape[0])*factor
    # print npl.eigvals(A+A.T)
    # npl.eigvals(A+A.T)

    # bx= h * (M*qdot_0 + tau - c)
    # b =[N.T * Jc * invM * kx]
    #   [D.T * Jc * invM * kx]
    #   [0]

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)
    qdot_0 = np.asarray(qdot_0)
    if tau0 is None:
        tau0 = np.zeros(np.shape(qdot_0))

    # non-penentration condition
    # b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)

    # improved non-penentration condition : add position condition
    penDepth = 0.003
    bPenDepth = np.zeros(A1.shape[0])
    for i in range(contactNum):
        if abs(contactPositions[i][1]) > penDepth:
            bPenDepth[i] = contactPositions[i][1] + penDepth

    b1 = JTN.T.dot(qdot_0) + h*temp_NM.dot(tau0 - c) #+ 0.5*invh*bPenDepth
    b2 = JTD.T.dot(qdot_0) + h*temp_DM.dot(tau0 - c)
    b3 = np.zeros(mus.shape[0])
    b = np.hstack((np.hstack((b1, b2)), b3)) * factor

    # lo = np.zeros(A.shape[0])
    lo = 0.*np.ones(A.shape[0])
    hi = 1000000. * np.ones(A.shape[0])
    x = 100.*np.ones(A.shape[0])

    # normalizeMatrix(A, b)

    # for torque equality constraints computation
    modelCom = model.getCOM()
    rcN = np.zeros((3, N.shape[1]))
    rcD = np.zeros((3, D.shape[1]))
    for cIdx in range(len(contactPositions)):
        r = contactPositions[cIdx] - modelCom
        rcN[:3, cIdx] = np.cross(r, N[:3, cIdx])
        for fbIdx in range(numFrictionBases):
            dIdx = numFrictionBases * cIdx + fbIdx
            rcD[:3, dIdx] = np.cross(r, D[:3, dIdx])

    if True:
        # if True:
        try:
            # Qqp = cvxMatrix(A+A.T)
            # pqp = cvxMatrix(b)

            # Qtauqp = np.hstack((np.dot(pinvM1[:6], np.hstack((JTN, JTD))), np.zeros_like(N[:6])))
            # ptauqp = np.dot(pinvM1[:6], (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0[:6])

            # Qqp = cvxMatrix(2.*A + wTorque * np.dot(Qtauqp.T, Qtauqp))
            # pqp = cvxMatrix(b + wTorque * np.dot(ptauqp.T, Qtauqp))

            Qfqp = np.concatenate((N[:3], D[:3], np.zeros_like(N[:3])), axis=1)
            pfqp = -totalForce[:3]

            Qfqp = np.concatenate((N[1:2], D[1:2], np.zeros_like(N[1:2])), axis=1)
            pfqp = -totalForce[1:2]

            # TODO:
            # add tau norm term ||tau||^2
            # and momentum derivative term
            QtauNormqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            ptauNormqp = np.dot(pinvM1, (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0)

            QqNormqp = np.hstack((np.dot(pinvM0, np.hstack((JTN, JTD))), np.zeros((pinvM0.shape[0], N.shape[1]))))
            pqNormqp = np.dot(pinvM0, (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0)

            # Qqp = cvxMatrix(2.*A + wTorque * np.dot(Qfqp.T, Qfqp) + np.dot(QqNormqp.T, QqNormqp))
            # pqp = cvxMatrix(b + wTorque * np.dot(pfqp.T, Qfqp) + np.dot(pqNormqp.T, QqNormqp))

            Qqp = cvxMatrix(2.*A + wForce * np.dot(Qfqp.T, Qfqp) + wTorque * np.dot(QtauNormqp.T, QtauNormqp))
            pqp = cvxMatrix(b + wForce * np.dot(pfqp.T, Qfqp) + wTorque * np.dot(ptauNormqp.T, QtauNormqp))

            # Qqp = cvxMatrix(2.*A + wForce * np.dot(Qfqp.T, Qfqp) )
            # pqp = cvxMatrix(b + wForce * np.dot(pfqp.T, Qfqp))


            equalConstForce = False
            G = np.vstack((-A, -np.eye(A.shape[0])))
            hnp = np.hstack((b.T, np.zeros(A.shape[0])))

            if False and not equalConstForce:
                constMu = .1
                constFric = totalForce[1]*constMu
                totalForceMat = np.concatenate((N, D, np.zeros_like(N)), axis=1)
                G = np.concatenate((G, totalForceMat[:3], totalForceMat[:3]), axis=0)
                G[-6] *= -1.
                G[-5] *= -1.
                G[-4] *= -1.
                hnp = np.hstack((hnp, np.zeros(6)))
                hnp[-6] = -totalForce[0] - constFric
                hnp[-5] = -totalForce[1] * .9
                hnp[-4] = -totalForce[2] - constFric
                hnp[-3] = totalForce[0] + constFric
                hnp[-2] = totalForce[1] * 1.1
                hnp[-1] = totalForce[2] + constFric


            # G = np.vstack((G, np.hstack((np.ones((2, N.shape[1])), np.zeros((2, D.shape[1]+N.shape[1]))))))
            # G[-2] *= -1.
            # hnp = np.hstack((hnp, np.zeros(2)))
            # hnp[-2] = -totalForce[1] * .9
            # hnp[-1] = totalForce[1] * 1.1

            # root torque 0 condition as inequality constraint
            Atauqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            btauqp = np.dot(pinvM1, (np.asarray(c)-np.asarray(tau0))) - np.array(tau0)

            # G = np.concatenate((G, -Atauqp, Atauqp), axis=0)
            # hnp = np.hstack((hnp, np.hstack((-btauqp, btauqp))))
            # hnp[-2*pinvM1.shape[0]:] += 1. * np.ones(2*pinvM1.shape[0])

            Gqp = cvxMatrix(G)
            hqp = cvxMatrix(hnp)

            # check correctness of equality constraint
            # tau = np.dot(pinvM1, -c + tau0 + np.dot(JTN, normalForce) + np.dot(JTD, tangenForce))
            # tau = pinvM1*JTN*theta + pinvM1*JTD*phi + pinvM1*tau0 - pinvM1*b + tau0
            Atauqp = np.hstack((np.dot(pinvM1[:6], np.hstack((JTN, JTD))), np.zeros_like(N[:6])))
            btauqp = np.dot(pinvM1[:6], (np.asarray(c)-np.asarray(tau0))) - np.asarray(tau0[:6])
            # Atauqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            # btauqp = np.dot(pinvM1, (np.asarray(c)-np.asarray(tau0))) - np.asarray(tau0)

            AextTorqp = np.concatenate((rcN, rcD, np.zeros_like(N[:3])), axis=1)
            bextTorqp = totalForce[3:]


            # Atauqp = np.vstack((Atauqp, AextTorqp))
            # btauqp = np.hstack((btauqp, bextTorqp))

            if equalConstForce:
                Atauqp = cvxMatrix(np.vstack((np.concatenate((N[1:2], D[1:2], np.zeros(N[1:2].shape))), Atauqp)))
                btauqp = cvxMatrix(np.hstack((np.array(totalForce[1]), btauqp)))
                # Atauqp = cvxMatrix(np.vstack((np.concatenate((N[1:2], D[1:2], np.zeros(N[1:2].shape), AextTorqp)), Atauqp)))
                # btauqp = cvxMatrix(np.concatenate((np.array(totalForce[1]), bextTorqp, btauqp), axis=1))

            Aqp = cvxMatrix(Atauqp)
            bqp = cvxMatrix(btauqp)

            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiters'] = 100
            cvxSolvers.options['refinement'] = 1
            xqp = np.array(cvxSolvers.qp(Qqp, pqp, Gqp, hqp, Aqp, bqp)['x']).flatten()
            # xqp = np.asarray(cvxSolvers.qp(Qqp, pqp, Gqp, hqp)['x']).flatten()
            # print "x: ", x
            # zqp = np.dot(A, xqp).T + b
            # print "QP z: ", np.dot(xqp, zqp)
            # if np.dot(xqp, zqp) < np.dot(x, z):
            x = xqp.copy()
        except Exception, e:
            print 'LCPControl!!', e
            pass

    normalForce = x[:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    minTangenVel = x[contactNum + numFrictionBases*contactNum:]

    tau = np.dot(pinvM1, -c + tau0 + np.dot(JTN, normalForce) + np.dot(JTD, tangenForce)) + np.array(tau0)

    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]

        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
        # print force
        forces.append(force)

    # repairForces(forces, contactPositions)
    # print forces
    return bodyIDs, contactPositions, contactPositionsLocal, forces, tau


def calcLCPbasicControl(motion, world, model, bodyIDsToCheck, mu, totalForce, weights, tau0=None, numFrictionBases=8):
    # tau0 = None
    # model = VpControlModel
    # numFrictionBases = 8
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, contactVelocities, JTN, JTD, E, N, D \
        = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)
    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None, None

    wLCP = weights[0]
    wTorque = weights[1]
    wForce = weights[2]

    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF, totalDOF))
    invMc = np.zeros(totalDOF)
    model.getInverseEquationOfMotion(invM, invMc)

    # M = npl.inv(invM)
    # c = np.dot(M, invMc)

    # Mtmp = np.dot(M, M.T)+np.eye(M.shape[0])
    # Mtmp[:6, :6] -= np.eye(6)

    # pinvM = npl.inv(Mtmp)

    h = world.GetTimeStep()
    invh = 1./h
    mus = mu * np.eye(contactNum)
    temp_NM = JTN.T.dot(invM)
    temp_DM = JTD.T.dot(invM)

    A00 = np.eye(totalDOF)
    A10 = h*temp_NM
    A11 = h*temp_NM.dot(JTN)
    A12 = h*temp_NM.dot(JTD)
    A20 = h*temp_DM
    A21 = h*temp_DM.dot(JTN)
    A22 = h*temp_DM.dot(JTD)

    factor = 1.
    # A, b = getLCPMatrix(world, model, pinvM0, c, mu, tau0, contactNum, contactPositions, JTN, JTD, E, factor)

    # A0 = np.concatenate((A00, np.zeros((A00.shape[0], A11.shape[1]+A12.shape[1]+E.shape[1]))), axis=1)
    A0 = np.zeros((A00.shape[0], A00.shape[1] + A11.shape[1]+A12.shape[1]+E.shape[1]))
    A1 = np.concatenate((A10, A11, A12,  np.zeros((A11.shape[0], E.shape[1]))),   axis=1)
    A2 = np.concatenate((A20, A21, A22,  E),                                      axis=1)
    A3 = np.concatenate((np.zeros((mus.shape[0], A00.shape[1])), 1.*mus, -1.*E.T, np.zeros((mus.shape[0], E.shape[1]))),  axis=1)
    A_ori = np.concatenate((A0,
                        wLCP*A1,
                        wLCP*A2,
                        wLCP*A3), axis=0) * factor

    A = A_ori.copy()
    # A = A_ori + 0.01 * np.eye(A_ori.shape[0])*factor

    # bx= h * (M*qdot_0 + tau - c)
    # b =[N.T * Jc * invM * kx]
    #   [D.T * Jc * invM * kx]
    #   [0]

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)
    qdot_0 = np.asarray(qdot_0)
    if tau0 is None:
        tau0 = np.zeros(np.shape(qdot_0))

    # non-penentration condition
    # b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)

    # improved non-penentration condition : add position condition
    penDepth = 0.003
    bPenDepth = np.zeros(A1.shape[0])
    for i in range(contactNum):
        if abs(contactPositions[i][1]) > penDepth:
            bPenDepth[i] = contactPositions[i][1] + penDepth

    b0 = np.zeros(A00.shape[0])
    b1 = JTN.T.dot(qdot_0 - h*invMc)# + 0.5*invh*bPenDepth
    b2 = JTD.T.dot(qdot_0 - h*invMc)
    b3 = np.zeros(mus.shape[0])
    b = np.hstack((wTorque*b0, wLCP*np.hstack((np.hstack((b1, b2)), b3)))) * factor

    x = 100.*np.ones(A.shape[0])
    zqp = np.zeros(A.shape[0])
    Qfqp = None
    pfqp = None

    # for torque equality constraints computation
    modelCom = model.getCOM()
    rcN = np.zeros((3, N.shape[1]))
    rcD = np.zeros((3, D.shape[1]))
    for cIdx in range(len(contactPositions)):
        r = contactPositions[cIdx] - modelCom
        rcN[:3, cIdx] = np.cross(r, N[:3, cIdx])
        for fbIdx in range(numFrictionBases):
            dIdx = numFrictionBases * cIdx + fbIdx
            rcD[:3, dIdx] = np.cross(r, D[:3, dIdx])

    if True:
        # if True:
        try:
            # Qqp = cvxMatrix(A+A.T)
            # pqp = cvxMatrix(b)

            # Qtauqp = np.hstack((np.dot(pinvM1[:6], np.hstack((JTN, JTD))), np.zeros_like(N[:6])))
            # ptauqp = np.dot(pinvM1[:6], (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0[:6])

            Qtauqp = np.hstack((np.eye(totalDOF), np.zeros((A00.shape[0], A11.shape[1]+A12.shape[1]+E.shape[1]))))
            ptauqp = np.zeros(totalDOF)

            Q2dotqp = np.hstack((np.dot(invM, np.concatenate((wTorque* np.eye(totalDOF), JTN, JTD), axis=1)), np.zeros((A0.shape[0], E.shape[1])) ))
            p2dotqp = -invMc.copy()

            # Qqp = cvxMatrix(2.*A + wTorque * np.dot(Qtauqp.T, Qtauqp))
            # pqp = cvxMatrix(b + wTorque * np.dot(ptauqp.T, Qtauqp))

            Qfqp = np.concatenate((np.zeros((3, totalDOF)), N[:3], D[:3], np.zeros_like(N[:3])), axis=1)
            pfqp = -totalForce[:3]

            # Qfqp = np.concatenate((np.zeros((1, totalDOF)),N[1:2], D[1:2], np.zeros_like(N[1:2])), axis=1)
            # pfqp = -totalForce[1:2]

            # TODO:
            # add momentum term
            # QtauNormqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            # ptauNormqp = np.dot(pinvM1, (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0)

            # QqNormqp = np.hstack((np.dot(pinvM0, np.hstack((JTN, JTD))), np.zeros((pinvM0.shape[0], N.shape[1]))))
            # pqNormqp = np.dot(pinvM0, (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0)

            # Qqp = cvxMatrix(2.*A + wTorque * np.dot(Qfqp.T, Qfqp) + np.dot(QqNormqp.T, QqNormqp))
            # pqp = cvxMatrix(b + wTorque * np.dot(pfqp.T, Qfqp) + np.dot(pqNormqp.T, QqNormqp))

            # Qqp = cvxMatrix(2.*A + wForce * np.dot(Qfqp.T, Qfqp) + wTorque * np.dot(QtauNormqp.T, QtauNormqp))
            # pqp = cvxMatrix(b + wForce * np.dot(pfqp.T, Qfqp) + wTorque * np.dot(ptauNormqp.T, QtauNormqp))

            # objective : LCP
            Qqp = cvxMatrix(A+A.T )
            pqp = cvxMatrix(b)

            QQ = A+A.T
            pp = b.copy()

            # objective : torque
            if True:
                Qqp += cvxMatrix(wTorque * np.dot(Qtauqp.T, Qtauqp) )
                pqp += cvxMatrix(wTorque * np.dot(ptauqp.T, Qtauqp))

                QQ += wTorque * np.dot(Qtauqp.T, Qtauqp)
                pp += wTorque * np.dot(ptauqp.T, Qtauqp)

            # objective : q2dot
            if False:
                Qqp += cvxMatrix(wTorque * np.dot(Q2dotqp.T, Q2dotqp) )
                pqp += cvxMatrix(wTorque * np.dot(p2dotqp.T, Q2dotqp))

                QQ += wTorque * np.dot(Q2dotqp.T, Q2dotqp)
                pp += wTorque * np.dot(p2dotqp.T, Q2dotqp)

            # objective : force
            if True:
                Qqp += cvxMatrix(wForce * np.dot(Qfqp.T, Qfqp) )
                pqp += cvxMatrix(wForce * np.dot(pfqp.T, Qfqp))

                QQ += wForce * np.dot(Qfqp.T, Qfqp)
                pp += wForce * np.dot(pfqp.T, Qfqp)



            equalConstForce = False
            G = np.vstack((-A[totalDOF:], -np.eye(A.shape[0])[totalDOF:]))
            hnp = np.hstack((b[totalDOF:].T, np.zeros(A.shape[0])[totalDOF:]))
            # G = np.vstack((-A_ori[totalDOF:], -np.eye(A_ori.shape[0])[totalDOF:]))
            # hnp = np.hstack((b[totalDOF:].T, np.zeros(A_ori.shape[0])[totalDOF:]))

            if False and not equalConstForce:
                # 3direction
            # if not equalConstForce:
                constMu = .1
                constFric = totalForce[1]*constMu
                totalForceMat = np.concatenate((np.zeros((6, totalDOF)), N, D, np.zeros_like(N)), axis=1)
                G = np.concatenate((G, -totalForceMat[:3], totalForceMat[:3]), axis=0)
                hnp = np.hstack((hnp, np.zeros(6)))
                hnp[-6] = -totalForce[0] - constFric
                hnp[-5] = -totalForce[1] * 0.9
                hnp[-4] = -totalForce[2] - constFric
                hnp[-3] = totalForce[0] + constFric
                hnp[-2] = totalForce[1] * 1.1
                hnp[-1] = totalForce[2] + constFric

            if False and not equalConstForce:
                # just normal direction
                # if not equalConstForce:
                constMu = .1
                constFric = totalForce[1]*constMu
                totalForceMat = np.concatenate((np.zeros((6, totalDOF)), N, D, np.zeros_like(N)), axis=1)
                G = np.concatenate((G, -totalForceMat[1:2], totalForceMat[1:2]), axis=0)
                hnp = np.hstack((hnp, np.zeros(2)))
                hnp[-2] = -totalForce[1] * 0.9
                hnp[-1] = totalForce[1] * 1.1

            # G = np.vstack((G, np.hstack((np.ones((2, N.shape[1])), np.zeros((2, D.shape[1]+N.shape[1]))))))
            # G[-2] *= -1.
            # hnp = np.hstack((hnp, np.zeros(2)))
            # hnp[-2] = -totalForce[1] * .9
            # hnp[-1] = totalForce[1] * 1.1

            # root torque 0 condition as inequality constraint
            # Atauqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            # btauqp = np.dot(pinvM1, (np.asarray(c)-np.asarray(tau0))) - np.array(tau0)

            # G = np.concatenate((G, -Atauqp, Atauqp), axis=0)
            # hnp = np.hstack((hnp, np.hstack((-btauqp, btauqp))))
            # hnp[-2*pinvM1.shape[0]:] += 1. * np.ones(2*pinvM1.shape[0])

            Gqp = cvxMatrix(G)
            hqp = cvxMatrix(hnp)

            # check correctness of equality constraint
            # tau = np.dot(pinvM1, -c + tau0 + np.dot(JTN, normalForce) + np.dot(JTD, tangenForce))
            # tau = pinvM1*JTN*theta + pinvM1*JTD*phi + pinvM1*tau0 - pinvM1*b + tau0
            # Atauqp = np.hstack((np.dot(pinvM1[:6], np.hstack((JTN, JTD))), np.zeros_like(N[:6])))
            # btauqp = np.dot(pinvM1[:6], (np.asarray(c)-np.asarray(tau0))) - np.asarray(tau0[:6])
            # Atauqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            # btauqp = np.dot(pinvM1, (np.asarray(c)-np.asarray(tau0))) - np.asarray(tau0)
            Atauqp = np.hstack((np.eye(6), np.zeros((6, A.shape[1]-6))))
            btauqp = np.zeros((6))

            AextTorqp = np.concatenate((rcN, rcD, np.zeros_like(N[:3])), axis=1)
            bextTorqp = totalForce[3:]


            # Atauqp = np.vstack((Atauqp, AextTorqp))
            # btauqp = np.hstack((btauqp, bextTorqp))

            if equalConstForce:
                Atauqp = cvxMatrix(np.vstack((np.concatenate((N[1:2], D[1:2], np.zeros(N[1:2].shape))), Atauqp)))
                btauqp = cvxMatrix(np.hstack((np.array(totalForce[1]), btauqp)))
                # Atauqp = cvxMatrix(np.vstack((np.concatenate((N[1:2], D[1:2], np.zeros(N[1:2].shape), AextTorqp)), Atauqp)))
                # btauqp = cvxMatrix(np.concatenate((np.array(totalForce[1]), bextTorqp, btauqp), axis=1))

            Aqp = cvxMatrix(Atauqp)
            bqp = cvxMatrix(btauqp)

            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiters'] = 100
            cvxSolvers.options['refinement'] = 1
            cvxSolvers.options['kktsolver'] = "robust"
            xqp = np.array(cvxSolvers.qp(Qqp, pqp, Gqp, hqp, Aqp, bqp)['x']).flatten()
            x = xqp.copy()

            # print "x: ", x
            # zqp = np.dot(A_ori, xqp) + b
            # zqp = np.dot(A, xqp) + b
            # print "QP z: ", np.dot(xqp, zqp)
            # if np.dot(xqp, zqp) < np.dot(x, z):


            # bp::list qp(const object &H, const object &g, const object &A, const object &lb, const object &ub, const object &lbA, const object ubA, int nWSR)
            # print qpos.qp


            # lb = [-1000.]*(totalDOF-6)
            # lb.extend([0.]*(A.shape[0]-totalDOF))
            # xqpos = qpos.qp(QQ[6:, 6:], pp[6:], G[:, 6:], lb, None, None, hnp, 200, False, "NONE")
            # xtmp = [0.]*6
            # xtmp.extend(xqpos[:])
            # x = np.array(xtmp)

            # lb = [-1000.]*totalDOF
            # lb.extend([0.]*(A.shape[0]-totalDOF))
            # xqpos = qpos.qp(QQ, pp, G, lb, None, None, hnp, 200, False, "NONE")
            # x = np.array(xqpos)

            zqp = np.dot(A, x) + b

            '''
            cons = []

            # for ii in range(A.shape[0]):
            #     cons.append({'type': 'eq',
            #                  'fun' : lambda xx: np.dot(Atauqp[i], xx)
            #                  #,'jac' : lambda xx: Atauqp[i]
            #     })

            for ii in range(G.shape[0]):
                cons.append({'type':'ineq',
                             'fun' : lambda xx: -np.dot(G[:,6:][i], xx)+hnp[i]
                            #,'jac' : lambda xx: -G[i]
                             })

            L-BFGS-B
            TNC
            COBYLA
            SLSQP
            res = spopt.minimize(lambda xx: np.dot(xx, .5*np.dot(QQ[6:, 6:], xx)+pp[6:]), xqp[6:],
                                 # jac=lambda xx: np.dot(np.dot(QQ, xx)+pp),
                                 method='SLSQP', constraints=cons, options={'disp': True})
            # res = spopt.minimize(lambda xx: np.dot(xx, .5*np.dot(QQ, xx)+pp) , xqp)
            print res.x
            # print res.hess
            # print res.message

            '''


        except Exception, e:
            print 'LCPbasicControl!!', e
            pass

    def refine(xx):
        for i in range(len(xx)):
            if xx[i] < 0.001:
                xx[i] = 0.
        return xx

    tau = x[:totalDOF]
    normalForce = x[totalDOF:totalDOF+contactNum]
    tangenForce = x[totalDOF+contactNum:totalDOF+contactNum + numFrictionBases*contactNum]
    minTangenVel = x[totalDOF+contactNum + numFrictionBases*contactNum:]

    # for i in range(len(tau)):
    #     tau[i] = 10.*x[i]


    # print np.array(tau)

    # zqp = np.dot(A, x)+b

    lcpValue = np.dot(x[totalDOF:], zqp[totalDOF:])
    tauValue = np.dot(tau, tau)
    Q2dotqpx = np.dot(Q2dotqp, x)+p2dotqp
    q2dotValue = np.dot(Q2dotqpx, Q2dotqpx)
    Qfqpx = np.dot(Qfqp, x)+pfqp
    forceValue = np.dot(Qfqpx, Qfqpx)
    print "LCP value: ", wLCP, lcpValue/wLCP, lcpValue
    print "tau value: ", wTorque, tauValue, wTorque*tauValue
    # print "q2dot value: ", wTorque, q2dotValue, wTorque*q2dotValue
    print "For value: ", wForce, forceValue, wForce*forceValue
    # print "x: ", x[totalDOF:]
    # print "z: ", zqp[totalDOF:]
    # print "b: ", b[totalDOF:]
    # print "elevalue: ", np.multiply(x[totalDOF:], zqp[totalDOF:])

    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]

        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
        # print force
        forces.append(force)


    # repairForces(forces, contactPositions)
    # print forces
    return bodyIDs, contactPositions, contactPositionsLocal, forces, tau


def calcLCPbasicControl2(motion, world, model, bodyIDsToCheck, mu, totalForce, wForce, wTorque, tau0=None, numFrictionBases=8):
    # tau0 = None
    # model = VpControlModel
    # numFrictionBases = 8
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, contactVelocities, JTN, JTD, E, N, D \
        = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)
    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None, None

    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF, totalDOF))
    invMc = np.zeros(totalDOF)
    model.getInverseEquationOfMotion(invM, invMc)

    # Jc = np.zeros(())
    # N = np.zeros(())
    # D = np.zeros(())
    # E = np.zeros(())

    M = npl.inv(invM)
    c = np.dot(M, invMc)

    Mtmp = np.dot(M, M.T)+np.eye(M.shape[0])
    # Mtmp[:6, :6] -= np.eye(6)

    pinvM = npl.inv(Mtmp)

    h = world.GetTimeStep()
    invh = 1./h
    mus = mu * np.eye(contactNum)
    temp_NM = JTN.T.dot(pinvM)
    temp_DM = JTD.T.dot(pinvM)

    A10 = wTorque * np.eye(totalDOF)
    A20 = h*temp_NM
    A21 = h*temp_NM.dot(JTN)
    A22 = h*temp_NM.dot(JTD)
    A30 = h*temp_DM
    A31 = h*temp_DM.dot(JTN)
    A32 = h*temp_DM.dot(JTD)

    factor = 1.
    # A, b = getLCPMatrix(world, model, pinvM0, c, mu, tau0, contactNum, contactPositions, JTN, JTD, E, factor)

    A1 = np.concatenate((A10, np.zeros((A10.shape[0], A21.shape[1]+A22.shape[1]+E.shape[1]))), axis=1)
    A2 = np.concatenate((A20, A21, A22,  np.zeros((A21.shape[0], E.shape[1]))),   axis=1)
    A3 = np.concatenate((A30, A31, A32,  E),                                      axis=1)
    A4 = np.concatenate((np.zeros((mus.shape[0], A10.shape[1])), mus, -E.T, np.zeros((mus.shape[0], E.shape[1]))), axis=1)
    A0 = np.zeros_like(A1)

    A = np.concatenate((A0, A1, A2, A3, A4), axis=0) * factor

    # A = 0.01 * np.eye(A.shape[0])*factor

    # bx= h * (M*qdot_0 + tau - c)
    # b =[N.T * Jc * invM * kx]
    #   [D.T * Jc * invM * kx]
    #   [0]

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)
    qdot_0 = np.asarray(qdot_0)
    if tau0 is None:
        tau0 = np.zeros(np.shape(qdot_0))

    # non-penentration condition
    # b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)

    # improved non-penentration condition : add position condition
    penDepth = 0.003
    bPenDepth = np.zeros(A1.shape[0])
    for i in range(contactNum):
        if abs(contactPositions[i][1]) > penDepth:
            bPenDepth[i] = contactPositions[i][1] + penDepth

    b0 = np.zeros(A00.shape[0])
    b1 = JTN.T.dot(qdot_0) - h*temp_NM.dot(c) + 0.5*invh*bPenDepth
    b2 = JTD.T.dot(qdot_0) - h*temp_DM.dot(c)
    b3 = np.zeros(mus.shape[0])
    b = np.hstack((b0, np.hstack((np.hstack((b1, b2)), b3)))) * factor

    x = 100.*np.ones(A.shape[0])

    # for torque equality constraints computation
    modelCom = model.getCOM()
    rcN = np.zeros((3, N.shape[1]))
    rcD = np.zeros((3, D.shape[1]))
    for cIdx in range(len(contactPositions)):
        r = contactPositions[cIdx] - modelCom
        rcN[:3, cIdx] = np.cross(r, N[:3, cIdx])
        for fbIdx in range(numFrictionBases):
            dIdx = numFrictionBases * cIdx + fbIdx
            rcD[:3, dIdx] = np.cross(r, D[:3, dIdx])

    if True:
        try:
            # Qqp = cvxMatrix(A+A.T)
            # pqp = cvxMatrix(b)

            # Qtauqp = np.hstack((np.dot(pinvM1[:6], np.hstack((JTN, JTD))), np.zeros_like(N[:6])))
            # ptauqp = np.dot(pinvM1[:6], (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0[:6])

            # Qqp = cvxMatrix(2.*A + wTorque * np.dot(Qtauqp.T, Qtauqp))
            # pqp = cvxMatrix(b + wTorque * np.dot(ptauqp.T, Qtauqp))

            Qfqp = np.concatenate((np.zeros((3, totalDOF)), N[:3], D[:3], np.zeros_like(N[:3])), axis=1)
            pfqp = -totalForce[:3]

            # Qfqp = np.concatenate((np.zeros((1, totalDOF)),N[1:2], D[1:2], np.zeros_like(N[1:2])), axis=1)
            # pfqp = -totalForce[1:2]

            # TODO:
            # add momentum term
            # QtauNormqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            # ptauNormqp = np.dot(pinvM1, (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0)

            # QqNormqp = np.hstack((np.dot(pinvM0, np.hstack((JTN, JTD))), np.zeros((pinvM0.shape[0], N.shape[1]))))
            # pqNormqp = np.dot(pinvM0, (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0)

            # Qqp = cvxMatrix(2.*A + wTorque * np.dot(Qfqp.T, Qfqp) + np.dot(QqNormqp.T, QqNormqp))
            # pqp = cvxMatrix(b + wTorque * np.dot(pfqp.T, Qfqp) + np.dot(pqNormqp.T, QqNormqp))

            # Qqp = cvxMatrix(2.*A + wForce * np.dot(Qfqp.T, Qfqp) + wTorque * np.dot(QtauNormqp.T, QtauNormqp))
            # pqp = cvxMatrix(b + wForce * np.dot(pfqp.T, Qfqp) + wTorque * np.dot(ptauNormqp.T, QtauNormqp))

            Qqp = cvxMatrix(2.*A + wForce * np.dot(Qfqp.T, Qfqp) )
            pqp = cvxMatrix(b + wForce * np.dot(pfqp.T, Qfqp))

            equalConstForce = False
            G = np.vstack((-A[totalDOF:], -np.eye(A.shape[0])[totalDOF:]))
            hnp = np.hstack((b[totalDOF:].T, np.zeros(A.shape[0])[totalDOF:]))

            if False and not equalConstForce:
                constMu = .1
                constFric = totalForce[1]*constMu
                totalForceMat = np.concatenate((N, D, np.zeros_like(N)), axis=1)
                G = np.concatenate((G, totalForceMat[:3], totalForceMat[:3]), axis=0)
                G[-6] *= -1.
                G[-5] *= -1.
                G[-4] *= -1.
                hnp = np.hstack((hnp, np.zeros(6)))
                hnp[-6] = -totalForce[0] - constFric
                hnp[-5] = -totalForce[1] * .9
                hnp[-4] = -totalForce[2] - constFric
                hnp[-3] = totalForce[0] + constFric
                hnp[-2] = totalForce[1] * 1.1
                hnp[-1] = totalForce[2] + constFric


            # G = np.vstack((G, np.hstack((np.ones((2, N.shape[1])), np.zeros((2, D.shape[1]+N.shape[1]))))))
            # G[-2] *= -1.
            # hnp = np.hstack((hnp, np.zeros(2)))
            # hnp[-2] = -totalForce[1] * .9
            # hnp[-1] = totalForce[1] * 1.1

            # root torque 0 condition as inequality constraint
            # Atauqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            # btauqp = np.dot(pinvM1, (np.asarray(c)-np.asarray(tau0))) - np.array(tau0)

            # G = np.concatenate((G, -Atauqp, Atauqp), axis=0)
            # hnp = np.hstack((hnp, np.hstack((-btauqp, btauqp))))
            # hnp[-2*pinvM1.shape[0]:] += 1. * np.ones(2*pinvM1.shape[0])

            Gqp = cvxMatrix(G)
            hqp = cvxMatrix(hnp)

            # check correctness of equality constraint
            # tau = np.dot(pinvM1, -c + tau0 + np.dot(JTN, normalForce) + np.dot(JTD, tangenForce))
            # tau = pinvM1*JTN*theta + pinvM1*JTD*phi + pinvM1*tau0 - pinvM1*b + tau0
            # Atauqp = np.hstack((np.dot(pinvM1[:6], np.hstack((JTN, JTD))), np.zeros_like(N[:6])))
            # btauqp = np.dot(pinvM1[:6], (np.asarray(c)-np.asarray(tau0))) - np.asarray(tau0[:6])
            # Atauqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            # btauqp = np.dot(pinvM1, (np.asarray(c)-np.asarray(tau0))) - np.asarray(tau0)
            Atauqp = np.hstack((np.eye(6), np.zeros((6, A.shape[1]-6))))
            btauqp = np.zeros((6))

            AextTorqp = np.concatenate((rcN, rcD, np.zeros_like(N[:3])), axis=1)
            bextTorqp = totalForce[3:]


            # Atauqp = np.vstack((Atauqp, AextTorqp))
            # btauqp = np.hstack((btauqp, bextTorqp))

            if equalConstForce:
                Atauqp = cvxMatrix(np.vstack((np.concatenate((N[1:2], D[1:2], np.zeros(N[1:2].shape))), Atauqp)))
                btauqp = cvxMatrix(np.hstack((np.array(totalForce[1]), btauqp)))
                # Atauqp = cvxMatrix(np.vstack((np.concatenate((N[1:2], D[1:2], np.zeros(N[1:2].shape), AextTorqp)), Atauqp)))
                # btauqp = cvxMatrix(np.concatenate((np.array(totalForce[1]), bextTorqp, btauqp), axis=1))

            Aqp = cvxMatrix(Atauqp)
            bqp = cvxMatrix(btauqp)

            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiters'] = 100
            cvxSolvers.options['refinement'] = 1
            xqp = np.array(cvxSolvers.qp(Qqp, pqp, Gqp, hqp, Aqp, bqp)['x']).flatten()
            # xqp = np.asarray(cvxSolvers.qp(Qqp, pqp, Gqp, hqp)['x']).flatten()
            # print "x: ", x
            # zqp = np.dot(A, xqp).T + b
            # print "QP z: ", np.dot(xqp, zqp)
            # if np.dot(xqp, zqp) < np.dot(x, z):
            x = xqp.copy()
        except Exception, e:
            print 'LCPControl!!', e
            pass

    tau = x[:totalDOF]
    normalForce = x[totalDOF:totalDOF+contactNum]
    tangenForce = x[totalDOF+contactNum:totalDOF+contactNum + numFrictionBases*contactNum]
    minTangenVel = x[totalDOF+contactNum + numFrictionBases*contactNum:]

    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]

        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
        # print force
        forces.append(force)

    # repairForces(forces, contactPositions)
    # print forces
    return bodyIDs, contactPositions, contactPositionsLocal, forces, tau

def calcIterLCPControl(iterNum, motion, world, model, bodyIDsToCheck, mu, totalForce, wForce, wTorque=0., tau0=None, numFrictionBases=8):
    # tau0 = None
    # model = VpControlModel
    # numFrictionBases = 8
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, contactVelocities, JTN, JTD, E, N, D \
        = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)
    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None, None

    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF, totalDOF))
    invMc = np.zeros(totalDOF)
    model.getInverseEquationOfMotion(invM, invMc)

    # Jc = np.zeros(())
    # N = np.zeros(())
    # D = np.zeros(())
    # E = np.zeros(())

    M = npl.inv(invM)
    c = np.dot(M, invMc)

    Mtmp = np.dot(M, M.T)+np.eye(M.shape[0])
    # Mtmp[:6, :6] -= np.eye(6)

    pinvM = npl.inv(Mtmp)
    pinvM0 = np.dot(M.T, pinvM)
    pinvM1 = -pinvM

    h = world.GetTimeStep()
    invh = 1./h
    print "TimeStep: :", h
    mus = mu * np.eye(contactNum)
    temp_NM = JTN.T.dot(pinvM0)
    temp_DM = JTD.T.dot(pinvM0)

    A11 = h*temp_NM.dot(JTN)
    A12 = h*temp_NM.dot(JTD)
    A21 = h*temp_DM.dot(JTN)
    A22 = h*temp_DM.dot(JTD)

    factor = 1.
    # A, b = getLCPMatrix(world, model, pinvM0, c, mu, tau0, contactNum, contactPositions, JTN, JTD, E, factor)

    A1 = np.concatenate((A11, A12,  np.zeros((A11.shape[0], E.shape[1]))),   axis=1)
    A2 = np.concatenate((A21, A22,  E),                                      axis=1)
    A3 = np.concatenate((mus, -E.T, np.zeros((mus.shape[0], E.shape[1]))),  axis=1)
    A = np.concatenate((A1,
                        A2,
                        A3), axis=0) * factor
    A = 0.01 * np.eye(A.shape[0])*factor
    # print npl.eigvals(A+A.T)
    # npl.eigvals(A+A.T)

    # bx= h * (M*qdot_0 + tau - c)
    # b =[N.T * Jc * invM * kx]
    #   [D.T * Jc * invM * kx]
    #   [0]

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)
    qdot_0 = np.asarray(qdot_0)
    if tau0 is None:
        tau0 = np.zeros(np.shape(qdot_0))

    # non-penentration condition
    # b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)

    # improved non-penentration condition : add position condition
    penDepth = 0.003
    bPenDepth = np.zeros(A1.shape[0])
    for i in range(contactNum):
        if abs(contactPositions[i][1]) > penDepth:
            bPenDepth[i] = contactPositions[i][1] + penDepth

    b1 = JTN.T.dot(qdot_0) + h*temp_NM.dot(tau0 - c) + 0.5*invh*bPenDepth
    b2 = JTD.T.dot(qdot_0) + h*temp_DM.dot(tau0 - c)
    b3 = np.zeros(mus.shape[0])
    b = np.hstack((np.hstack((b1, b2)), b3)) * factor

    # lo = np.zeros(A.shape[0])
    lo = 0.*np.ones(A.shape[0])
    hi = 1000000. * np.ones(A.shape[0])
    x = 100.*np.ones(A.shape[0])

    # normalizeMatrix(A, b)

    # for torque equality constraints computation
    modelCom = model.getCOM()
    rcN = np.zeros((3, N.shape[1]))
    rcD = np.zeros((3, D.shape[1]))
    for cIdx in range(len(contactPositions)):
        r = contactPositions[cIdx] - modelCom
        rcN[:3, cIdx] = np.cross(r, N[:3, cIdx])
        for fbIdx in range(numFrictionBases):
            dIdx = numFrictionBases * cIdx + fbIdx
            rcD[:3, dIdx] = np.cross(r, D[:3, dIdx])

    if True:
        # if True:
        try:
            # Qqp = cvxMatrix(A+A.T)
            # pqp = cvxMatrix(b)

            # Qtauqp = np.hstack((np.dot(pinvM1[:6], np.hstack((JTN, JTD))), np.zeros_like(N[:6])))
            # ptauqp = np.dot(pinvM1[:6], (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0[:6])

            # Qqp = cvxMatrix(2.*A + wTorque * np.dot(Qtauqp.T, Qtauqp))
            # pqp = cvxMatrix(b + wTorque * np.dot(ptauqp.T, Qtauqp))

            Qfqp = np.concatenate((N[:3], D[:3], np.zeros_like(N[:3])), axis=1)
            pfqp = -totalForce[:3]


            # TODO:
            # add tau norm term ||tau||^2
            # and momentum derivative term
            QtauNormqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            ptauNormqp = np.dot(pinvM1, (-np.asarray(c)+np.asarray(tau0))) + np.asarray(tau0)

            Qqp = cvxMatrix(2.*A + wForce * np.dot(Qfqp.T, Qfqp) + wTorque * np.dot(QtauNormqp.T, QtauNormqp))
            pqp = cvxMatrix(b + wForce * np.dot(pfqp.T, Qfqp) + wTorque * np.dot(ptauNormqp.T, QtauNormqp))

            # Qqp = cvxMatrix(2.*A + wTorque * np.dot(Qfqp.T, Qfqp) )
            # pqp = cvxMatrix(b + wTorque * np.dot(pfqp.T, Qfqp))


            equalConstForce = False
            G = np.vstack((-A, -np.eye(A.shape[0])))
            hnp = np.hstack((b.T, np.zeros(A.shape[0])))

            # G = np.vstack((G, np.hstack((np.ones((2, N.shape[1])), np.zeros((2, D.shape[1]+N.shape[1]))))))
            # G[-2] *= -1.
            # hnp = np.hstack((hnp, np.zeros(2)))
            # hnp[-2] = -totalForce[1] * .9
            # hnp[-1] = totalForce[1] * 1.1

            # root torque 0 condition as inequality constraint
            Atauqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            btauqp = np.dot(pinvM1, (np.asarray(c)-np.asarray(tau0))) - np.array(tau0)

            # G = np.concatenate((G, -Atauqp, Atauqp), axis=0)
            # hnp = np.hstack((hnp, np.hstack((-btauqp, btauqp))))
            # hnp[-2*pinvM1.shape[0]:] += 1. * np.ones(2*pinvM1.shape[0])

            Gqp = cvxMatrix(G)
            hqp = cvxMatrix(hnp)

            # check correctness of equality constraint
            # tau = np.dot(pinvM1, -c + tau0 + np.dot(JTN, normalForce) + np.dot(JTD, tangenForce))
            # tau = pinvM1*JTN*theta + pinvM1*JTD*phi + pinvM1*tau0 - pinvM1*b + tau0
            Atauqp = np.hstack((np.dot(pinvM1[:6], np.hstack((JTN, JTD))), np.zeros_like(N[:6])))
            btauqp = np.dot(pinvM1[:6], (np.asarray(c)-np.asarray(tau0))) - np.asarray(tau0[:6])
            # Atauqp = np.hstack((np.dot(pinvM1, np.hstack((JTN, JTD))), np.zeros((pinvM1.shape[0], N.shape[1]))))
            # btauqp = np.dot(pinvM1, (np.asarray(c)-np.asarray(tau0))) - np.asarray(tau0)

            AextTorqp = np.concatenate((rcN, rcD, np.zeros_like(N[:3])), axis=1)
            bextTorqp = totalForce[3:]


            # Atauqp = np.vstack((Atauqp, AextTorqp))
            # btauqp = np.hstack((btauqp, bextTorqp))

            if equalConstForce:
                Atauqp = cvxMatrix(np.vstack((np.concatenate((N[1:2], D[1:2], np.zeros(N[1:2].shape))), Atauqp)))
                btauqp = cvxMatrix(np.hstack((np.array(totalForce[1]), btauqp)))
                # Atauqp = cvxMatrix(np.vstack((np.concatenate((N[1:2], D[1:2], np.zeros(N[1:2].shape), AextTorqp)), Atauqp)))
                # btauqp = cvxMatrix(np.concatenate((np.array(totalForce[1]), bextTorqp, btauqp), axis=1))

            Aqp = cvxMatrix(Atauqp)
            bqp = cvxMatrix(btauqp)

            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiters'] = 100
            cvxSolvers.options['refinement'] = 1
            xqp = np.array(cvxSolvers.qp(Qqp, pqp, Gqp, hqp, Aqp, bqp)['x']).flatten()
            x = xqp.copy()

        except Exception, e:
            print 'IterLCPControl!!', e
            pass

    normalForce = x[:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    minTangenVel = x[contactNum + numFrictionBases*contactNum:]

    tau = np.dot(pinvM1, -c + tau0 + np.dot(JTN, normalForce) + np.dot(JTD, tangenForce)) + np.array(tau0)

    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]

        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
        # print force
        forces.append(force)

    # repairForces(forces, contactPositions)
    # print forces
    return bodyIDs, contactPositions, contactPositionsLocal, forces, tau
    pass

