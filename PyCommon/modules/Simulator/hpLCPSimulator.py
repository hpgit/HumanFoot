import Optimization.csLCPLemkeSolver as lcp
import Optimization.csLCPDantzigSolver as lcpD
from cvxopt import matrix as cvxMatrix
from cvxopt import solvers as cvxSolvers

import ArticulatedBody.ysJacobian as yjc

import Util.ysPythonEx as ype
import Math.mmMath as mm

import numpy as np
import numpy.linalg as npl
import math


def makeFrictionCone(skeleton, world, model, bodyIDsToCheck, numFrictionBases):
    cVpBodyIds, cPositions, cPositionsLocal, cVelocities = world.getContactPoints(bodyIDsToCheck)
    N = None
    D = None
    E = None

    cNum = len(cVpBodyIds)
    if cNum == 0:
        return len(cVpBodyIds), cVpBodyIds, cPositions, cPositionsLocal, None, None, None, None, None
    d = [None]*numFrictionBases

    DOFs = model.getDOFs()
    Jic = yjc.makeEmptyJacobian(DOFs, 1)
    jointPositions = model.getJointPositionsGlobal()
    jointAxeses = model.getDOFAxesesLocal()

    for vpidx in range(len(cVpBodyIds)):
        bodyidx = model.id2index(cVpBodyIds[vpidx])
        contactJointMasks = [yjc.getLinkJointMask(skeleton, bodyidx)]
        yjc.computeJacobian2(Jic, DOFs, jointPositions, jointAxeses, [cPositions[vpidx]], contactJointMasks)
        n = np.array([[0., 1., 0., 0., 0., 0.]]).T
        JTn = Jic.T.dot(n)
        if N is None:
            JTN = JTn
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
                              math.sin(offsetAngle + 2.*math.pi*i/numFrictionBases),
                              0., 0., 0.]]).T

        for i in range(numFrictionBases):
            JTd = Jic.T.dot(d[i])
            if D is None:
                JTD = JTd
                D = d[i].copy()
            else:
                JTD = np.hstack((JTD, JTd))
                D = np.hstack((D, d[i]))
    
    E = np.zeros((cNum*numFrictionBases, cNum))
    for cIdx in range(cNum):
        for fcIdx in range(numFrictionBases):
            E[cIdx*numFrictionBases + fcIdx][cIdx] = 1.

    return len(cVpBodyIds), cVpBodyIds, cPositions, cPositionsLocal, JTN, JTD, E, N, D


def repairForces(forces, contactPositions):
    for idx in range(0, len(forces)):
        force = forces[idx]
        # if force[1] < 0.:
        #    force[0] = 0.
        #    force[2] = 0.
        #    force[1] = 0.
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


def calcLCPForces(motion, world, model, bodyIDsToCheck, mu, tau=None, numFrictionBases=8):
    # model = VpControlModel
    # numFrictionBases = 8
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, JTN, JTD, E, N, D\
        = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)
    # print "hpLCPSimulator:contactNum : ", contactNum
    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None
    # print D
    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF,totalDOF))
    invMc = np.zeros(totalDOF)
    model.getInverseEquationOfMotion(invM, invMc)

    # Jc = np.zeros(())
    # N = np.zeros(())
    # D = np.zeros(())
    # E = np.zeros(())

    h = world.getTimeStep()
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

    # print "A11: ", A11
    # print "A12: ", A12
    # print "A21: ", A21
    # print "A22: ", A22

    factor = 1000.

    A1 = np.hstack((np.hstack((A11, A12)), np.zeros((A11.shape[0], E.shape[1]))))
    A2 = np.hstack((np.hstack((A21, A22)), E))
    A3 = np.hstack((np.hstack((mus, -E.T)), np.zeros((mus.shape[0], E.shape[1]))))
    A = np.vstack((np.vstack((A1, A2)), A3)) * factor
    # print npl.eigvals(A)
    # pdb.set_trace()
    # A = A + 0.0001*np.eye(A.shape[0])

    # bx= h * (M*qdot_0 + tau - c)
    # b =[N.T * Jc * invM * kx]
    #   [D.T * Jc * invM * kx]
    #   [0]

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getDOFVelocitiesLocal(), qdot_0)
    qdot_0 = np.array(qdot_0)
    if tau is None:
        tau = np.zeros(np.shape(qdot_0))

    # non-penentration condition
    # b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)
    
    # improved non-penentration condition : add position condition
    penDepth = 0.003
    bPenDepth = np.zeros(A1.shape[0])
    for i in range(contactNum):
        if abs(contactPositions[i][1]) > penDepth:
            bPenDepth[i] = contactPositions[i][1] + penDepth

    b1 = JTN.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau) + 0.5*invh*bPenDepth
    # print b1
    b2 = JTD.T.dot(qdot_0 - h*invMc) + h*temp_DM.dot(tau)
    b3 = np.zeros(mus.shape[0])

    b = np.hstack((np.hstack((b1, b2)), b3)) * factor

    # print "b: ", b

    # print "np.shape(A): ", np.shape(A)
    # print "np.shape(b): ", np.shape(b)

    # print "A: ", A
    # print "b: ", b

    # lo = np.zeros(A.shape[0])
    lo = 0.*np.ones(A.shape[0])
    hi = 1000000. * np.ones(A.shape[0])
    x = 100.*np.ones(A.shape[0])

    # normalizeMatrix(A, b)

    # print A[0]

    lcpSolver = lcp.LemkeSolver()
    # lcpSolver = lcpD.DantzigSolver()
    # lcpSolver.solve(A.shape[0], A, b, x, lo, hi)
    z = np.dot(A, x) + b
    
    # if abs(np.dot(x,z)) > 100.:
    if True:
        try:
            # print "prev z: ", np.dot(x, z)
            Aqp = cvxMatrix(2*A)
            bqp = cvxMatrix(b)
            Gqp = cvxMatrix(np.vstack((-A, -np.eye(A.shape[0]))))
            hqp = cvxMatrix(np.hstack((b.T, np.zeros(A.shape[0]))))
            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiter'] = 100
            # cvxSolvers.options['refinement'] = 10
            xqp = np.array(cvxSolvers.qp(Aqp, bqp, Gqp, hqp)['x']).flatten()
            # print "x: ", x
            zqp = np.dot(A,xqp).T +b
            # print "QP z: ", np.dot(xqp, zqp)
            # if np.dot(xqp, zqp) < np.dot(x, z):
            if True:
                x = xqp.copy()
        except Exception, e:
            # print e
            pass

    normalForce = x[:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    minTangenVel = x[contactNum + numFrictionBases*contactNum:]
    # print "x: ", x
    # print "normalForce: ", normalForce
    # print "tangenForce: ", tangenForce
    # print "minTangenVel: ", minTangenVel
    
    # print "z: ", z
    # print "np.dot(x,z): ", np.dot(x,z)
    # print sum(x >= 0.)
    # print sum(z >= 0.)

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
    return bodyIDs, contactPositions, contactPositionsLocal, forces


def calcLCPControl(motion, world, model, bodyIDsToCheck, mu, totalForce, tau0=None, numFrictionBases=8):
    # model = VpControlModel
    # numFrictionBases = 8
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, JTN, JTD, E, N, D = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)
    # print "hpLCPSimulator:contactNum : ", contactNum
    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None

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

    pinvM = npl.inv(np.dot(M, M.T)+np.eye(M.shape[0]))
    pinvM0 = np.dot(M.T, pinvM)
    pinvM1 = -pinvM

    h = world.getTimeStep()
    invh = 1./h
    mus = mu * np.eye(contactNum)
    temp_NM = JTN.T.dot(pinvM0)
    temp_DM = JTD.T.dot(pinvM0)

    A11 = h*temp_NM.dot(JTN)
    A12 = h*temp_NM.dot(JTD)
    A21 = h*temp_DM.dot(JTN)
    A22 = h*temp_DM.dot(JTD)

    factor = 1000.

    A1 = np.hstack((np.hstack((A11, A12)), np.zeros((A11.shape[0], E.shape[1]))))
    A2 = np.hstack((np.hstack((A21, A22)), E))
    A3 = np.hstack((np.hstack((mus, -E.T)), np.zeros((mus.shape[0], E.shape[1]))))
    A = np.vstack((np.vstack((A1, A2)), A3)) * factor

    # bx= h * (M*qdot_0 + tau - c)
    # b =[N.T * Jc * invM * kx]
    #   [D.T * Jc * invM * kx]
    #   [0]

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getDOFVelocitiesLocal(), qdot_0)
    qdot_0 = np.array(qdot_0)
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
    # print b1
    b2 = JTD.T.dot(qdot_0) + h*temp_DM.dot(tau0 - c)
    b3 = np.zeros(mus.shape[0])

    b = np.hstack((np.hstack((b1, b2)), b3)) * factor

    # lo = np.zeros(A.shape[0])
    lo = 0.*np.ones(A.shape[0])
    hi = 1000000. * np.ones(A.shape[0])
    x = 100.*np.ones(A.shape[0])

    # normalizeMatrix(A, b)

    if True:
        try:
            # print "prev z: ", np.dot(x, z)
            Qqp = cvxMatrix(2*A)
            pqp = cvxMatrix(b)
            Gqp = cvxMatrix(np.vstack((-A, -np.eye(A.shape[0]))))
            hqp = cvxMatrix(np.hstack((b.T, np.zeros(A.shape[0]))))
            Aqp = cvxMatrix(np.hstack((np.hstack((N[:3], D[:3])), np.zeros(N[:3].shape))))
            bqp = cvxMatrix(np.array(totalForce))
            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiter'] = 100
            # cvxSolvers.options['refinement'] = 10
            xqp = np.array(cvxSolvers.qp(Qqp, pqp, Gqp, hqp, Aqp, bqp)['x']).flatten()
            # print "x: ", x
            zqp = np.dot(A, xqp).T + b
            # print "QP z: ", np.dot(xqp, zqp)
            # if np.dot(xqp, zqp) < np.dot(x, z):
            if True:
                x = xqp.copy()
        except Exception, e:
            # print e
            pass

    normalForce = x[:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    minTangenVel = x[contactNum + numFrictionBases*contactNum:]

    tau = np.dot(pinvM1, -c + tau0 + np.dot(N, normalForce) + np.dot(D, tangenForce))

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


