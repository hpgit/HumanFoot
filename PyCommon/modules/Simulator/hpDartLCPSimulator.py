import time
import math
import numpy as np
import numpy.linalg as npl
from cvxopt import solvers as cvxSolvers
from cvxopt import matrix as cvxMatrix

from PyCommon.modules.Simulator import csDartModel as cdm
import PyCommon.modules.pydart2 as pydart

from PyCommon.modules.Motion import ysMotion as ym


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

def makeFrictionCone(skeleton, world, model, bodyIDsToCheck, numFrictionBases):
    """

    :type skeleton: ym.JointSkeleton
    :type world: pydart.World
    :type model: cdm.DartModel
    :type bodyIDsToCheck: list[int]
    :type numFrictionBases: int
    :rtype:
    """
    cBodyIds, cPositions, cPositionsLocal, cVelocities = model.getContactPoints(bodyIDsToCheck)
    N = None
    D = None
    E = None

    cNum = len(cBodyIds)
    if cNum == 0:
        return len(cBodyIds), cBodyIds, cPositions, cPositionsLocal, cVelocities, None, None, None, None, None
    d = [None]*numFrictionBases # type: np.ndarray

    DOFs = model.getDOFs()

    for idx in range(len(cBodyIds)):
        body = model.getBody(cBodyIds[idx])
        jacobian = body.world_jacobian(cPositionsLocal[idx])
        n = np.array([[0., 1., 0., 0., 0., 0.]]).T
        JTn = np.dot(jacobian.T, n)
        if N is None:
            JTN = JTn.copy()
            N = n.copy()
        else:
            JTN = np.hstack((JTN, JTn))
            N = np.hstack((N, n))
        # cVel = cVelocities[idx]
        # offsetAngle = math.atan2(cVel[2], cVel[0])
        offsetAngle = 0.
        for i in range(numFrictionBases):
            d[i] = np.array([[math.cos((2.*math.pi*i)/numFrictionBases), 0., math.sin((2.*math.pi*i)/numFrictionBases), 0., 0., 0.]]).T
        for i in range(numFrictionBases):
            JTd = np.dot(jacobian.T, d[i])
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

    return len(cBodyIds), cBodyIds, cPositions, cPositionsLocal, cVelocities, JTN, JTD, E, N, D

def getLCPMatrix(world, model, invM, invMc, mu, tau, contactNum, contactPositions, JTN, JTD, E, factor=1.):
    """

    :type world: pydart.World
    :type model: cdm.DartModel
    :type invM: np.ndarray
    :type invMc: np.ndarray
    :type mu: float
    :type tau: np.ndarray
    :type contactNum: int
    :type contactPositions: list[np.ndarray]
    :type JTN: np.ndarray
    :type JTD: np.ndarray
    :type E: np.ndarray
    :type factor: float
    :return:
    """
    totalDOF = model.getTotalDOF()

    h = model.GetTimeStep()
    invh = 1./h
    mus = mu * np.eye(contactNum) # type: np.ndarray
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
            h * np.concatenate((mus, -E.T, np.zeros((mus.shape[0], E.shape[1]))),  axis=1),
        ), axis=0
    )
    # A = A + 0.1*np.eye(A.shape[0])

    qdot_0 = np.asarray(model.skeleton.dq)
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

def calcLCPForces(motion, world, model, bodyIDsToCheck, mu, tau=None, numFrictionBases=8, solver='qp'):
    """

    :type motion: ym.JointMotion
    :type world: pydart.World
    :type model: cdm.DartModel
    :type bodyIDsToCheck: list[int]
    :type mu: float
    :type tau: np.ndarray
    :type numFrictionBases: int
    :type solver: str
    :return:
    """
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

    invM = model.skeleton.inv_mass_matrix()
    invMc = np.dot(invM, model.skeleton.coriolis_and_gravity_forces())

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
            solution = cvxSolvers.qp(Aqp, bqp, Gqp, hqp)
            xqp = np.array(solution['x']).flatten()
            x = xqp.copy()
        except Exception, e:
            print e
            pass


    normalForce = x[:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    # tangenForce = np.zeros_like(x[contactNum:contactNum + numFrictionBases*contactNum])
    minTangenVel = x[contactNum + numFrictionBases*contactNum:]
    # print minTangenVel

    tangenForceDual = (np.dot(A,x)+b)[contactNum:contactNum+numFrictionBases*contactNum]
    # print "hehe:", (np.dot(A,x)+b)[contactNum:contactNum+numFrictionBases*contactNum]
    # print "hihi:", tangenForce
    # print np.dot(tangenForce, tangenForceDual)

    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]
        # contactTangenForce = tangenForce[8*cIdx:8*(cIdx+1)]
        contactTangenForceDual = tangenForceDual[8*cIdx:8*(cIdx+1)]
        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d

        # minBasisIdx = np.argmin(contactTangenForceDual)
        # d = np.array((math.cos((2.*math.pi*minBasisIdx)/numFrictionBases), 0., math.sin((2.*math.pi*minBasisIdx)/numFrictionBases)))
        # force += tangenForce[cIdx*numFrictionBases + minBasisIdx] * d

        forces.append(force)

    # repairForces(forces, contactPositions)
    # print forces
    timeStamp, timeIndex, prevTime = setTimeStamp(timeStamp, timeIndex, prevTime)



    # debug
    __HP__DEBUG__= False
    if __HP__DEBUG__ and len(bodyIDs) ==4:
        vpidx = 3
        DOFs = model.getDOFs()
        Jic = yjc.makeEmptyJacobian(DOFs, 1)

        qdot_0 = ype.makeFlatList(totalDOF)
        ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)

        jointAxeses = model.getBodyRootDOFAxeses()
        bodyidx = model.id2index(bodyIDs[vpidx])
        contactJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, bodyidx)]

        jointPositions = model.getJointPositionsGlobal()
        jointPositions[0] = model.getBodyPositionGlobal(0)
        yjc.computeLocalRootJacobian(Jic, DOFs, jointPositions, jointAxeses, [contactPositions[vpidx]], contactJointMasks)

        h = world.GetTimeStep()
        vv = np.dot(Jic, qdot_0) - h * np.dot(Jic, invMc) + h * np.dot(Jic, np.dot(invM, tau))
        for vpidxx in range(len(bodyIDs)):
            bodyidx = model.id2index(bodyIDs[vpidxx])
            contactJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, bodyidx)]
            yjc.computeLocalRootJacobian(Jic, DOFs, jointPositions, jointAxeses, [contactPositions[vpidxx]], contactJointMasks)
            vv += h * np.dot(Jic, np.dot(invM, np.dot(Jic[:3].T, forces[vpidxx])))

        print "vv:", vv[:3]



    return bodyIDs, contactPositions, contactPositionsLocal, forces, timeStamp



def calcLCPbasicControl(motion, world, model, bodyIDsToCheck, mu, totalForce, weights, ddq0=None, numFrictionBases=8):
    """

    :type motion: ym.JointMotion
    :type world: pydart.World
    :type model: cdm.DartModel
    :type bodyIDsToCheck: list[int]
    :type mu: float
    :type ddq0: np.ndarray
    :type numFrictionBases: int
    :return:
    """
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

