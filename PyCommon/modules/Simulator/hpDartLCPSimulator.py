import time
import math
import numpy as np
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



