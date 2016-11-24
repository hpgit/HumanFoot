import time

from PyCommon.modules.Simulator import csDartModel as cdm
import PyCommon.modules.pydart2 as pydart

from PyCommon.modules.Motion import ysMotion as ym


def makeFrictionCone(skeleton, world, model, bodyIDsToCheck, numFrictionBases):
    """

    :type skeleton: ym.JointSkeleton
    :type world:
    :type model: cdm.DartModel
    :type bodyIDsToCheck: list[int]
    :type numFrictionBases: int
    :rtype:
    """
    cVpBodyIds, cPositions, cPositionsLocal, cVelocities = world.getContactPoints(bodyIDsToCheck)

    model.getBody(1).jacobian()
    model.getBody(1).angular_jacobian()
    model.getBody(1).linear_jacobian()
    model.getBody(1).world_jacobian()
    model.getBody(1).jacobian()
    model.getBody(1).jacobian()
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
    # jointAxeses = model.getBodyRootJointAngJacobiansGlobal()

    # totalDOF = model.getTotalDOF()
    # qdot_0 = ype.makeFlatList(totalDOF)
    # # ype.flatten(model.getDOFVelocitiesLocal(), qdot_0)
    # # bodyGenVelLocal = model.getBodyGenVelLocal(0)
    # #
    # # for i in range(3):
    # #     qdot_0[i] = bodyGenVelLocal[i+3]
    # #     qdot_0[i+3] = bodyGenVelLocal[i]
    # ype.flatten(model.getBodyRootDOFVelocitiesLocal(), qdot_0)

    for vpidx in range(len(cVpBodyIds)):
        bodyidx = model.id2index(cVpBodyIds[vpidx])
        contactJointMasks = [yjc.getLinkJointMask(skeleton, bodyidx)]
        yjc.computeLocalRootJacobian(Jic, DOFs, jointPositions, jointAxeses, [cPositions[vpidx]], contactJointMasks)
        # yjc.computeControlModelJacobian(Jic, DOFs, jointPositions, jointAxeses, [cPositions[vpidx]], contactJointMasks)
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
            d[i] = np.array([[math.cos((2.*math.pi*i)/numFrictionBases), 0., math.sin((2.*math.pi*i)/numFrictionBases)
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

def calcLCPForces(motion, world, model, bodyIDsToCheck, mu, tau=None, numFrictionBases=8, solver='qp'):
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
    __HP__DEBUG__= True
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


