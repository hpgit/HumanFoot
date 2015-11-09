import Optimization.csLCPLemkeSolver as lcp

import ArticulatedBody.ysJacobian as yjc

import Util.ysPythonEx as ype

import numpy as np
import math


def makeFrictionCone(skeleton, world, model, bodyIDsToCheck, numFrictionBases):
    contactVpBodyIds, contactPositions, contactPositionsLocal = world.getContactPoints(bodyIDsToCheck)
    N = None
    D = None

    DOFs = model.getDOFs()
    Jic = yjc.makeEmptyJacobian(DOFs, 1)
    jointPositions = model.getJointPositionsGlobal()
    jointAxeses = model.getDOFAxeses()

    for vpidx in range(len(contactVpBodyIds)):
        bodyidx = model.id2index(contactVpBodyIds[vpidx])
        contactJointMasks = [yjc.getLinkJointMask(skeleton, bodyidx)]
        yjc.computeJacobian2(Jic, DOFs, jointPositions, jointAxeses, [contactPositions[vpidx]], contactJointMasks)

        n = np.array((0.,1.,0.,0.,0.,0.))
        if N is None:
            N = np.dot(Jic.T, n.T)
        else:
            N = np.vstack( (N, np.dot(Jic.T, n.T)) )
        for i in range(numFrictionBases):
            d = np.array( (math.cos(2.*math.pi*i/numFrictionBases), 0., math.sin(2.*math.pi*i/numFrictionBases), 0., 0., 0.))
            if D is None:
                D = np.dot(Jic.T, d)
            else:
                D = np.vstack( (D, np.dot(Jic.T, d)) )
    contactNum = len(contactVpBodyIds)
    #TODO: E is nd*n matrix
    E = np.eye(contactNum)
    E = np.zeros(np.shape(N)).T

    if N is not None:
        N = N.T
        D = D.T

    return len(contactVpBodyIds), contactVpBodyIds, contactPositions, contactPositionsLocal, N, D, E  

def calcLCPForces(motion, world, model, bodyIDsToCheck, mu, numFrictionBases, tau):
    #model = VpControlModel
    totalDOF = model.getTotalDOF()

    invM = np.zeros((totalDOF,totalDOF))
    invMc = np.zeros(totalDOF)
    model.getInverseEquationOfMotion(invM, invMc)

    #Jc = np.zeros(())
    #N = np.zeros(())
    #D = np.zeros(())
    #E = np.zeros(())

    #numFrictionBases = 8
    contactNum, bodyIDs, contactPositions, contactPositionsLocal, N, D, E = makeFrictionCone(motion[0].skeleton, world, model, bodyIDsToCheck, numFrictionBases)
    print "hpLCPSimulator:contactNum : ", contactNum
    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, []

    mus = mu * np.eye(contactNum)
    temp_NM = N.T.dot(invM)
    temp_DM = D.T.dot(invM)


    #A =[ A11,  A12, 0]
    #   [ A21,  A22, E]
    #   [ mus, -E.T, 0]

    A11 = temp_NM.dot(N)
    A12 = temp_NM.dot(D)
    A21 = temp_DM.dot(N)
    A22 = temp_DM.dot(D)

    E = np.zeros(np.shape(A21))
    for cIdx in range(contactNum):
        for fcIdx in range(numFrictionBases):
            E[cIdx*numFrictionBases + fcIdx][cIdx] = 1.

    A1 = np.hstack((np.hstack((A11, A12)), np.zeros((A11.shape[0], E.shape[1])) ))
    A2 = np.hstack((np.hstack((A21, A22)), E ))
    A3 = np.hstack((np.hstack((mus,-E.T)), np.zeros((mus.shape[0], E.shape[1])) ))
    A = np.vstack((np.vstack((A1, A2)), A3))

    #bx= h * (M*qdot_0 + tau - c)
    #b =[N.T * Jc * invM * kx]
    #   [D.T * Jc * invM * kx]
    #   [0]

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getDOFVelocitiesLocal(), qdot_0)
    h = world.getTimeStep()

    tau = np.zeros(np.shape(qdot_0))

    b1 = h*(N.T.dot(qdot_0 - invMc) + temp_NM.dot(tau))
    b2 = h*(D.T.dot(qdot_0 - invMc) + temp_DM.dot(tau))
    b3 = np.zeros(mus.shape[0])

    b  = np.hstack((np.hstack((b1, b2)), b3))

    #print "np.shape(A): ", np.shape(A)
    #print "np.shape(b): ", np.shape(b)

    #print "A: ", A
    #print "b: ", b
    lo = np.zeros(A.shape[0])
    hi = 10000. * np.ones(A.shape[0])

    x = np.zeros(A.shape[0])
    lcpSolver = lcp.LemkeSolver()
    lcpSolver.solve(A.shape[0], A, b, x, lo, hi)

    normalForce = x[0:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]

    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]
        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
    	forces.append(force) 

    print forces
    return bodyIDs, contactPositions, contactPositionsLocal, forces





