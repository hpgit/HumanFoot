import Optimization.csLCPLemkeSolver as lcp
import Optimization.csLCPDantzigSolver as lcpD

import ArticulatedBody.ysJacobian as yjc

import Util.ysPythonEx as ype

import numpy as np
import math


def makeFrictionCone(skeleton, world, model, bodyIDsToCheck, numFrictionBases):
    contactVpBodyIds, contactPositions, contactPositionsLocal = world.getContactPoints(bodyIDsToCheck)
    NT = None
    DT = None
    E = None

    contactNum = len(contactVpBodyIds)
    if contactNum == 0:
        return len(contactVpBodyIds), contactVpBodyIds, contactPositions, contactPositionsLocal, NT, DT, None
    d = [None]*numFrictionBases
    for i in range(numFrictionBases):
        d[i] = np.array( (math.cos(2.*math.pi*i/numFrictionBases), 0., math.sin(2.*math.pi*i/numFrictionBases), 0., 0., 0.))

    DOFs = model.getDOFs()
    Jic = yjc.makeEmptyJacobian(DOFs, 1)
    jointPositions = model.getJointPositionsGlobal()
    jointAxeses = model.getDOFAxeses()

    for vpidx in range(len(contactVpBodyIds)):
        bodyidx = model.id2index(contactVpBodyIds[vpidx])
        contactJointMasks = [yjc.getLinkJointMask(skeleton, bodyidx)]
        yjc.computeJacobian2(Jic, DOFs, jointPositions, jointAxeses, [contactPositions[vpidx]], contactJointMasks)

        n = np.array((0.,1.,0.,0.,0.,0.))
        #JTn = np.dot(Jic.T, n.T)
        JTn = Jic[1]
        if NT is None:
            NT = JTn.T
        else:
            #NT = np.vstack( (NT, np.dot(Jic.T, n.T)) )
            NT = np.vstack( ( NT, JTn.T))
        for i in range(numFrictionBases):
            #JTd = np.dot(Jic.T, d[i])
            JTd = Jic[0]*d[i][0] + Jic[2]*d[i][2]
            if DT is None:
                DT = JTd.T
            else:
                DT = np.vstack( (DT, JTd.T) )

    
    E = np.zeros((contactNum*numFrictionBases,contactNum))
    for cIdx in range(contactNum):
        for fcIdx in range(numFrictionBases):
            E[cIdx*numFrictionBases + fcIdx][cIdx] = 1.

    return len(contactVpBodyIds), contactVpBodyIds, contactPositions, contactPositionsLocal, NT.T, DT.T, E  

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
        return bodyIDs, contactPositions, contactPositionsLocal, None

    h = world.getTimeStep()
    mus = mu * np.eye(contactNum)
    temp_NM = N.T.dot(invM)
    temp_DM = D.T.dot(invM)


    #A =[ A11,  A12, 0]
    #   [ A21,  A22, E]
    #   [ mus, -E.T, 0]

    A11 = h*temp_NM.dot(N)
    A12 = h*temp_NM.dot(D)
    A21 = h*temp_DM.dot(N)
    A22 = h*temp_DM.dot(D)
    
   
    #print "A11: ", A11
    #print "A12: ", A12
    #print "A21: ", A21
    #print "A22: ", A22

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
    qdot_0 = np.array(qdot_0)

    tau = np.zeros(np.shape(qdot_0))

    print "invMc: ", invMc


    b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)
    b2 = D.T.dot(qdot_0 - h*invMc) + h*temp_DM.dot(tau)
    b3 = np.zeros(mus.shape[0])

    b  = np.hstack((np.hstack((b1, b2)), b3))

    #print "b: ", b

    #print "np.shape(A): ", np.shape(A)
    #print "np.shape(b): ", np.shape(b)

    #print "A: ", A
    #print "b: ", b
    
    #lo = np.zeros(A.shape[0])
    lo = 0. * np.ones(A.shape[0])
    hi = 1000000. * np.ones(A.shape[0])

    x = 10.*np.ones(A.shape[0])
    lcpSolver = lcp.LemkeSolver()
    #lcpSolver = lcpD.DantzigSolver()
    lcpSolver.solve(A.shape[0], A, b, x, lo, hi)

    normalForce = x[:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    minTangenVel = x[contactNum + numFrictionBases*contactNum:]
    #print "x: ", x
    #print "normalForce: ", normalForce
    #print "tangenForce: ", tangenForce
    #print "minTangenVel: ", minTangenVel
    z = np.dot(A,x) + b
    #print "z: ", z
    print "np.dot(x,z): ", np.dot(x,z)

    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]
        
        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
        print force
    	forces.append(force) 

    #print forces
    return bodyIDs, contactPositions, contactPositionsLocal, forces





