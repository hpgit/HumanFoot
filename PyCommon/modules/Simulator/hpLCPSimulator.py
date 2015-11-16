import Optimization.csLCPLemkeSolver as lcp
import Optimization.csLCPDantzigSolver as lcpD
from cvxopt import matrix as cvxMatrix
from cvxopt import solvers as cvxSolvers

import ArticulatedBody.ysJacobian as yjc

import Util.ysPythonEx as ype

import numpy as np
import numpy.linalg as npl
import math

import pdb



def makeFrictionCone(skeleton, world, model, bodyIDsToCheck, numFrictionBases):
    contactVpBodyIds, contactPositions, contactPositionsLocal = world.getContactPoints(bodyIDsToCheck)
    N = None
    D = None
    E = None

    contactNum = len(contactVpBodyIds)
    if contactNum == 0:
        return len(contactVpBodyIds), contactVpBodyIds, contactPositions, contactPositionsLocal, None, None, None
    d = [None]*numFrictionBases
    for i in range(numFrictionBases):
        d[i] = np.array( [[math.cos(2.*math.pi*i/numFrictionBases), 0., math.sin(2.*math.pi*i/numFrictionBases), 0., 0., 0.]]).T

    DOFs = model.getDOFs()
    Jic = yjc.makeEmptyJacobian(DOFs, 1)
    jointPositions = model.getJointPositionsGlobal()
    #jointAxeses = model.getDOFAxeses()
    jointAxeses = model.getDOFAxesesLocal()

    for vpidx in range(len(contactVpBodyIds)):
        bodyidx = model.id2index(contactVpBodyIds[vpidx])
        contactJointMasks = [yjc.getLinkJointMask(skeleton, bodyidx)]
        yjc.computeJacobian2(Jic, DOFs, jointPositions, jointAxeses, [contactPositions[vpidx]], contactJointMasks)
        #pdb.set_trace()
        n = np.array([[0.,1.,0.,0.,0.,0.]]).T
        JTn = Jic.T.dot(n)
        if N is None:
            N = JTn
        else:
            N = np.hstack( ( N, JTn))
        for i in range(numFrictionBases):
            JTd = Jic.T.dot(d[i])
            if D is None:
                D = JTd
            else:
                D = np.hstack( (D, JTd) )
        #pdb.set_trace()
    
    E = np.zeros((contactNum*numFrictionBases,contactNum))
    for cIdx in range(contactNum):
        for fcIdx in range(numFrictionBases):
            E[cIdx*numFrictionBases + fcIdx][cIdx] = 1.

    return len(contactVpBodyIds), contactVpBodyIds, contactPositions, contactPositionsLocal, N, D, E  

def repairForces(forces, contactPositions):
    for idx in range(0, len(forces)):
        force = forces[idx]
        #if force[1] < 0.:
        #    force[0] = 0.
        #    force[2] = 0.
        #    force[1] = 0.
            #force[1] = -contactPositions[idx][1]*2000.
        #elif force[1] > 10000.:
        #    ratio = 10000./force[1]
        #    force *= ratio
        #if force[1]*force[1] < force[2]*force[2] + force[0]*force[0] :
        #    norm = math.sqrt(force[0] * force[0] + force[2]*force[2])
        #    force[0] /= norm
        #    force[2] /= norm
        pass


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
    #print "hpLCPSimulator:contactNum : ", contactNum
    if contactNum == 0:
        return bodyIDs, contactPositions, contactPositionsLocal, None

    h = world.getTimeStep()
    mus = mu * np.eye(contactNum)
    temp_NM = N.T.dot(invM)
    temp_DM = D.T.dot(invM)

    #pdb.set_trace()


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

    factor = 10000.

    A1 = np.hstack((np.hstack((A11, A12)), np.zeros((A11.shape[0], E.shape[1])) ))
    A2 = np.hstack((np.hstack((A21, A22)), E ))
    A3 = np.hstack((np.hstack((mus,-E.T)), np.zeros((mus.shape[0], E.shape[1])) ))
    A = np.vstack((np.vstack((A1, A2)), A3)) * factor
    #pdb.set_trace()
    #A = A + 0.0001*np.eye(A.shape[0])

    #bx= h * (M*qdot_0 + tau - c)
    #b =[N.T * Jc * invM * kx]
    #   [D.T * Jc * invM * kx]
    #   [0]

    qdot_0 = ype.makeFlatList(totalDOF)
    ype.flatten(model.getDOFVelocitiesLocal(), qdot_0)
    qdot_0 = np.array(qdot_0)
    if tau is None:
        tau = np.zeros(np.shape(qdot_0))

    b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)
    b2 = D.T.dot(qdot_0 - h*invMc) + h*temp_DM.dot(tau)
    b3 = np.zeros(mus.shape[0])

    b  = np.hstack((np.hstack((b1, b2)), b3)) * factor

    #print "b: ", b

    #print "np.shape(A): ", np.shape(A)
    #print "np.shape(b): ", np.shape(b)

    #print "A: ", A
    #print "b: ", b
    
    #lo = np.zeros(A.shape[0])
    lo = 0.*np.ones(A.shape[0])
    hi = 1000000. * np.ones(A.shape[0])
    x = 100.*np.ones(A.shape[0])
    #try:
    #    Aqp = cvxMatrix(2*A)
    #    bqp = cvxMatrix(b)
    #    Gqp = cvxMatrix(np.vstack((-A,-np.eye(A.shape[0]))))
    #    hqp = cvxMatrix(np.hstack((b.T,np.zeros(A.shape[0]))))
    #    cvxSolvers.options['show_progress'] = False
    #    cvxSolvers.options['maxiter'] = 100000
    #    x = np.array(cvxSolvers.qp(Aqp, bqp, Gqp, hqp)['x']).flatten()
    #    print "x: ", x
    #    z = np.dot(A,x).T +b
    #    #print z
    #    print "QP!"
    #except Exception, e:
    #    print e
    lcpSolver = lcp.LemkeSolver()
    #lcpSolver = lcpD.DantzigSolver()
    lcpSolver.solve(A.shape[0], A, b, x, lo, hi)
    z = np.dot(A,x) + b
    
    if abs(np.dot(x,z)) > 100.:
        try:
            print "prev z: ", np.dot(x, z)
            Aqp = cvxMatrix(2*A)
            bqp = cvxMatrix(b)
            Gqp = cvxMatrix(np.vstack((-A,-np.eye(A.shape[0]))))
            hqp = cvxMatrix(np.hstack((b.T,np.zeros(A.shape[0]))))
            cvxSolvers.options['show_progress'] = False
            cvxSolvers.options['maxiter'] = 100000
            xqp = np.array(cvxSolvers.qp(Aqp, bqp, Gqp, hqp)['x']).flatten()
            #print "x: ", x
            zqp = np.dot(A,x).T +b
            print "QP z: ", np.dot(xqp, zqp)
            if np.dot(xqp, zqp) < np.dot(x, z):
                x = xqp.copy()
        except Exception, e:
            print e



    normalForce = x[:contactNum]
    tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]
    minTangenVel = x[contactNum + numFrictionBases*contactNum:]
    #print "x: ", x
    #print "normalForce: ", normalForce
    #print "tangenForce: ", tangenForce
    #print "minTangenVel: ", minTangenVel
    
    #print "z: ", z
    #print "np.dot(x,z): ", np.dot(x,z)
    #print sum(x >= 0.)
    #print sum(z >= 0.)


    forces = []
    for cIdx in range(contactNum):
        force = np.zeros(3)
        force[1] = normalForce[cIdx]
        
        for fcIdx in range(numFrictionBases):
            d = np.array((math.cos(2.*math.pi*fcIdx/numFrictionBases), 0., math.sin(2.*math.pi*fcIdx/numFrictionBases)))
            force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
        #print force
    	forces.append(force) 
    repairForces(forces, contactPositions)
    #print forces
    return bodyIDs, contactPositions, contactPositionsLocal, forces





