import Optimization.csLCPLemkeSolver as lcp

import ArticulatedBody.ysJacobian as yjc

import numpy as np
import math

M_2PI = math.pi * 2

model = VpControlModel

totalDOF = model.getTotalDOF()

invM = np.zeros((totalActuator,totalDOF))
invMc = np.zeros(totalDOF)

model.getInverseEquationOfMotion(invM, invMc)


Jc = np.zeros(())
N = np.zeros(())
D = np.zeros(())
E = np.zeros(())


numFrictionBases = 8
contactNum, Jc, N, D, E = makeFrictionCone(bodyIDsToCheck, numFrictionBases)

mus = [mu ] * contactNum
temp_NM = N.T.dot(invM)
temp_DM = D.T.dot(invM)


#A =[ A11,  A12, 0]
#   [ A21,  A22, E]
#   [ mus, -E.T, 0]


A11 = temp_NM.dot(N.T)
A12 = temp_NM.dot(D.T)
A21 = temp_DM.dot(N.T)
A22 = temp_DM.dot(D.T)

A1 = np.hstack((np.hstack((A11, A12)), np.zeros((A11.shape[0], E.shape[1])) ))
A2 = np.hstack((np.hstack((A21, A22)), E ))
A3 = np.hstack((np.hstack((mus,-E.T)), np.zeros((mus.shape[0], E.shape[1])) ))
A = np.vstack((np.vstack((A1, A2)), A3))

#kx= h * (M*qdot_0 + tau - c)
#k =[N.T * Jc * invM * kx]
#   [D.T * Jc * invM * kx]
#   [0]


b1 = N.T.dot(qdot_0 - invMc) + temp_NM.dot(tau)
b2 = D.T.dot(qdot_0 - invMc) + temp_DM.dot(tau)
b3 = np.zeros((mus.shape[0], 1))
b  = np.vstack((np.vstack((b1, b2)), b3))

lo = np.zeros(A.shape[0])
hi = 10000. * np.ones(A.shape[0])

lcpSolver = lcp.LemkeSolver()
lcpSolver.solve(A.shape[0], A, b, x, lo, hi)

normalForce = x[0:contactNum]
tangenForce = x[contactNum:contactNum + numFrictionBases*contactNum]

forces = []
for cIdx in range(contactNum):
    force = np.zeros(6, 1)
    force[1] = normalForce[idx]
    for fcIdx in range(numFrictionBases):
        d = np.array( math.cos(M_2PI*i/numFrictionBases), 0., math.sin(M_2PI*i/numFrictionBases), 0., 0., 0.)
        force += tangenForce[cIdx*numFrictionBases + fcIdx] * d
	forces.append(force) 


def makeFrictionCone(bodyIDsToCheck, numFrictionBases):
	contactVpBodyIds, contactPositions, contactPositionsLocal = world.getContactPoints(bodyIDsToCheck)

	for vpidx in contactVpBodyIds:
		yjc.computeJacobian2(Jic, DOFs, jointPositions, jointAxeses, [headPos], comUpperJointMasks)

		bodyidx = model.id2index(vpidx)
		n = np.array((0.,1.,0.,0.,0.,0.))
		N = np.hstack( (N, np.dot(Jic.T, n)) )
		for i in range(numFrictionBases):
			d = np.array( math.cos(M_2PI*i/numFrictionBases), 0., math.sin(M_2PI*i/numFrictionBases), 0., 0., 0.)
			D = np.hstack( (D, np.dot(Jic.T, d)) )

	numContact = len(contactVpBodyIds)
	E = np.eye(numContact)
		
	return numContact, Jc, N, D, E	


