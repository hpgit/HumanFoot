import numpy as np
import numpy.linalg as npl

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')
import PyCommon.modules.Math.mmMath as mm
import PyCommon.modules.Util.ysPythonEx as ype

def getTrackingWeight(DOFs, skeleton, weightMap, rootPositionWeight=0.):
    weights = [1.]*skeleton.getJointNum()
    for name, weight in weightMap.items():
        index = skeleton.getJointIndex(name)

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


# Quadratic Programming 
# x = [ddotq tau lambda]

def addQPTrackingTerms(qp, totalProblemSize, si, totalDOF, weight, jointWeights, ddth_des_flat):
    # minimize | Wt(ddth_des - ddth) |^2
    #jointWeights[0] = .5
    #jointWeights[1] = 1.1
    #jointWeights[2] = .5
    #jointWeights[3] = 1.1
    #jointWeights[4] = 1.1
    #jointWeights[5] = 1.1

    jointWeights[0] = .5
    jointWeights[1] = .1
    jointWeights[2] = .5
    jointWeights[3] = .0001
    jointWeights[4] = .0001
    jointWeights[5] = .0001

    aaa = .5
    #A = np.diag( np.append([jointWeights[i] for i in range(len(jointWeights))], np.zeros(totalProblemSize-totalDOF)) )
    A = np.diag( np.append([aaa for i in range(len(jointWeights))], np.zeros(totalProblemSize-totalDOF)) )
    #b = np.append(np.array([jointWeights[i]*ddth_des_flat[i] for i in range(len(jointWeights))]), np.zeros(totalProblemSize-totalDOF))
    b = np.append(np.array([aaa*ddth_des_flat[i] for i in range(len(jointWeights))]), np.zeros(totalProblemSize-totalDOF))
    qp.addObjective(A,b,weight)

def addQPTorqueTerms(qp, totalProblemSize, si, totalActuator, weight, jointTorqueWeights):
    # minimize |tau|^2
    A = np.diag(np.append(np.append(np.zeros((si)), 1.*np.ones((totalActuator))), np.zeros(totalProblemSize-si-totalActuator)))
    b = np.zeros(totalProblemSize)
    qp.addObjective(A,b,weight)

def addQPContactForceTerms(qp, totalProblemSize, si, totalContact, weight):
    # minimize |lambda|^2
    A = np.diag(np.append(np.zeros(si), 1.*np.ones(totalContact)))
    b = np.zeros(totalProblemSize)
    qp.addObjective(A,b,weight)

def addQPEqualityEomConstraint(qp, totalProblemSize, totalDOF, totalActuator, totalContact, M, c, JcTVc_append):
    # subject to Mq'' -tau - JcTVclambda = -b
    #                  tau[0:6)          =  0
    # [M -I -JcTVc]
    S = np.diag(np.append(np.zeros(6), 1.*np.ones(totalActuator-6)))
    A = np.vstack(( np.zeros((6,totalProblemSize)), np.hstack(( M,-S,-JcTVc_append )) ))
    for i in range(0, 6):
        A[i, totalDOF+i] = 1.
    b = np.append(np.zeros(6), -c)
    qp.addEqualityConstraint(A, b)

def addQPEqualityInverseEomConstraint(qp, totalProblemSize, totalDOF, totalActuator, totalContact, invM, invMc, JcTVc_append):
    # subject to Mq'' -tau - JcTVclambda = -b
    #                  tau[0:6)          =  0
    # [I -M^-1 -M^-1*JcTVc]
    S = np.diag(np.append(np.zeros(6), 1.*np.ones(totalActuator-6)))
    #A = np.vstack(( np.zeros((6,totalProblemSize)), np.hstack(( np.eye(totalDOF),-np.dot(invM, S),np.dot(invM, -JcTVc_append) )) ))
    A = np.vstack(( np.zeros((6,totalProblemSize)), np.hstack(( np.eye(totalDOF),-invM,-np.dot(invM, JcTVc_append) )) ))
    for i in range(0, 6):
        A[i, totalDOF+i] = 1.
    b = np.append(np.zeros(6), -invMc)
    #print(A[totalDOF:totalDOF+6])
    qp.addEqualityConstraint(A, b)

def addQPEqualityContactConstraint(qp, totalProblemSize, totalDOF, totalActuator, totalContact, Jc, dJc, dth, a_c):
    # subject to Jc q'' = -dJc q' + a_sup
    A = np.hstack( (Jc , np.zeros((6, totalActuator+totalContact))) )
    b = -np.dot(dJc, dth) + a_c
    qp.addEqualityConstraint(A, b)

def addQPInequalityTorqueConstraint(qp, totalProblemSize, totalDOF, totalActuator, totalContact, torqueMax, torqueMin):
    # subject to tau <= max and -tau <= min
    G_max = np.hstack((np.zeros((totalActuator-6, totalDOF+6)), np.diag(1.*np.ones(totalActuator-6)), np.zeros((totalActuator-6, totalContact)) ))
    G_min = -G_max
    G = np.vstack((G_max, G_min))
    h = np.append( torqueMax, -torqueMin)
    if G.shape[0] != h.shape[0]:
        print('Inequality Torque : G and h shapes are not matched')

    qp.addInequalityConstraint(G, h)

def addQPInequalityContactForceConstraint(qp, totalProblemSize, totalDOF, totalActuator, totalContact):
    # subject to -lambda <= 0
    G = -np.hstack((np.zeros((totalContact, totalDOF+totalActuator)), np.diag(1.*np.ones(totalContact)) ))
    h = np.zeros(totalContact)
    if G.shape[0] != h.shape[0]:
        print('Inequality Contact : G and h shapes are not matched')
    qp.addInequalityConstraint(G, h)

def addQPInequalityVelocityConstraint(qp, totalProblemSize, totalDOF, totalActuator, totalContact, VcTJc_list, VcTdJc_list, dVcTJc_list, dq, ac_offset_list, invdt):
    # subject to -(VcTJcq'' + VcTJc'q' + VcT'Jcq') <= 0
    #TODO:
    # assume that Vc' = 0 <- is it right? check it!
    G = None
    h = None
    for i in range(len(VcTJc_list)):
        G_temp = np.hstack( (-VcTJc_list[i], np.zeros((4, totalActuator+totalContact))) )
        #h_temp = np.dot(VcTdJc_list[i], dq) + (-.05)*np.ones(4)
        #h_temp = (-1/30.)*np.dot(VcTJc_list[i], dq)+ np.dot(VcTdJc_list[i], dq) + (-1.)*ac_offset_list[i]
        #h_temp = (-1/30.)*np.dot(VcTJc_list[i], dq)+ np.dot(VcTdJc_list[i], dq)
        h_temp = np.dot(dVcTJc_list[i], dq) +  np.dot(VcTdJc_list[i], dq) + np.dot(VcTJc_list[i], dq) * invdt + (-1.)*ac_offset_list[i]
        if G == None:
            G = G_temp.copy()
            h = h_temp.copy()
        else:
            G = np.vstack( (G, G_temp) )
            h = np.append( h, h_temp )

    if G.shape[0] != h.shape[0]:
        print('Inequality Velocity : G and h shapes are not matched')
    qp.addInequalityConstraint(G, h)
