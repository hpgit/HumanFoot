import numpy as np
import numpy.linalg as npl

import sys
#if '../PyCommon/modules' not in sys.path:
#    sys.path.append('../PyCommon/modules')
if './modules' not in sys.path:
    sys.path.append('./modules')

import Math.mmMath as mm
import Util.ysPythonEx as ype

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

def addSoftPointConstraintTerms(problem, totalDOF, weight, ddP_des, Q, q_bias):
    # min | ddP_des - (Q*ddth + q_bias) |^2
    problem.addObjective_matrix(Q, ddP_des - q_bias, weight)
        
def addConstraint(problem, totalDOF, J, dJ, dth_flat, a_sup):
    # subject to J_sup*ddth + dJ_sup*dth_flat = a_sup
    #problem.setConstraint_matrix(J, a_sup - np.dot(dJ, dth_flat))
    problem.addConstraint_matrix(J, a_sup - np.dot(dJ, dth_flat))
    
def addConstraint2(problem, totalDOF, A, b):
    # subject to A*ddth = b
    problem.addConstraint_matrix(A, b)
    
def setConstraint(problem, totalDOF, J, dJ, dth_flat, a_sup):
    # subject to J_sup*ddth + dJ_sup*dth_flat = a_sup
    problem.setConstraint_matrix(J, a_sup - np.dot(dJ, dth_flat))
    
def setConstraint2(problem, totalDOF, J, dth_flat, v_sup):
    # subject to J_sup*ddth + dJ_sup*dth_flat = a_sup
    problem.setConstraint_matrix(J, a_sup - np.dot(dJ, dth_flat))

