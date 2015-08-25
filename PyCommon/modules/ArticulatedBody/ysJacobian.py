import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Motion.ysMotionUtil as ymu
import Util.ysPythonEx as ype

#===============================================================================
# Notice : 
# In this module, each joint DOF is assumed to be rotational DOF. 
# 1~3DOF rotational joint and 6DOF floating joint are supported.
#
# jacobian J
# rowNum : (number of effector dofs) * (effector number)
# colNum : number of joint dofs
#===============================================================================

# len(jointDOFs) : number of real joints
#    ex. jointDOFs = [3,3] : 2 ball joints 
# sum of all elements of jointDOFs : total DOF 
def makeEmptyJacobian(jointDOFs, effectorNum, applyOrientation = True):
    dof_per_effector = 3 if applyOrientation==False else 6
    rowNum = dof_per_effector * effectorNum
    colNum = 0
    for i in range(len(jointDOFs)):
        colNum += jointDOFs[i]
    return np.zeros((rowNum, colNum), float)

#===============================================================================
# # Treat one 3DOF joint as one 3DOF joint
#===============================================================================
# compute J that dx = J*dq
# Treat one 3DOF joint as one 3DOF joint
#
# All parameters are given w.r.t. the global coordinate system
# linearFirst=True : generalized velocity = [ v^t w^t ]^t, False : [ w^t v^t ]^t
# effectorJointMasks : 
# ex. len(jointPositions) == 3, len(effectorPositions) == 2
#     effectorJointMasks == [[1,1,1], [1,1,1]] : every effector affected by every joint
#     effectorJointMasks == [[1,0,0], [1,1,0]] : 1st effector affected by only 1st joint, 2nd effector affected by 1st & 2nd joints
# 
# (jointDOFs[0]==6) means joint[0] is 6DOF floating joint, jointAxeses[0] should be [(1,0,0), (0,1,0), (0,0,1), R^t[0], R^t[1], R^t[2]]
def computeJacobian2(J, jointDOFs, jointPositions, jointAxeses, effectorPositions, effectorJointMasks=None, linearFirst=True):
    rowNum, colNum = J.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6

    for e in range(len(effectorPositions)):
        col = 0
        
        for j in range(len(jointDOFs)):
            jointDOF_jth_joint = jointDOFs[j]
            jointPosition_jth_joint = jointPositions[j]
            jointAxes_jth_joint = jointAxeses[j]
    
            for d in range(jointDOF_jth_joint):
                if (effectorJointMasks==None or effectorJointMasks[e][j]) :
                    axis_colth_dof = jointAxes_jth_joint[d]
                    rotationalDOF = False if jointDOF_jth_joint==6 and d<3 else True
                    
                    if rotationalDOF:
                        instanteneousAngVelocity_colth_dof = axis_colth_dof 
                        instanteneousVelocity_colth_dof = np.cross(axis_colth_dof, effectorPositions[e] - jointPosition_jth_joint)
                    else:   # translationalDOF
                        instanteneousAngVelocity_colth_dof = [0.,0.,0.] 
                        instanteneousVelocity_colth_dof = axis_colth_dof
                else:
                    instanteneousAngVelocity_colth_dof = [0.,0.,0.] 
                    instanteneousVelocity_colth_dof = [0.,0.,0.]
        
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            J[e*dof_per_effector + i, col] = instanteneousVelocity_colth_dof[i]
                            J[e*dof_per_effector + 3 + i, col] = instanteneousAngVelocity_colth_dof[i]
                        else:
                            J[e*dof_per_effector + i, col] = instanteneousAngVelocity_colth_dof[i]
                            J[e*dof_per_effector + 3 + i, col] = instanteneousVelocity_colth_dof[i]
                    else:
                        J[e*dof_per_effector + i, col] = instanteneousVelocity_colth_dof[i]
                        
                col += 1

    
# compute dJ that ddx = dJ*dq + J*ddq
# Treat one 3DOF joint as one 3DOF joint
#
# All parameters are given w.r.t. the global coordinate system
# internalJointsOnly = False: joint[0] - link[0](root) - joint[1] - link[1] - ... - joint[n-1] - link[n-1] 
# internalJointsOnly = True : link[0](root) - joint[0] - link[1] - joint[1] - ... - joint[n-2] - link[n-1]
#
# jointDOFs[0] = 6 means joint[0] is 6DOF floating joint, jointAxeses[0] should be [(1,0,0), (0,1,0), (0,0,1), R^t[0], R^t[1], R^t[2]]
def computeJacobianDerivative2(dJ, jointDOFs, jointPositions, jointAxeses, linkAngVels, effectorPositions, effectorJointMasks, internalJointsOnly=False, linearFirst=True):
    rowNum, colNum = dJ.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6
    
    for e in range(len(effectorPositions)):
        col = 0

        for j in range(len(jointDOFs)):
            jointDOF_jth_joint = jointDOFs[j]
            jointPosition_jth_joint = jointPositions[j]
            jointAxes_jth_joint = jointAxeses[j]
    
            if internalJointsOnly:
                parentLinkAngVel_jth_joint = linkAngVels[j]
            else:
                parentLinkAngVel_jth_joint = linkAngVels[j-1] if j>0 else (0,0,0)
            
            for d in range(jointDOF_jth_joint):
                if effectorJointMasks==None or effectorJointMasks[e][j]:
                    axis_colth_dof = jointAxes_jth_joint[d]
                    rotationalDOF = False if jointDOF_jth_joint==6 and d<3 else True
                    
                    if rotationalDOF:
                        # dZ(i) = w(i)<cross>Z(i)
                        instanteneousAngAcceleration_colth_dof = np.cross(parentLinkAngVel_jth_joint, axis_colth_dof)
                        
                        # Z(i)<cross>(<sum j=i to n>w(j+1)<cross>P(j+1, j)) + (w(i)<cross>Z(i))<cross>P(n+1, i)
                        instanteneousAcceleration_colth_dof = np.cross(axis_colth_dof, get_dP_effector_from_joint2(j, jointPositions, linkAngVels, effectorJointMasks[e], effectorPositions[e], internalJointsOnly)) \
                                                              + np.cross(np.cross(parentLinkAngVel_jth_joint, axis_colth_dof), effectorPositions[e]-jointPosition_jth_joint)
                    else:   # translationalDOF
                        instanteneousAngAcceleration_colth_dof = [0.,0.,0.] 
                        instanteneousAcceleration_colth_dof = np.cross(parentLinkAngVel_jth_joint, axis_colth_dof)
                else:
                    instanteneousAngAcceleration_colth_dof = [0.,0.,0.] 
                    instanteneousAcceleration_colth_dof = [0.,0.,0.]                    

                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            dJ[e*dof_per_effector + i, col] = instanteneousAcceleration_colth_dof[i]
                            dJ[e*dof_per_effector + 3 + i, col] = instanteneousAngAcceleration_colth_dof[i]
                        else:
                            dJ[e*dof_per_effector + i, col] = instanteneousAngAcceleration_colth_dof[i]
                            dJ[e*dof_per_effector + 3 + i, col] = instanteneousAcceleration_colth_dof[i]
                    else:
                        dJ[e*dof_per_effector + i, col] = instanteneousAcceleration_colth_dof[i]
                        
                col += 1    

# P_effector_from_joint = effectorPositions[e]-jointPositions[j]
# dP_effector_from_joint = time derivative of P_effector_from_joint
# internalJointsOnly = False: joint[0] - link[0](root) - joint[1] - link[1] - ... - joint[n-1] - link[n-1] 
# internalJointsOnly = True : link[0](root) - joint[0] - link[1] - joint[1] - ... - joint[n-2] - link[n-1]
#
# <sum j=i to n>w(j+1)<cross>P(j+1, j)
def get_dP_effector_from_joint2(jointIndex, jointPositions, linkAngVels, effectorJointMask, effectorPosition, internalJointsOnly=False):
    jointIndexes = jointMask_2_jointIndexesDownward(effectorJointMask, jointIndex)

    dP = mm.v3(0,0,0)
    for i in range(len(jointIndexes)):
        index = jointIndexes[i]
        jointPosition = jointPositions[index]
        childPosition = jointPositions[jointIndexes[i+1]] if i < len(jointIndexes)-1 else effectorPosition
        linkAngVel = linkAngVels[index] if internalJointsOnly==False else linkAngVels[index+1]
        dP += np.cross(linkAngVel, childPosition - jointPosition)
    return dP 


#===============================================================================
# # Treat each DOF of 3DOF joint as one 1DOF joint    
#===============================================================================
# compute J that dx = J*dq
# Treat each DOF of 3DOF joint as one 1DOF joint
#
# All parameters are given w.r.t. the global coordinate system
# linearFirst=True : generalized velocity = [ v^t w^t ]^t, False : [ w^t v^t ]^t
# effectorJointMasks : 
# ex. len(jointPositions) == 3
#     len(effectorPositions) == 2
#     effectorJointMasks == [[1,1,1], [1,1,1]] : every effector affected by every joint
#     effectorJointMasks == [[1,0,0], [1,1,0]] : 1st effector affected by only 1st joint, 2nd effector affected by 1st & 2nd joints 
def computeJacobian(J, jointPositions, jointAxes, effectorPositions, effectorJointMasks=None, linearFirst=True):
    rowNum, colNum = J.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6
    
    for j in range(colNum):
        instanteneousAngVelocity_for_j_th_joint = jointAxes[j]
        
        for e in range(len(effectorPositions)):
            if effectorJointMasks==None or effectorJointMasks[e][j]:
                instanteneousVelocity_for_j_th_joint = np.cross(jointAxes[j], effectorPositions[e]-jointPositions[j])
                
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            J[e*dof_per_effector + i, j] = instanteneousVelocity_for_j_th_joint[i]
                            J[e*dof_per_effector + 3 + i, j] = instanteneousAngVelocity_for_j_th_joint[i]
                            
                            # don't affect to angular velocity
#                            J[e*dof_per_effector + 3 + i, j] = 0.
                        else:
                            # don't affect to angular velocity
#                            J[e*dof_per_effector + i, j] = jointAxes[j][i]

                            J[e*dof_per_effector + i, j] = instanteneousAngVelocity_for_j_th_joint[i]
                            J[e*dof_per_effector + 3 + i, j] = instanteneousVelocity_for_j_th_joint[i]
                    else:
                        J[e*dof_per_effector + i, j] = instanteneousVelocity_for_j_th_joint[i]
            else:
                for i in range(dof_per_effector):
                    J[e*dof_per_effector + i, j] = 0.
                        
# compute dJ that ddx = dJ*dq + J*ddq
# Treat each DOF of 3DOF joint as one 1DOF joint
#
# All parameters are given w.r.t. the global coordinate system
# internalJointsOnly = False: joint[0] - link[0](root) - joint[1] - link[1] - ... - joint[n-1] - link[n-1] 
# internalJointsOnly = True : link[0](root) - joint[0] - link[1] - joint[1] - ... - joint[n-2] - link[n-1]
def computeJacobianDerivative(dJ, jointPositions, jointAxes, linkAngVels, effectorPositions, effectorJointMasks, internalJointsOnly=False, linearFirst=True):
    rowNum, colNum = dJ.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6
    
    for j in range(colNum):
        if not internalJointsOnly:
            if j>=0 and j<=2:
                parentLinkAngVel = mm.v3(0,0,0)  
            else:
                parentLinkAngVel = linkAngVels[j-3]
        else:
            parentLinkAngVel = linkAngVels[j]

        # dZ(i) = w(i)<cross>Z(i)
        instanteneousAngAcceleration_for_j_th_joint = np.cross(parentLinkAngVel, jointAxes[j])
        
        for e in range(len(effectorPositions)):
            if effectorJointMasks[e][j]:
                instanteneousAcceleration_for_j_th_joint = np.cross(jointAxes[j], get_dP_effector_from_joint(j, jointPositions, linkAngVels, effectorJointMasks[e], effectorPositions[e], internalJointsOnly)) \
                                                        + np.cross(np.cross(linkAngVels[j], jointAxes[j]), effectorPositions[e]-jointPositions[j])
                 
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            dJ[e*dof_per_effector + i, j] = instanteneousAcceleration_for_j_th_joint[i]
                            dJ[e*dof_per_effector + 3 + i, j] = instanteneousAngAcceleration_for_j_th_joint[i]
                        else:
                            dJ[e*dof_per_effector + i, j] = instanteneousAngAcceleration_for_j_th_joint[i]
                            dJ[e*dof_per_effector + 3 + i, j] = instanteneousAcceleration_for_j_th_joint[i]
                    else:
                        dJ[e*dof_per_effector + i, j] = instanteneousAcceleration_for_j_th_joint[i]
            else:
                for i in range(dof_per_effector):
                    dJ[e*dof_per_effector + i, j] = 0.

# P_effector_from_joint = effectorPositions[e]-jointPositions[j]
# dP_effector_from_joint = time derivative of P_effector_from_joint
# internalJointsOnly = False: joint[0] - link[0](root) - joint[1] - link[1] - ... - joint[n-1] - link[n-1] 
# internalJointsOnly = True : link[0](root) - joint[0] - link[1] - joint[1] - ... - joint[n-2] - link[n-1]
#
# <sum j=i to n>w(j+1)<cross>P(j+1, j)
def get_dP_effector_from_joint(jointIndex, jointPositions, linkAngVels, effectorJointMask, effectorPosition, internalJointsOnly=False):
    jointIndexes = jointMask_2_jointIndexesDownward(effectorJointMask, jointIndex)

    dP = mm.v3(0,0,0)
    for i in range(len(jointIndexes)):
        index = jointIndexes[i]
        jointPosition = jointPositions[index]
        childPosition = jointPositions[jointIndexes[i+1]] if i < len(jointIndexes)-1 else effectorPosition
        linkAngVel = linkAngVels[index] if internalJointsOnly==False else linkAngVels[index+3]
        dP += np.cross(linkAngVel, childPosition - jointPosition)
    return dP 


#===============================================================================
# utility function
#===============================================================================
def jointMask_2_jointIndexesDownward(effectorJointMask, startJointIndex=None):
    if startJointIndex==None:
        return [i for i in range(len(effectorJointMask)) if effectorJointMask[i]==1]
    else:
        return [i for i in range(startJointIndex, len(effectorJointMask)) if effectorJointMask[i]==1]

def getRepeatedEffectorJointMasks(jointDOFs, effectorJointMasks):
    totalDOF = 0
    for i in range(len(jointDOFs)):
        totalDOF += jointDOFs[i]
    
    repeatedMasks = [None]*len(effectorJointMasks)
    for i in range(len(effectorJointMasks)):
        repeated_per_joint = [None]*totalDOF
        ype.repeatListElements(effectorJointMasks[i], repeated_per_joint, jointDOFs)
        repeatedMasks[i] = repeated_per_joint
    
    return repeatedMasks

# linkNum == 3, jointNum == 3
# return mask == [[1,1,1], [1,1,1]] : every link affected by every joint
# return mask == [[1,0,0], [1,1,0]] : link0 affected by only joint0, link1 affected by joint0, joint1 
def getLinkJointMask(skeleton, linkIndex):
    jointNum = skeleton.getJointNum()
    mask = [0]*jointNum
    
    # get parent joints of joint at linkIndex
    affectingJointIndexes = ymu.getParentJointIndexes(skeleton, linkIndex)
    
    # append body own index
    affectingJointIndexes.append(linkIndex)
    
    # fill masks
    for jointIndex in affectingJointIndexes:
        mask[jointIndex] = 1
        
    return mask

def getLinksJointMask(skeleton, linkIndeses):
    jointNum = skeleton.getJointNum()
    mask = [0]*jointNum
    
    for i in range(len(linkIndeses)):
        # get parent joints of joint at linkIndex
        affectingJointIndexes = ymu.getParentJointIndexes(skeleton, linkIndeses[i])
    
        # append body own index
        affectingJointIndexes.append(linkIndeses[i])
    
        # fill masks
        for jointIndex in affectingJointIndexes:
            mask[jointIndex] = 1

        
    return mask
    
# linkNum == 3, jointNum == 3
# return mask == [[1,1], [1,1]] : every link affected by every joint
# return mask == [[1,0], [1,1]] : link0 affected by only joint1(position 0 in internal joint indexes), link1 affected by joint1, joint2 
def getLinkInternalJointMask(skeleton, linkIndex):
    jointNum = skeleton.getInternalJointNum()
    mask = [0]*jointNum
    
    # get parent joints of joint at linkIndex
    affectingJointIndexes = ymu.getParentJointIndexes(skeleton, linkIndex)
    
    # append body own index
    affectingJointIndexes.append(linkIndex)
    
    # fill masks
    for jointIndex in affectingJointIndexes:
        if jointIndex!=0:   # don't mark at root joint
            mask[jointIndex-1] = 1
        
    return mask

def getAllLinkJointMasks(skeleton):
    linkNum = skeleton.getLinkNum()
    masks = [None]*linkNum
    for i in range(linkNum):
        masks[i] = getLinkJointMask(skeleton, i)
    return masks
    
def getAllLinkInternalJointMasks(skeleton):
    linkNum = skeleton.getLinkNum()
    masks = [None]*linkNum
    for i in range(linkNum):
        masks[i] = getLinkInternalJointMask(skeleton, i)
    return masks
    
def computeAngJacobian2(J, jointDOFs, jointPositions, jointAxeses, effectorPositions, effectorJointMasks=None, linearFirst=True):
    rowNum, colNum = J.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6

    for e in range(len(effectorPositions)):
        col = 0
        
        for j in range(len(jointDOFs)):
            jointDOF_jth_joint = jointDOFs[j]
            jointPosition_jth_joint = jointPositions[j]
            jointAxes_jth_joint = jointAxeses[j]
    
            for d in range(jointDOF_jth_joint):
                if effectorJointMasks==None or effectorJointMasks[e][j]:
                    axis_colth_dof = jointAxes_jth_joint[d]
                    rotationalDOF = False if jointDOF_jth_joint==6 and d<3 else True
                    
                    if rotationalDOF:
                        instanteneousAngVelocity_colth_dof = axis_colth_dof 
                        instanteneousVelocity_colth_dof = np.cross(axis_colth_dof, effectorPositions[e] - jointPosition_jth_joint)
                    else:   # translationalDOF
                        instanteneousAngVelocity_colth_dof = (0.,0.,0.) 
                        instanteneousVelocity_colth_dof = axis_colth_dof
                else:
                    instanteneousAngVelocity_colth_dof = (0.,0.,0.) 
                    instanteneousVelocity_colth_dof = (0.,0.,0.)
        
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            J[e*dof_per_effector + i, col] = instanteneousVelocity_colth_dof[i]
                            J[e*dof_per_effector + 3 + i, col] = instanteneousAngVelocity_colth_dof[i]
                        else:
                            J[e*dof_per_effector + i, col] = instanteneousAngVelocity_colth_dof[i]
                            J[e*dof_per_effector + 3 + i, col] = instanteneousVelocity_colth_dof[i]
                    else:
                        J[e*dof_per_effector + i, col] = instanteneousAngVelocity_colth_dof[i]
                        
                col += 1


def computeAngJacobianDerivative2(dJ, jointDOFs, jointPositions, jointAxeses, linkAngVels, effectorPositions, effectorJointMasks, internalJointsOnly=False, linearFirst=True):
    rowNum, colNum = dJ.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6
    
    for e in range(len(effectorPositions)):
        col = 0

        for j in range(len(jointDOFs)):
            jointDOF_jth_joint = jointDOFs[j]
            jointPosition_jth_joint = jointPositions[j]
            jointAxes_jth_joint = jointAxeses[j]
    
            if internalJointsOnly:
                parentLinkAngVel_jth_joint = linkAngVels[j]
            else:
                parentLinkAngVel_jth_joint = linkAngVels[j-1] if j>0 else (0,0,0)
            
            for d in range(jointDOF_jth_joint):
                if effectorJointMasks==None or effectorJointMasks[e][j]:
                    axis_colth_dof = jointAxes_jth_joint[d]
                    rotationalDOF = False if jointDOF_jth_joint==6 and d<3 else True
                    
                    if rotationalDOF:
                        # dZ(i) = w(i)<cross>Z(i)
                        instanteneousAngAcceleration_colth_dof = np.cross(parentLinkAngVel_jth_joint, axis_colth_dof)
                        
                        # Z(i)<cross>(<sum j=i to n>w(j+1)<cross>P(j+1, j)) + (w(i)<cross>Z(i))<cross>P(n+1, i)
                        instanteneousAcceleration_colth_dof = np.cross(axis_colth_dof, get_dP_effector_from_joint2(j, jointPositions, linkAngVels, effectorJointMasks[e], effectorPositions[e], internalJointsOnly)) \
                                                              + np.cross(np.cross(parentLinkAngVel_jth_joint, axis_colth_dof), effectorPositions[e]-jointPosition_jth_joint)
                    else:   # translationalDOF
                        instanteneousAngAcceleration_colth_dof = (0.,0.,0.) 
                        instanteneousAcceleration_colth_dof = np.cross(parentLinkAngVel_jth_joint, axis_colth_dof)
                else:
                    instanteneousAngAcceleration_colth_dof = (0.,0.,0.) 
                    instanteneousAcceleration_colth_dof = (0.,0.,0.)
                    
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            dJ[e*dof_per_effector + i, col] = instanteneousAcceleration_colth_dof[i]
                            dJ[e*dof_per_effector + 3 + i, col] = instanteneousAngAcceleration_colth_dof[i]
                        else:
                            dJ[e*dof_per_effector + i, col] = instanteneousAngAcceleration_colth_dof[i]
                            dJ[e*dof_per_effector + 3 + i, col] = instanteneousAcceleration_colth_dof[i]
                    else:
                        dJ[e*dof_per_effector + i, col] = instanteneousAngAcceleration_colth_dof[i]
                        
                col += 1  

def computePartialJacobian2(J, jointDOFs, jointPositions, jointAxeses, effectorPositions, effectorJointMasks=None, partialDOFIndex = [0,0], linearFirst=True):
    rowNum, colNum = J.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6

    for e in range(len(effectorPositions)):
        col = 0
        
        for j in range(len(jointDOFs)):
            jointDOF_jth_joint = jointDOFs[j]
            jointPosition_jth_joint = jointPositions[j]
            jointAxes_jth_joint = jointAxeses[j]
    
            for d in range(jointDOF_jth_joint):
                if (effectorJointMasks==None or effectorJointMasks[e][j]) and (j < partialDOFIndex[0] or j >= partialDOFIndex[1] or (j >= partialDOFIndex[0] and j < partialDOFIndex[1] and d == 0) ):
                    axis_colth_dof = jointAxes_jth_joint[d]
                    rotationalDOF = False if jointDOF_jth_joint==6 and d<3 else True
                    
                    if rotationalDOF:
                        instanteneousAngVelocity_colth_dof = axis_colth_dof 
                        instanteneousVelocity_colth_dof = np.cross(axis_colth_dof, effectorPositions[e] - jointPosition_jth_joint)
                    else:   # translationalDOF
                        instanteneousAngVelocity_colth_dof = [0.,0.,0.] 
                        instanteneousVelocity_colth_dof = axis_colth_dof
                else:
                    instanteneousAngVelocity_colth_dof = [0.,0.,0.] 
                    instanteneousVelocity_colth_dof = [0.,0.,0.]
        
                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            J[e*dof_per_effector + i, col] = instanteneousVelocity_colth_dof[i]
                            J[e*dof_per_effector + 3 + i, col] = instanteneousAngVelocity_colth_dof[i]
                        else:
                            J[e*dof_per_effector + i, col] = instanteneousAngVelocity_colth_dof[i]
                            J[e*dof_per_effector + 3 + i, col] = instanteneousVelocity_colth_dof[i]
                    else:
                        J[e*dof_per_effector + i, col] = instanteneousVelocity_colth_dof[i]
                        
                col += 1
                    
def computePartialJacobianDerivative2(dJ, jointDOFs, jointPositions, jointAxeses, linkAngVels, effectorPositions, effectorJointMasks, internalJointsOnly=False, partialDOFIndex = [0,0], linearFirst=True):
    rowNum, colNum = dJ.shape
    dof_per_effector = rowNum / len(effectorPositions)   # dof_per_effector = 3 if applyOrientation==False else 6
    
    for e in range(len(effectorPositions)):
        col = 0

        for j in range(len(jointDOFs)):
            jointDOF_jth_joint = jointDOFs[j]
            jointPosition_jth_joint = jointPositions[j]
            jointAxes_jth_joint = jointAxeses[j]
    
            if internalJointsOnly:
                parentLinkAngVel_jth_joint = linkAngVels[j]
            else:
                parentLinkAngVel_jth_joint = linkAngVels[j-1] if j>0 else (0,0,0)
            
            for d in range(jointDOF_jth_joint):
                if (effectorJointMasks==None or effectorJointMasks[e][j]) and (j < partialDOFIndex[0] or j >= partialDOFIndex[1] or (j >= partialDOFIndex[0] and j < partialDOFIndex[1] and d == 0) ):
                    axis_colth_dof = jointAxes_jth_joint[d]
                    rotationalDOF = False if jointDOF_jth_joint==6 and d<3 else True
                    
                    if rotationalDOF:
                        # dZ(i) = w(i)<cross>Z(i)
                        instanteneousAngAcceleration_colth_dof = np.cross(parentLinkAngVel_jth_joint, axis_colth_dof)
                        
                        # Z(i)<cross>(<sum j=i to n>w(j+1)<cross>P(j+1, j)) + (w(i)<cross>Z(i))<cross>P(n+1, i)
                        instanteneousAcceleration_colth_dof = np.cross(axis_colth_dof, get_dP_effector_from_joint2(j, jointPositions, linkAngVels, effectorJointMasks[e], effectorPositions[e], internalJointsOnly)) \
                                                              + np.cross(np.cross(parentLinkAngVel_jth_joint, axis_colth_dof), effectorPositions[e]-jointPosition_jth_joint)
                    else:   # translationalDOF
                        instanteneousAngAcceleration_colth_dof = [0.,0.,0.] 
                        instanteneousAcceleration_colth_dof = np.cross(parentLinkAngVel_jth_joint, axis_colth_dof)
                else:
                    instanteneousAngAcceleration_colth_dof = [0.,0.,0.] 
                    instanteneousAcceleration_colth_dof = [0.,0.,0.]                    

                for i in range(3):
                    if dof_per_effector == 6:
                        if linearFirst:
                            dJ[e*dof_per_effector + i, col] = instanteneousAcceleration_colth_dof[i]
                            dJ[e*dof_per_effector + 3 + i, col] = instanteneousAngAcceleration_colth_dof[i]
                        else:
                            dJ[e*dof_per_effector + i, col] = instanteneousAngAcceleration_colth_dof[i]
                            dJ[e*dof_per_effector + 3 + i, col] = instanteneousAcceleration_colth_dof[i]
                    else:
                        dJ[e*dof_per_effector + i, col] = instanteneousAcceleration_colth_dof[i]
                        
                col += 1    
