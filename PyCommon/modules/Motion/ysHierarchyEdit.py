import numpy as np
import math

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm

import Motion.ysMotion as ym

def removeJoint(motion, joint_name_or_index, update=True):
    if isinstance(joint_name_or_index, int):
        removeIndex = joint_name_or_index
    else:
        removeIndex = motion[0].skeleton.getElementIndex(joint_name_or_index)

    skeleton = motion[0].skeleton
    children = skeleton.getChildIndexes(removeIndex)
    
    for posture in motion:
        for child in children:
            posture.setLocalR(child, np.dot(posture.getLocalR(removeIndex), posture.getLocalR(child)))

    skeleton.removeElement(removeIndex)
    for posture in motion:
        del posture.localRs[removeIndex:removeIndex+1]
        del posture.globalTs[removeIndex:removeIndex+1]
        if update:
            for child in children:
                posture.updateGlobalT(child)
            
def offsetJointLocal(motion, joint_name_or_index, offset, update=True):
    if isinstance(joint_name_or_index, int):
        index = joint_name_or_index
    else:
        index = motion[0].skeleton.getElementIndex(joint_name_or_index)
    
    skeleton = motion[0].skeleton
    skeleton.getElement(index).offset += offset
    
    if update:
        for posture in motion:
            posture.updateGlobalT(index)
        
def rotateJointLocal(motion, joint_name_or_index, R, update=True):
    if isinstance(joint_name_or_index, int):
        index = joint_name_or_index
    else:
        index = motion[0].skeleton.getElementIndex(joint_name_or_index)
        
    for posture in motion:
        posture.mulLocalR(index, R)
        if update:
            posture.updateGlobalT(index)
            
def updateGlobalT(motion):
    for posture in motion:
        posture.updateGlobalT()

        
def addJoint(motion, parent_joint_name_or_index, joint_name, offset=None, update=True):
    if isinstance(parent_joint_name_or_index, int):
        parentIndex = joint_name_or_index
    else:
        parentIndex = motion[0].skeleton.getElementIndex(parent_joint_name_or_index)
        
    skeleton = motion[0].skeleton
    
    parentJoint = skeleton.getElement(parentIndex)
    childOffset = (0, 0., 0.)
    if offset != None :
        childOffset = offset
    newJoint = parentJoint.addChild(joint_name, childOffset)#ym.Joint(joint_name, parentJoint)
    
    motion[0].skeleton.addElement(newJoint, joint_name)
    motion[0].skeleton.initialize()
    
    localR = mm.I_SO3()    
    for jointPosture in motion:
        jointPosture.addJoint(skeleton, localR)
        jointPosture.setLocalR(jointPosture.skeleton.getElementIndex(joint_name), localR)        
        jointPosture.updateGlobalT()



