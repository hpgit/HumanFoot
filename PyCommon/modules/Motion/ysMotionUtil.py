import copy, math
import numpy as np 

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Motion.ysMotion as ym
import Math.mmMath as mmMath

#===============================================================================
# # skeleton
#===============================================================================
def getAllParentIndexes(skeleton):
    ls = [None]*skeleton.getElementNum()
    for i in range(skeleton.getElementNum()):
        ls[i] = getParentIndexes(skeleton, i)
    return ls
def getParentIndexes(skeleton, index):
    ls = []
    _addParentIndex(skeleton, index, ls)
    return ls 
def _addParentIndex(skeleton, index, ls):
    parentIndex = skeleton.getParentIndex(index)
    if parentIndex!=None:
        ls.append(parentIndex)
        _addParentIndex(skeleton, parentIndex, ls)

def getAllParentJointIndexes(skeleton):
    ls = [None]*skeleton.getJointNum()
    for i in range(skeleton.getJointNum()):
        ls[i] = getParentJointIndexes(skeleton, i)
    return ls
def getParentJointIndexes(skeleton, jointIndex):
    ls = []
    _addParentJointIndex(skeleton, jointIndex, ls)
    return ls 
def _addParentJointIndex(skeleton, jointIndex, ls):
    parentJointIndex = skeleton.getParentJointIndex(jointIndex)
    if parentJointIndex!=None:
        ls.append(parentJointIndex)
        _addParentJointIndex(skeleton, parentJointIndex, ls)
