import copy, math
import numpy as np
import operator as op

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Util.ysPythonEx as ype

#===============================================================================
# motion classes
#===============================================================================
class MotionSystem:
    def __init__(self):
        self.motions = []
    def addMotion(self, motion):
        self.motions.append(motion)
    def removeAllMotions(self):
        del self.motions[:]
    def updateFrame(self, frame):
        for i in range(len(self.motions)):
            self.motions[i].goToFrame(frame)
    def getNumMotion(self):
        return len(self.motions)
    def getMaxFrame(self):
        maxFrame = 0
        for motion in self.motions:
            if maxFrame < len(motion)-1:
                maxFrame = len(motion)-1
        return maxFrame

            
#===============================================================================
# base classes
#===============================================================================
class Motion(list):
    def __init__(self, ls=[]):
        list.__init__(self, ls)
        self.frame = 0
        self.fps = 30.
        self.resourceName = 'unnamed'
    def getState(self):
        return self.frame
    def setState(self, state):
        self.frame = state
    def __getslice__(self, i, j):
        motion = self.__new__(self.__class__)
        motion.__init__(list.__getslice__(self, i, j))
        motion.fps = self.fps
        motion.resourceName = self.resourceName
        return motion
    def __add__(self, nextMotion):
        motion = self.__new__(self.__class__)
        motion.__init__(list.__add__(self, nextMotion))
        motion.fps = self.fps
        motion.resourceName = self.resourceName
        return motion
    def goToFrame(self, frame):
        if frame > -1 and frame < len(self):
            self.frame = frame
    def getPostureAt(self, frame):
        floor = int(frame)
        t = frame - floor
        if t==0.0:
            return self[floor]
        else:
            return self[floor].blendPosture(self[floor+1], t)
        
    def getFiniteDifferenceFrames(self, frame):
        prevFrame = frame-1 if frame > 0 else frame
        nextFrame  = frame+1 if frame < len(self)-1 else frame
        return prevFrame, nextFrame
    
    def getPosition(self, index, frame):
        return self.getPostureAt(frame).getPosition(index)
    def getPositions(self, frame):
        return self.getPostureAt(frame).getPositions()
    def getVelocity(self, index, frame0, frame1=None):
        return self._getDerivative(index, frame0, frame1, self.getPosition, op.sub)
    def getVelocities(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getPositions, op.sub)
    def getAcceleration(self, index, frame0, frame1=None):
        return self._getDerivative(index, frame0, frame1, self.getVelocity, op.sub)
    def getAccelerations(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getVelocities, op.sub)

    def _getDerivative(self, index, frame0, frame1, positionFunc, subFunc):
        if frame0 == frame1 or len(self)==1:
            return mm.O_Vec3()
        
        if frame1 == None:
            frame0, frame1 = self.getFiniteDifferenceFrames(frame0)
        p0 = positionFunc(index, frame0)
        p1 = positionFunc(index, frame1)
        return (self.fps/(frame1-frame0)) * subFunc(p1, p0)
    def _getDerivatives(self, frame0, frame1, positionsFunc, subFunc):
        if frame0 == frame1 or len(self)==1:
            return [mm.O_Vec3()]*self[0].skeleton.getElementNum()
        
        if frame1 == None:
            frame0, frame1 = self.getFiniteDifferenceFrames(frame0)
        positions0 = positionsFunc(frame0)
        positions1 = positionsFunc(frame1)
        return map(lambda p0,p1: (self.fps/(frame1-frame0)) * subFunc(p1, p0), positions0, positions1)
    
    def translateByOffset(self, offset):
        for posture in self:
            posture.translateByOffset(offset)
            
class Skeleton:
    def __init__(self):
        self.elements = []
        self.elementNames = []
        self.reverseElementNames = {}
    def __str__(self):
        s = ''
#        s += self.elementNames.__str__() + '\n'
        s += '<ELEMENTS>\n'
        for i in range(len(self.elementNames)):
            s += '[%d]:\'%s\', '%(i, self.elementNames[i])
        s += '\n'
        return s
    def addElement(self, element, name):
        self.elements.append(element)
        if name!=None:
            self.elementNames.append(name)
            self.reverseElementNames[name] = len(self.elementNames)-1
        
    def getElement(self, index):
        return self.elements[index]
    def getElementNum(self):
        return len(self.elements)
    def getElementName(self, index):
        if index < len(self.elementNames):
            return self.elementNames[index]
        else:
            return None
    def getElementIndex(self, name):
        if name in self.reverseElementNames:
            return self.reverseElementNames[name]
        else:
            return None
    def removeElement(self, index):
        del self.elements[index:index+1]
        del self.elementNames[index:index+1]
        
        self.reverseElementNames.clear()
        for i in range(len(self.elements)):
            self.reverseElementNames[self.elementNames[i]] = i 
        
class Posture:
    def __init__(self, skeleton):
        self.skeleton = skeleton
    def getPosition(self, index):
        raise NotImplementedError
    def getPositions(self):
        raise NotImplementedError
    def blendPosture(self, otherPosture, t):
        raise NotImplementedError
    def translateByOffset(self, offset):
        raise NotImplementedError


#===============================================================================
# joint motion
#
# number of links: n <-> number of joints: n (including imaginary root joint)
#                    <-> number of internal joints: n-1
# 
# parent        <--->        child
# joint[0] - link[0] - joint[1] - link[1] - ... - joint[n-1] - link[n-1]
#
# joint[0]: (imaginary root joint)
# link[0]: (root body) 
#===============================================================================
class JointMotion(Motion):
    # lv : linear velocity, av : angular velocity, la : linear acceleration, aa : angular acceleration 
    # p: Vec3(position), R : SO3(orientation)
    # _g : w.r.t. global frame, _l : w.r.t. local frame
    # x[0] : x of joint[0]
    
    # [(p_g[0], R_g[0]), R_l[1], R_l[2], ... ,R_l[n-1]]
    def getDOFPositions(self, frame):
#        return [self[frame].getGlobalT(0)] + self.getInternalJointOrientationsLocal(frame)
        return [(self[frame].rootPos, self[frame].getLocalR(0))] + self.getInternalJointOrientationsLocal(frame)
    
    # [(p_l[0], R_l[0]), R_l[1], R_l[2], ... ,R_l[n-1]]
    def getDOFPositionsLocal(self, frame):
#        return [self[frame].getGlobalT(0)] + self.getInternalJointOrientationsLocal(frame)
        return [(np.dot(self[frame].getLocalR(0).T, self[frame].rootPos), self[frame].getLocalR(0))] + self.getInternalJointOrientationsLocal(frame)

    def getDOFPositionsEuler(self, frame):
        positions = self.getDOFPositions(frame)
        positionsEuler = []
        positionsEuler.append(np.concatenate((positions[0][0],mm.R2ZXY(positions[0][1]))))
        for i in range(1,len(positions)): 
            positionsEuler.append(mm.R2ZXY(positions[i]))
        return positionsEuler

    # [lv_g[0]<hmerge>av_l[0], av_l[1], av_l[2], ... av_l[n-1]]
    def getDOFVelocities(self, frame):
#        return [np.concatenate( (self.getJointVelocityGlobal(0, frame), self.getJointAngVelocityGlobal(0, frame)) )]\
#                + self.getInternalJointAngVelocitiesLocal(frame) 
        return [np.concatenate( (self.getJointVelocityGlobal(0, frame), self.getJointAngVelocityLocal(0, frame)) )]\
                + self.getInternalJointAngVelocitiesLocal(frame)
                
    # [lv_l[0]<hmerge>av_l[0], av_l[1], av_l[2], ... av_l[n-1]]
    def getDOFVelocitiesLocal(self, frame):
        return [np.concatenate( (np.dot(self[frame].getLocalR(0).T, self.getJointVelocityGlobal(0, frame)), self.getJointAngVelocityLocal(0, frame)) )]\
                + self.getInternalJointAngVelocitiesLocal(frame)

    def getDOFVelocitiesEuler(self, frame):
        #if frame == 0:
            #frame = frame+1
        #positionEuler0 = self.getDOFPositionsEuler(frame-1)
        #positionEuler1 = self.getDOFPositionsEuler(frame)
        #position0 = self.getDOFVelocities(frame)
        #dotR = np.dot(self[frame].getLocalR(0), self[frame].getLocalR(0))
        #velocities = []
        #for i in range(0,len(position0)):
            #velocities.append(30.*(positionEuler1[i]-positionEuler0[i]))
        velocities = self.getDOFVelocities(frame)
        velocityRoot = velocities[0].copy()
        velocities[0][3] = velocityRoot[5] #Z
        velocities[0][4] = velocityRoot[3] #X
        velocities[0][5] = velocityRoot[4] #Y
        for i in range(1, len(velocities)):
            velocity = velocities[i].copy()
            velocities[i][0] = velocity[2]
            velocities[i][1] = velocity[0]
            velocities[i][2] = velocity[1]
        return velocities

    # [la_g[0]<hmerge>aa_l[0], aa_l[1], aa_l[2], ... aa_l[n-1]] 
    def getDOFAccelerations(self, frame):
#        return [np.concatenate( (self.getJointAccelerationGlobal(0, frame), self.getJointAngAccelerationGlobal(0, frame)) )]\
#                + self.getInternalJointAngAccelerationsLocal(frame) 
        return [np.concatenate( (self.getJointAccelerationGlobal(0, frame), self.getJointAngAccelerationLocal(0, frame)) )]\
                + self.getInternalJointAngAccelerationsLocal(frame)
                 
    # [la_l[0]<hmerge>aa_l[0], aa_l[1], aa_l[2], ... aa_l[n-1]] 
    def getDOFAccelerationsLocal(self, frame):
        return [np.concatenate( (np.dot(self[frame].getLocalR(0).T, self.getJointAccelerationGlobal(0, frame)), self.getJointAngAccelerationLocal(0, frame)) )]\
                + self.getInternalJointAngAccelerationsLocal(frame)

    def getDOFAccelerationsEuler(self, frame):
        #if frame == 0:
            #frame = frame+1
        #velocity0 = self.getDOFVelocitiesEuler(frame-1)
        #velocity1 = self.getDOFVelocitiesEuler(frame)
        #accelerations = []
        #for i in range(0,len(velocity0)):
            #accelerations.append(30.*(velocity1[i]-velocity1[i]))
        #return accelerations
        accelerations = self.getDOFAccelerations(frame)
        accelerationRoot = accelerations[0].copy()
        accelerations[0][3] = accelerationRoot[5] #Z
        accelerations[0][4] = accelerationRoot[3] #X
        accelerations[0][5] = accelerationRoot[4] #Y
        for i in range(1, len(accelerations)):
            acceleration = accelerations[i].copy()
            accelerations[i][0] = acceleration[2]
            accelerations[i][1] = acceleration[0]
            accelerations[i][2] = acceleration[1]
        return accelerations

    # [I<vmerge>R_g[0], R_l[1]^t, R_l[2]^t, ... R_l[n-1]^t]
    def getDOFAxeses(self, frame):
#        return [np.concatenate((mm.I_SO3(), mm.I_SO3()))] + [R.transpose() for R in self.getInternalJointOrientationsGlobal(frame)] 
        return [np.concatenate((mm.I_SO3(), self[frame].getJointOrientationGlobal(0).transpose()))] + [R.transpose() for R in self.getInternalJointOrientationsGlobal(frame)]
    
    # [I<vmerge>R_g[0], R_l[1]^t, R_l[2]^t, ... R_l[n-1]^t]
    def getDOFAxesesLocal(self, frame):
#        return [np.concatenate((mm.I_SO3(), mm.I_SO3()))] + [R.transpose() for R in self.getInternalJointOrientationsGlobal(frame)] 
        return [np.concatenate((mm.I_SO3(), self[frame].getJointOrientationGlobal(0).transpose()))] + [R.transpose() for R in self.getInternalJointOrientationsGlobal(frame)]
    
    def setDOFPositions(self, frame, DOFPositions):
#        R, p = mm.T2Rp(DOFPositions[0])
#        self[frame].rootPos = p
#        self.setJointOrientationsLocal(frame, [R]+DOFPositions[1:])
        self[frame].rootPos = DOFPositions[0][0]
        self.setJointOrientationsLocal(frame, [DOFPositions[0][1]]+DOFPositions[1:])
    
    def getJointOrientationLocal(self, jointIndex, frame):
        return self[frame].getJointOrientationLocal(jointIndex)
    def getJointAngVelocityLocal(self, jointIndex, frame0, frame1=None):
        return self._getDerivative(jointIndex, frame0, frame1, self.getJointOrientationLocal, lambda R1, R0: mm.logSO3(np.dot(R0.transpose(), R1)))
    def getJointAngAccelerationLocal(self, jointIndex, frame0, frame1=None):
        return self._getDerivative(jointIndex, frame0, frame1, self.getJointAngVelocityLocal, op.sub)

    def getJointPositionGlobal(self, jointIndex, frame):
        return self[frame].getJointPositionGlobal(jointIndex)
    def getJointVelocityGlobal(self, jointIndex, frame0, frame1=None):
        return self._getDerivative(jointIndex, frame0, frame1, self.getJointPositionGlobal, op.sub)
    def getJointAccelerationGlobal(self, jointIndex, frame0, frame1=None):
        return self._getDerivative(jointIndex, frame0, frame1, self.getJointVelocityGlobal, op.sub)
    def getJointOrientationGlobal(self, jointIndex, frame):
        return self[frame].getJointOrientationGlobal(jointIndex)
    def getJointAngVelocityGlobal(self, jointIndex, frame0, frame1=None):
        return self._getDerivative(jointIndex, frame0, frame1, self.getJointOrientationGlobal, lambda R1, R0: np.dot(R0, mm.logSO3(np.dot(R0.transpose(), R1))))
    def getJointAngAccelerationGlobal(self, jointIndex, frame0, frame1=None):
        return self._getDerivative(jointIndex, frame0, frame1, self.getJointAngVelocityGlobal, op.sub)
    
    def setJointPositionGlobal(self, jointIndex, frame, position):
        return self[frame].setJointPositionGlobal(jointIndex, position)
    def setJointOrientationGlobal(self, jointIndex, frame, R):
        return self[frame].setJointOrientationGlobal(jointIndex, R)

    def getJointOrientationsLocal(self, frame):
        return self[frame].getJointOrientationsLocal()
    def getJointAngVelocitiesLocal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getJointOrientationsLocal, lambda R1, R0: mm.logSO3(np.dot(R0.transpose(), R1)))
    def getJointAngAccelerationsLocal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getJointAngVelocitiesLocal, op.sub)
    
    def setJointOrientationsLocal(self, frame, Rs):
        self[frame].setJointOrientationsLocal(Rs)
    
    def getJointPositionsGlobal(self, frame):
        return self[frame].getJointPositionsGlobal()
    def getJointVelocitiesGlobal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getJointPositionsGlobal, op.sub)
    def getJointAccelerationsGlobal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getJointVelocitiesGlobal, op.sub)
    def getJointOrientationsGlobal(self, frame):
        return self[frame].getJointOrientationsGlobal()
    def getJointAngVelocitiesGlobal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getJointOrientationsGlobal, lambda R1, R0: np.dot(R0, mm.logSO3(np.dot(R0.transpose(), R1))))
    def getJointAngAccelerationsGlobal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getJointAngVelocitiesGlobal, op.sub)
    
    def getInternalJointOrientationsLocal(self, frame):
        return self[frame].getInternalJointOrientationsLocal()
    def getInternalJointAngVelocitiesLocal(self, frame0, frame1=None):
#        # m2 - m1 : m2.__sub__(m1)
#        # d = m2 - m1
#        # R_d = inverse(R_m1) (dot) R_m2
#        orientations0 = self.getOrientationsLocal(frame0)
#        orientations1 = self.getOrientationsLocal(frame1)
#        return map(lambda R0,R1: (self.fps/(frame1-frame0)) * mm.logSO3(np.dot(R0.transpose(), R1)), orientations0, orientations1)
        return self._getDerivatives(frame0, frame1, self.getInternalJointOrientationsLocal, lambda R1, R0: mm.logSO3(np.dot(R0.transpose(), R1)))
    def getInternalJointAngAccelerationsLocal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getInternalJointAngVelocitiesLocal, op.sub)

    def getInternalJointPositionsGlobal(self, frame):
        return self[frame].getInternalJointPositionsGlobal()
    def getInternalJointVelocitiesGlobal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getInternalJointPositionsGlobal, op.sub)
    def getInternalJointAccelerationsGlobal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getInternalJointVelocitiesGlobal, op.sub)
    def getInternalJointOrientationsGlobal(self, frame):
        return self[frame].getInternalJointOrientationsGlobal()
    def getInternalJointAngVelocitiesGlobal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getInternalJointOrientationsGlobal, lambda R1, R0: np.dot(R0, mm.logSO3(np.dot(R0.transpose(), R1))))
    def getInternalJointAngAccelerationsGlobal(self, frame0, frame1=None):
        return self._getDerivatives(frame0, frame1, self.getInternalJointAngVelocitiesGlobal, op.sub)
    
class JointSkeleton(Skeleton):
    def __init__(self, root):
        Skeleton.__init__(self)
        self.root = root
        self.rootIndex = None   # root always should be self.elements[0] and self.jointElementIndexes[0]
        self.jointElementIndexes = []
        self.reverseJointElementIndexes = {}
    def initialize(self):
        # build jointElementIndexes
        del self.jointElementIndexes[:]
        for i in range(self.getElementNum()):
            joint = self.elements[i]
            if len(joint.children)>0:
                self.jointElementIndexes.append(i)
                self.reverseJointElementIndexes[i] = len(self.jointElementIndexes)-1
    def __str__(self):
        s = Skeleton.__str__(self)
        s += '<HIERARCHY>\n'
        s += self.root.__strHierarchy__()
        s += '<JOINT INDEX:(ELEMENT INDEX) ELEMENT NAME>\n'
        for i in range(len(self.jointElementIndexes)):
            s += '[%d]:(%d)\'%s\', '%(i, self.jointElementIndexes[i], self.elementNames[self.jointElementIndexes[i]])
        s += '\n'
        s += '<INTERNAL JOINT INDEX:(ELEMENT INDEX) ELEMENT NAME>\n'
        for i in range(1, len(self.jointElementIndexes)):
                s += '[%d]:(%d)\'%s\', '%(i-1, self.jointElementIndexes[i], self.elementNames[self.jointElementIndexes[i]])
        s += '\n'
        return s
    
    #===============================================================================
    # DOF functions
    #===============================================================================
    def getDOFs(self):
        return [6] + [3]*self.getInternalJointNum()
    def getTotalDOF(self):
        return 6 + 3*self.getInternalJointNum()

    #===============================================================================
    # joint index
    #===============================================================================
    def getJointNum(self):
        return len(self.jointElementIndexes)
    def getJoint(self, jointIndex):
        return self.elements[self.jointElementIndexes[jointIndex]]
    def getJointName(self, jointIndex):
        return self.elementNames[self.jointElementIndexes[jointIndex]]
    def getJointIndex(self, name):
        if name in self.reverseElementNames:
            index = self.reverseElementNames[name]            
            return self.reverseJointElementIndexes[index]
        else:
            return None
    
    def getInternalJointNum(self):
        return len(self.jointElementIndexes)-1
    def getInternalJointName(self, internalJointIndex):
        return self.elementNames[self.jointElementIndexes[internalJointIndex+1]]
    
    def getLinkNum(self):
        return self.getJointNum()
    
    def getParentJointIndex(self, jointIndex):
        index = self.jointElementIndexes[jointIndex]
        parentIndex = self.getParentIndex(index)
        if parentIndex!=None:
            return self.reverseJointElementIndexes[parentIndex]
        else:
            return None
    
    #===============================================================================
    # element index
    #===============================================================================
    def getParentIndex(self, index):
        joint = self.elements[index]
        if joint.parent!=None:
            return self.reverseElementNames[joint.parent.name]
        else:
            return None
    def getChildIndexes(self, index):
        ls = []
        joint = self.elements[index]
        for child in joint.children:
            ls.append(self.getElementIndex(child.name))
        return ls
    def getOffset(self, index):
        return self.elements[index].offset
    def removeElement(self, index):
#        print index
        joint = Skeleton.getElement(self, index)
        parent = joint.parent

        for child in joint.children:
            child.parent = parent
            child.offset += joint.offset
        
        if parent!=None:
#            print [child.name for child in parent.children]
#            print [Skeleton.getElementIndex(self, child.name) for child in parent.children]
            index_in_parent = [Skeleton.getElementIndex(self, child.name) for child in parent.children].index(index)
#            print index_in_parent
            del parent.children[index_in_parent:index_in_parent+1]
            parent.children.extend(joint.children)
#            print [child.name for child in parent.children]
            
        Skeleton.removeElement(self, index)
        self.initialize()
        
class Joint:
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent
        self.children = []
        self.offset = mm.O_Vec3()
    def addChild(self, name_or_joint, offset=None):
        if isinstance(name_or_joint, Joint):
            childJoint = name_or_joint
        else:
            name = name_or_joint
            childJoint = Joint(name, self)
        if offset!=None:
            childJoint.offset = offset
        self.children.append(childJoint)
        return childJoint
    def calcGlobalOffset(self):
        offset = mm.O_Vec3()
        current = self
        while current:
            offset += current.offset
            current = current.parent
        return offset 
    def __str__(self):
        string = ''
        string += 'name: '+self.name
        string += '  offset: '+self.offset.__str__() 
#        string += ', globalOffset: '+self.calcGlobalOffset().__str__()
        return string
    def __strHierarchy__(self, depth = 0):
        s = ''
        tab = '  '*depth
        s += '%s%s\n'%(tab, self.__str__())
        depth += 1
        for child in self.children:
            s += child.__strHierarchy__(depth)
        return s
class JointPosture(Posture):
    def __init__(self, skeleton):
        Posture.__init__(self, skeleton)
        self.rootPos = mm.O_Vec3()
        self.localRs = [None]*skeleton.getElementNum()
        self.globalTs = [None]*skeleton.getElementNum()

    # m1 + d : m1.__add__(d)
    # m2 = m1 + d
    # R_m2 = R_m1 (dot) R_d
    def __add__(self, displacement):
        if displacement.__class__ == JointDisplacement:
            m1 = self
            d = displacement
            m2 = JointPosture(self.skeleton)
            m2.rootPos = m1.rootPos + d.rootPos
            for i in range(len(m1.localRs)):
                R_m1 = m1.localRs[i]
                R_d = d.localRs[i]
                R_m2 = np.dot(R_m1, R_d) 
                m2.setLocalR(i, R_m2)
            m2.updateGlobalT()
            return m2
        else:
            raise TypeError
    def __sub__(self, other):
        # m2 - m1 : m2.__sub__(m1)
        # d = m2 - m1
        # R_d = inverse(R_m1) (dot) R_m2
        if other.__class__ == JointPosture:
            m2 = self
            m1 = other
            d = JointDisplacement(self.skeleton)
            d.rootPos = m2.rootPos - m1.rootPos
            for i in range(len(m2.localRs)):
                R_m2 = m2.localRs[i]
                R_m1 = m1.localRs[i]
                R_d = np.dot(R_m1.transpose(), R_m2) 
                d.setLocalR(i, R_d)
            return d
        # m2 - d : m2.__sub__(d)
        # m1 = m2 + (-d)
        # R_m1 = R_m2 (dot) inverse(R_d)
        elif other.__class__ == JointDisplacement:
            m2 = self 
            d = other
            m1 = m2 + (-d)
            return m1
        else:
            raise TypeError
    
#    def initialize(self):
#        self.rootPos = mm.O_Vec3()
#        self.initLocalRs()
#        self.initGlobalTs()
#    def initGlobalTs(self):
#        for i in range(self.skeleton.getElementNum()):
#            self.globalTs[i] = mm.I_SE3()
    def initLocalRs(self, initialRs=None):
        if initialRs==None:
            for i in range(self.skeleton.getElementNum()):
                self.localRs[i] = mm.I_SO3()
        else:
            self.localRs = initialRs
            
    def getTPose(self, initialRs=None):
        tpose = JointPosture(self.skeleton) # share skeleton of self
        tpose.rootPos = self.rootPos.copy()
        tpose.initLocalRs(initialRs)
        return tpose
    
    def copy(self):
        copied = JointPosture(self.skeleton)   # share skeleton of self
        copied.rootPos = self.rootPos.copy()
        for i in range(self.skeleton.getElementNum()):
            copied.localRs[i] = self.localRs[i].copy()
            copied.globalTs[i] = self.globalTs[i].copy()
        return copied
    
    def getLocalR(self, index):
        return self.localRs[index]
    def setLocalR(self, index, localR):
        self.localRs[index] = localR
    def mulLocalR(self, index, R):
        self.localRs[index] = np.dot(self.localRs[index], R)

    def getLocalRFromParent(self, index):
        # Gp * Rl = Gi
        # Rl = Li (= Gp.transpose() * Gi)
        return self.localRs[index]
    def getGlobalRFromParent(self, index):
        # Rg * Gp = Gi
        # Rg = Gi * Gp.transpose()
        parent = self.skeleton.getParentIndex(index)
        if parent==None:
            Gp = mm.I_SO3()
        else:
            Gp = self.getGlobalR(parent)
        Gi = self.getGlobalR(index)
        Rg = np.dot(Gi, Gp.transpose()) 
        return Rg

    def getGlobalR(self, index):
        return mm.T2R(self.globalTs[index])
    
    # Gp : global rotation of parent of joint i 
    # Gi : global rotation of joint i
    # Gin : new global rotation of joint i
    # Li : local rotation of joint i
    # Lin : new local rotation of joint i
    def setGlobalR(self, index, globalR):
        # Gi = Gp * Li
        # Gin = Gp * Lin
        # Lin = Gp.transpose() * Gin
        Gin = globalR
        parent = self.skeleton.getParentIndex(index)
        if parent==None:
            Gp = mm.I_SO3()
        else:
            Gp = self.getGlobalR(parent)
        Lin = np.dot(Gp.transpose(), Gin)
        self.setLocalR(index, Lin)
        self.updateGlobalT(index)
    def mulGlobalR(self, index, R):
        # Gi = Gp * Li
        # R * Gi = Gp * Lin
        # Lin = Gp.transpose() * R * Gi
        parent = self.skeleton.getParentIndex(index)
        if parent==None:
            Gp = mm.I_SO3()
        else:
            Gp = self.getGlobalR(parent)
        Gi = self.getGlobalR(index)
        Lin = np.dot(Gp.transpose(), np.dot(R, Gi))
        self.setLocalR(index, Lin)
        self.updateGlobalT(index)
        
#    def calcGlobalPosition(self, index):
#        return mm.T2p(self._calcGlobalT(self.skeleton.getElement(index)))
#    def calcGlobalT(self, index):
#        return self._calcGlobalT(self.skeleton.getElement(index))
#    def _calcGlobalT(self, joint):
#        if joint.parent:
#            T = self._calcGlobalT(joint.parent)
#        else:
#            T = mm.p2T(self.rootPos)
#        T = np.dot(T, mm.p2T(joint.offset))
#
#        index = self.skeleton.getElementIndex(joint.name)
#        T = np.dot(T, mm.R2T(self.localRs[index]))
#        return T
    
    def getGlobalT(self, index):
        return self.globalTs[index]
    def updateGlobalT(self, fromIndex=None):
        if fromIndex==None:
            self._updateGlobalT(self.skeleton.root, mm.p2T(self.rootPos))
        else:
            parent = self.skeleton.getParentIndex(fromIndex)
            if parent==None:
                self._updateGlobalT(self.skeleton.root, mm.p2T(self.rootPos))
            else:
                joint = self.skeleton.getElement(fromIndex)
                self._updateGlobalT(joint, self.globalTs[parent])
    def _updateGlobalT(self, joint, parentT):
        index = self.skeleton.getElementIndex(joint.name)
        T = np.dot(parentT, mm.p2T(joint.offset))
        T = np.dot(T, mm.R2T(self.localRs[index]))
        self.globalTs[index] = T
        for childJoint in joint.children:
            self._updateGlobalT(childJoint, T)
    
    # do not use!
    # have to replace to IK version
    def setPosition(self, index, position):
        prevPos = self.getPosition(index)
        offset = position - prevPos
        self.rootPos += offset
        self.updateGlobalT()
        
    def getPosition(self, index):
        return mm.T2p(self.globalTs[index])
    def getPositions(self):
        return [self.getPosition(i) for i in range(self.skeleton.getElementNum())]
    
    def blendPosture(self, otherPosture, t):
        p = JointPosture(self.skeleton)
        p.rootPos = mm.linearInterpol(self.rootPos, otherPosture.rootPos, t)
        for i in range(len(self.localRs)):
            p.localRs[i] = mm.slerp(self.localRs[i], otherPosture.localRs[i], t)
        p.updateGlobalT()
        return p
    
    #===========================================================================
    # joint index based new functions
    #===========================================================================
    def translateByOffset(self, p_offset):
        self.rootPos += p_offset
        self.updateGlobalT()
        
    def translateByTarget(self, p_target, targetJointIndex=0):
        p = self.getJointPositionGlobal(targetJointIndex)
        p_offset = p_target - p
        self.translateByOffset(p_offset)
        
    def rotateByOffset(self, R_offset):
        self.setJointOrientationGlobal(0, np.dot(R_offset, self.getJointOrientationGlobal(0)))
        self.updateGlobalT()
        
    def rotateByTarget(self, R_target, targetJointIndex=0):
        # R_offset * R = R_target
        # R_offset = R_target * R.T
        # R_root_new = R_offset * R_root
        R = self.getJointOrientationGlobal(targetJointIndex)
        R_offset = np.dot(R_target, R.T)
        self.rotateByOffset(R_offset)
        
    def mulJointOrientationGlobal(self, jointIndex, R_offset):
        self.mulGlobalR(self.skeleton.jointElementIndexes[jointIndex], R_offset)
    
    def getJointPositionGlobal(self, jointIndex):
        return self.getPosition(self.skeleton.jointElementIndexes[jointIndex])
    def getJointOrientationLocal(self, jointIndex):
        return self.localRs[self.skeleton.jointElementIndexes[jointIndex]]
    def getJointOrientationGlobal(self, jointIndex):
        return self.getGlobalR(self.skeleton.jointElementIndexes[jointIndex])
    
    def setJointPositionGlobal(self, jointIndex, position):
        raise NotImplementedError('use IK?')
    def setJointOrientationGlobal(self, jointIndex, R):
        self.setGlobalR(self.skeleton.jointElementIndexes[jointIndex], R)
    
    def getJointPositionsGlobal(self):
        return [self.getPosition(self.skeleton.jointElementIndexes[i]) for i in range(len(self.skeleton.jointElementIndexes))]
    def getJointOrientationsLocal(self):
        return [self.localRs[self.skeleton.jointElementIndexes[i]] for i in range(len(self.skeleton.jointElementIndexes))]
    def getJointOrientationsGlobal(self):
        return [self.getGlobalR(self.skeleton.jointElementIndexes[i]) for i in range(len(self.skeleton.jointElementIndexes))]
    
    def setJointOrientationsLocal(self, Rs):
        for i in range(len(self.skeleton.jointElementIndexes)):
            self.localRs[self.skeleton.jointElementIndexes[i]] = Rs[i]
    
    def getInternalJointPositionsGlobal(self):
        return [self.getPosition(self.skeleton.jointElementIndexes[i]) for i in range(1, len(self.skeleton.jointElementIndexes))]
    def getInternalJointOrientationsLocal(self):
        return [self.localRs[self.skeleton.jointElementIndexes[i]] for i in range(1, len(self.skeleton.jointElementIndexes))]
    def getInternalJointOrientationsGlobal(self):
        return [self.getGlobalR(self.skeleton.jointElementIndexes[i]) for i in range(1, len(self.skeleton.jointElementIndexes))]

    #########################
    ## Additional
    def addJoint(self, skeleton, localR) :
        self.skeleton = skeleton
        self.localRs.append(localR)
        self.globalTs.append(localR)
    def getDOFPositions(self):
        return [(self.rootPos, self.getLocalR(0))] + self.getInternalJointOrientationsLocal()
    def getDOFAxeses(self):
        return [np.concatenate((mm.I_SO3(), self.getJointOrientationGlobal(0).transpose()))] + [R.transpose() for R in self.getInternalJointOrientationsGlobal()]
    def setDOFPositions(self, DOFPositions):
        self.rootPos = DOFPositions[0][0]
        self.setJointOrientationsLocal([DOFPositions[0][1]]+DOFPositions[1:])
        
