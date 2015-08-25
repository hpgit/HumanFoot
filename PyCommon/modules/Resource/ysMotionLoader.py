import numpy, math

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mmMath
import Motion.ysMotion as ym

#===============================================================================
# .bvh file
#===============================================================================
def readBvhFile(bvhFilePath, scale=1.0, applyRootOffset=False):
    bvh = Bvh()
    bvh.parseBvhFile(bvhFilePath)
    jointMotion = bvh.toJointMotion(scale, applyRootOffset)
    return jointMotion

def readBvhFileAsBvh(bvhFilePath):
    bvh = Bvh()
    bvh.parseBvhFile(bvhFilePath)
    return bvh

#def writeBvhFile(bvhFilePath, jointMotion):
#    bvh = Bvh()
#    bvh.fromJointMotion(jointMotion)
#    bvh.writeBvhFile(bvhFilePath)

class Bvh:
    class Joint:
        def __init__(self, name):
            self.name = name
            self.offset = None
            self.channels = []
            self.children = []
            self.jointIndex = None
        def __strHierarchy__(self, depth=0):
            s = ''
            tab1 = '  '*depth
            tab2 = '  '*(depth+1)
            s += '%sJOINT %s\n'%(tab1, self.name)
            s += '%s{\n'%tab1
            s += '%sOFFSET %s\n'%(tab2, self.offset)
            
            channelString = ''
            for channel in self.channels:
                channelString += channel.__str__()+' '
            s += '%sCHANNELS %s\n'%(tab2, channelString)
            
            for child in self.children:
                s += child.__strHierarchy__(depth+1)
            s += '%s}\n'%tab1
            return s

    class Channel:
        def __init__(self, channelType, channelIndex):
            self.channelType = channelType
            self.channelIndex = channelIndex
        def __str__(self):
            return self.channelType
        
    def __init__(self):
        self.joints = []
        self.frameNum = 0
        self.frameTime = 0
        self.motionList = []
        
        self.totalChannelCount = 0

    def __str__(self):
        s = 'HIERARCHY\n'
        s += self.joints[0].__strHierarchy__()
        s += 'MOTION\n'
        s += 'Frame: %d\n'%self.frameNum
        s += 'Frame Time: %f\n'%self.frameTime
#        for i in range(len(self.motionList)):
#            s += self.motionList[i].__str__() + '\n'
        return s

    #===========================================================================
    # read functions
    #===========================================================================
    def parseBvhFile(self, filepath_or_fileobject):
        if isinstance(filepath_or_fileobject, str):
            f = open(filepath_or_fileobject)
        else:
            f = filepath_or_fileobject
    
        tokens= f.read().split()
        tokens.reverse()
    
        self.totalChannelCount = 0
        self.parseBvhHierachy(tokens)
        self.parseBvhMotion(tokens)

    def parseBvhHierachy(self, tokens):
        if tokens.pop().upper() != "HIERARCHY":
            print "HIERARCHY missing"
            return 
        if tokens.pop().upper() != "ROOT":
            print "ROOT missing"
            return
        self.parseBvhJoint(tokens.pop(), tokens)
        
    def parseBvhJoint(self, name, tokens):
        bvhJoint = Bvh.Joint(name)
        self.joints.append(bvhJoint)
        
        if tokens.pop()!="{":
            print "'{' missing"
            return None
        
        endDetected = False
        while not endDetected:
            t = tokens.pop().upper()
            if t == '}':
                endDetected = True
            elif t == 'OFFSET':
                x = float(tokens.pop())
                y = float(tokens.pop())
                z = float(tokens.pop())
                bvhJoint.offset = numpy.array([x,y,z],float)
            elif t == 'CHANNELS':
                channelCount = int(tokens.pop())
                for i in range(channelCount):
                    channelType = tokens.pop().upper()
                    bvhJoint.channels.append(Bvh.Channel(channelType, self.totalChannelCount))
                    self.totalChannelCount += 1
            elif t == 'JOINT':
                bvhJoint.children.append(self.parseBvhJoint(tokens.pop(), tokens))
            elif t == 'END':
                next = tokens.pop().upper()
                if next != 'SITE':
                    print 'END', next, 'is unknown keyword'
                bvhJoint.children.append(self.parseBvhJoint("%s_Effector"%name, tokens))
            else:
                print "invalid bvhJoint definition"
                return None
        return bvhJoint

    def parseBvhMotion(self, tokens):
        if tokens.pop().upper() != 'MOTION':
            print "MOTION missing"
            return None
        if tokens.pop().upper() != 'FRAMES:':
            print "FRAMES: missing"
            return None
        self.frameNum = int(tokens.pop())
        if tokens.pop().upper() != 'FRAME TIME:':
            if tokens.pop().upper() != 'TIME:':
                print "FRAME TIME: missing"
                return None
        self.frameTime = float(tokens.pop())
        
        self.motionList = [None]*self.frameNum
        for i in range(self.frameNum):
            self.motionList[i] = [None]*self.totalChannelCount
            
        for i in range(self.frameNum):
            for j in range(self.totalChannelCount):
                self.motionList[i][j] = float(tokens.pop())

    #===========================================================================
    # write functions
    #===========================================================================
#    def writeBvhFile(self, filepath_or_fileobject):
#        if isinstance(filepath_or_fileobject, str):
#            f = open(filepath_or_fileobject)
#        else:
#            f = filepath_or_fileobject
#        print self

    #===========================================================================
    # conversion functions
    #===========================================================================
    def toJointMotion(self, scale, applyRootOffset):
        skeleton = self.toJointSkeleton(scale, applyRootOffset)
        
        jointMotion = ym.JointMotion()
        for i in range(len(self.motionList)):
            jointPosture = ym.JointPosture(skeleton)
            self.addJointSO3FromBvhJoint(jointPosture, self.joints[0], self.motionList[i], scale)
            jointPosture.updateGlobalT()
            jointMotion.append(jointPosture)
        
        jointMotion.fps = 1./self.frameTime
        return jointMotion

    def toJointSkeleton(self, scale, applyRootOffset):
        jointMap = {}
        
        root = self.addJointFromBvhJoint(jointMap, self.joints[0].name, self.joints[0], None, scale, applyRootOffset)
        
        skeleton = ym.JointSkeleton(root)
        for bvhJoint in self.joints:
            skeleton.addElement(jointMap[bvhJoint.name], bvhJoint.name)
        skeleton.rootIndex = skeleton.getElementIndex(root.name)
        skeleton.initialize()
        
        return skeleton
    
    def addJointFromBvhJoint(self, jointMap, jointName, bvhJoint, parentJoint, scale, applyOffset):
        joint = ym.Joint(jointName, parentJoint)
        if applyOffset:
            joint.offset = bvhJoint.offset*scale
        jointMap[jointName] = joint
        
        for i in range(len(bvhJoint.children)):
            child = self.addJointFromBvhJoint(jointMap, bvhJoint.children[i].name, bvhJoint.children[i], joint, scale, True)
            joint.children.append(child)
            
        return joint
        
    def addJointSO3FromBvhJoint(self,jointPosture, bvhJoint, channelValues, scale = 1.0):
        len_children = len(bvhJoint.children)
        
        localR = mmMath.I_SO3()
        for channel in bvhJoint.channels:
            if channel.channelType == 'XPOSITION':
                jointPosture.rootPos[0] = channelValues[channel.channelIndex]*scale
            elif channel.channelType == 'YPOSITION':
                jointPosture.rootPos[1] = channelValues[channel.channelIndex]*scale
            elif channel.channelType == 'ZPOSITION':
                jointPosture.rootPos[2] = channelValues[channel.channelIndex]*scale
            elif channel.channelType == 'XROTATION':
                localR = numpy.dot(localR, mmMath.exp(mmMath.s2v((1,0,0)), mmMath.deg2Rad(channelValues[channel.channelIndex])))
            elif channel.channelType == 'YROTATION':
                localR = numpy.dot(localR, mmMath.exp(mmMath.s2v((0,1,0)), mmMath.deg2Rad(channelValues[channel.channelIndex])))
            elif channel.channelType == 'ZROTATION':
                localR = numpy.dot(localR, mmMath.exp(mmMath.s2v((0,0,1)), mmMath.deg2Rad(channelValues[channel.channelIndex])))
    #    jointPosture.setLocalR(bvhJoint.name, localR)
        jointPosture.setLocalR(jointPosture.skeleton.getElementIndex(bvhJoint.name), localR)
            
        for i in range(len_children):
            self.addJointSO3FromBvhJoint(jointPosture, bvhJoint.children[i], channelValues)
        
    
