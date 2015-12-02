# import psyco; psyco.full()
from fltk import *
import copy
import numpy as np
import numpy.linalg as npl
import time

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')
# if './modules' not in sys.path:
#    sys.path.append('./modules')

import Math.mmMath as mm
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Simulator.hpLCPSimulator as hls
import GUI.hpSimpleViewer as hsv
import Optimization.ysAnalyticConstrainedOpt as yac
import Util.ysPythonEx as ype
import ArticulatedBody.ysJacobian as yjc
import ArticulatedBody.ysReferencePoints as yrp
import ArticulatedBody.ysMomentum as ymt
import ArticulatedBody.ysControl as yct
import GUI.ysMultiViewer as ymv
import Motion.ysHierarchyEdit as yme
import Simulator.ysPhysConfig as ypc


import mtOptimize as mot
import mtInitialize_Simple as mit


MOTION_COLOR = (213, 111, 162)
CHARACTER_COLOR = (20, 166, 188)
FEATURE_COLOR = (255, 102, 0)
CHARACTER_COLOR2 = (200, 200, 200)


def getPartJacobian(_Jsys, _jIdx):
    # warning : only Jsys works.
    return _Jsys[6*_jIdx:6*_jIdx+6].copy()


def getBodyGlobalPos(_model, _motion, _name):
    return _model.getBodyPositionGlobal(_motion[0].skeleton.getJointIndex(_name))


def getBodyGlobalOri(_model, _motion, _name):
    return _model.getBodyOrientationGlobal(_motion[0].skeleton.getJointIndex(_name))

motion = None
mcfg = None
wcfg = None
stepsPerFrame = None
config = None
mcfg_motion = None

vpWorld = None
controlModel = None

totalDOF = None
DOFs = None

bodyIDsToCheck = None

ddth_des_flat = None
dth_flat = None
ddth_sol = None

rd_cForces = None
rd_cPositions = None

rd_cForcesControl = None
rd_cPositionsControl = None

rd_jointPos = None

viewer = None


def init():
    global motion
    global mcfg
    global wcfg
    global stepsPerFrame
    global config
    global mcfg_motion
    global vpWorld
    global controlModel
    global totalDOF
    global DOFs
    global bodyIDsToCheck
    global ddth_des_flat
    global dth_flat
    global ddth_sol
    global rd_cForces
    global rd_cPositions
    global rd_jointPos
    global rd_cForcesControl
    global rd_cPositionsControl
    global viewer

    np.set_printoptions(precision=4, linewidth=200)
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_1()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_5()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_chiken_foot()
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_foot('fastswim.bvh')
    mcfg_motion = mit.normal_mcfg()

    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)

    vpWorld.initialize()
    # vpWorld.SetIntegrator("RK4")
    vpWorld.SetIntegrator("IMPLICIT_EULER_FAST")
    # vpWorld.SetGlobalDamping(0.001)
    # controlModel.initializeHybridDynamics()
    controlModel.initializeForwardDynamics()
    ModelOffset = np.array([0., 1., 0.])
    controlModel.translateByOffset(ModelOffset)

    totalDOF = controlModel.getTotalDOF()
    DOFs = controlModel.getDOFs()

    bodyIDsToCheck = range(vpWorld.getBodyNum())

    # flat data structure
    ddth_des_flat = ype.makeFlatList(totalDOF)
    dth_flat = ype.makeFlatList(totalDOF)
    ddth_sol = ype.makeNestedList(DOFs)

    rd_cForces = [None]
    rd_cPositions = [None]
    rd_cForcesControl = [None]
    rd_cPositionsControl = [None]
    rd_jointPos = [None]

    viewer = hsv.hpSimpleViewer()
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, yr.POLYGON_FILL))
    viewer.doc.addRenderer('rd_contactForcesControl', yr.VectorsRenderer(rd_cForcesControl, rd_cPositionsControl, (0, 255, 0), .1))
    viewer.doc.addRenderer('rd_contactForces', yr.VectorsRenderer(rd_cForces, rd_cPositions, (255, 0, 0), .1))
    # viewer.doc.addRenderer('rd_jointPos', yr.PointsRenderer(rd_jointPos))

    viewer.objectInfoWnd.add1DSlider('PD gain', minVal=0., maxVal=1000., initVal=180., valStep=.1)
    viewer.objectInfoWnd.add1DSlider('Joint Damping', minVal=1., maxVal=200., initVal=35., valStep=1.)

    for i in range(motion[0].skeleton.getJointNum()):
        print(i, motion[0].skeleton.getJointName(i))

    print "(index, id, name)"
    for i in range(controlModel.getBodyNum()):
        print (i, controlModel.index2id(i), controlModel.index2name(i))

init()

# controlModel.fixBody(0)


class Callback:
    def __init__(self):
        self.cBodyIDs = None
        self.cPositions = None
        self.cPositionLocals = None
        self.cForces = None
        self.frame = -1
        self.timeStamp = []
        self.timeIndex = 0
        self.prevTime = 0.
        self.LCPTimeStamp = None

    def setTimeStamp(self):
        if self.timeIndex == 0:
            self.prevTime = time.time()
            self.timeIndex += 1
            return
        if len(self.timeStamp) < self.timeIndex:
            self.timeStamp.append(0.)
        curTime = time.time()
        self.timeStamp[self.timeIndex - 1] += curTime - self.prevTime
        self.prevTime = curTime
        self.timeIndex += 1

    def simulateCallback(self, frame):
        global ddth_des_flat

        # reload(tf)
        self.frame = frame
        print "main:frame : ", frame
        # motionModel.update(motion[0])
        self.timeIndex = 0
        self.setTimeStamp()

        # constant setting
        (Kt, damp,) = viewer.objectInfoWnd.getVals()
        Dt = 2.*(Kt**.5)
        controlModel.SetJointsDamping(damp)

        # tracking
        th_r = motion.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt) # config['weightMapTuple'])
        ddth_c = controlModel.getDOFAccelerations()
        ype.flatten(ddth_des, ddth_des_flat)

        totalForce = np.array([0., 1000., 0.])

        torques = None
        ddth_des_flat[0:6] = [0.]*6
        self.setTimeStamp()
        for i in range(stepsPerFrame):
            # apply penalty force
            # cBodyIDs, cPositions, cPositionLocals, cForces, torque \
            #     = hls.calcLCPControl(motion, vpWorld, controlModel, bodyIDsToCheck, 1., totalForce, ddth_des_flat, 8)
            cBodyIDs, cPositions, cPositionLocals, cForces, timeStamp \
                = hls.calcLCPForces(motion, vpWorld, controlModel, bodyIDsToCheck, 1., ddth_des_flat, 8)

            if timeStamp is not None:
                if self.LCPTimeStamp is not None:
                    self.LCPTimeStamp += np.array(timeStamp)
                else:
                    self.LCPTimeStamp = np.array(timeStamp).copy()

            if len(cBodyIDs) > 0:
                vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForces)
                for idx in range(len(cForces)):
                    if cForces[idx][1] > 1000.:
                        print frame, cForces[idx]

            if torques is not None:
                controlModel.setDOFTorques(torques[1:])
            else:
                controlModel.setDOFTorques(ddth_des[1:])
            vpWorld.step()

        self.setTimeStamp()
        # print ddth_des_flat
        # print torques

        totalForce = np.array([0., 100., 0.])

        # self.cBodyIDs, self.cPositions, self.cPositionLocals, self.cForces, torques \
        #     = hls.calcLCPControl(motion, vpWorld, controlModel, bodyIDsToCheck, 1., totalForce, ddth_des_flat, 8)
        # del rd_cForcesControl[:]
        # del rd_cPositionsControl[:]
        # for i in range(len(self.cBodyIDs)):
        #     rd_cForcesControl.append(self.cForces[i].copy()/200.)
        #     rd_cPositionsControl.append(self.cPositions[i].copy())
        # # if torques is not None:
        # #     print 'torques: ', torques[:6]

        tmptmp = None

        self.cBodyIDs, self.cPositions, self.cPositionLocals, self.cForces, tmptmp \
            = hls.calcLCPForces(motion, vpWorld, controlModel, bodyIDsToCheck, 1., ddth_des_flat, 8)
        # print 'self.cPositionLocals: ', self.cPositionLocals
        del rd_cForces[:]
        del rd_cPositions[:]
        for i in range(len(self.cBodyIDs)):
            rd_cForces.append(self.cForces[i].copy()/10.)
            rd_cPositions.append(self.cPositions[i].copy())
        if self.cForces is not None:
            print "length: ", mm.length(sum(self.cForces))
        # if torques is not None:
        #     print np.array(torques) - np.array(ddth_des_flat)

        del rd_jointPos[:]
        for i in range(motion[0].skeleton.getJointNum()):
            rd_jointPos.append(motion[frame].getJointPositionGlobal(i))

        self.setTimeStamp()
        # print self.timeStamp
        # print self.LCPTimeStamp


callback = Callback()


viewer.setSimulateCallback(callback.simulateCallback)

viewer.startTimer(1/30.)
viewer.show()

Fl.run()

