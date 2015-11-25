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
    global viewer

    np.set_printoptions(precision=4, linewidth=200)
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_1()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_5()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped()
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_chiken_foot()
    mcfg_motion = mit.normal_mcfg()

    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)

    vpWorld.initialize()
    # vpWorld.SetIntegrator("RK4")
    vpWorld.SetIntegrator("IMPLICIT_EULER")
    # vpWorld.SetGlobalDamping(0.001)
    # controlModel.initializeHybridDynamics()
    controlModel.initializeForwardDynamics()

    totalDOF = controlModel.getTotalDOF()
    DOFs = controlModel.getDOFs()

    bodyIDsToCheck = range(vpWorld.getBodyNum())

    # flat data structure
    ddth_des_flat = ype.makeFlatList(totalDOF)
    dth_flat = ype.makeFlatList(totalDOF)
    ddth_sol = ype.makeNestedList(DOFs)

    rd_cForces = [None]
    rd_cPositions = [None]

    viewer = hsv.hpSimpleViewer()
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, yr.POLYGON_FILL))
    viewer.doc.addRenderer('rd_contactForces', yr.VectorsRenderer(rd_cForces, rd_cPositions, (0, 255, 0), .1))
    viewer.objectInfoWnd.add1DSlider('hehe', minVal=0., maxVal=10., initVal=1., valStep=.01)

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

    def simulateCallback(self, frame):

        global ddth_des_flat

        # reload(tf)
        self.frame = frame
        print "main:frame : ", frame
        # motionModel.update(motion[0])

        (Kt,) = viewer.objectInfoWnd.getVals()
        
        Dt = 2*(Kt**.5)

        # tracking
        th_r = motion.getDOFPositions(0)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(0)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(0)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt,
                                                  config['weightMap'].values())
        ddth_c = controlModel.getDOFAccelerations()
        ype.flatten(ddth_des, ddth_des_flat)

        for i in range(6):
            ddth_des_flat[i] = 0.

        for i in range(stepsPerFrame):
            # apply penalty force
            # bodyIDs, cPositions, cPositionLocals, cForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            cBodyIDs, cPositions, cPositionLocals, cForces \
                = hls.calcLCPForces(motion, vpWorld, controlModel, bodyIDsToCheck, 1., ddth_des_flat, 8)
            if len(cBodyIDs) > 0:
                vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForces)
                for idx in range(len(cForces)):
                    if cForces[idx][1] > 1000.:
                        print frame, cForces[idx]

            # controlModel.setDOFAccelerations(ddth_des)
            controlModel.setDOFTorques(ddth_des[1:])
            # controlModel.solveHybridDynamics()
            vpWorld.step()

        self.cBodyIDs, self.cPositions, self.cPositionLocals, self.cForces \
            = hls.calcLCPForces(motion, vpWorld, controlModel, bodyIDsToCheck, 1., ddth_des_flat, 8)

        del rd_cForces[:]
        del rd_cPositions[:]
        for i in range(len(self.cBodyIDs)):
            rd_cForces.append(self.cForces[i].copy()/200.)
            rd_cPositions.append(self.cPositions[i].copy())

callback = Callback()


viewer.setSimulateCallback(callback.simulateCallback)

viewer.startTimer(1/30.)
viewer.show()

