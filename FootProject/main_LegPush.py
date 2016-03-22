from fltk import *
import numpy as np
import time

import sys
sys.path.append("../PyCommon/modules")

import math
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Math.mmMath as mm

import Simulator.hpLCPSimulator as hls
import GUI.hpSimpleViewer as hsv
import Util.ysPythonEx as ype
import ArticulatedBody.ysControl as yct

import mtInitialize_Simple as mit

MOTION_COLOR = (213, 111, 162)
CHARACTER_COLOR = (20, 166, 188)
FEATURE_COLOR = (255, 102, 0)
CHARACTER_COLOR2 = (200, 200, 200)


def getPartJacobian(_Jsys, _jIdx):
    # warning : only Jsys works.
    return _Jsys[6 * _jIdx:6 * _jIdx + 6].copy()


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

torques_nested = None

ddth_des_flat = None
dth_flat = None
ddth_sol = None

rd_cForces = None
rd_cPositions = None

rd_cForcesControl = None
rd_cPositionsControl = None

rd_ForceControl = None
rd_ForceDes = None
rd_Position = None
rd_PositionDes = None

rd_jointPos = None

viewer = None


pushTime = 0.

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
    global torques_nested
    global ddth_des_flat
    global dth_flat
    global ddth_sol
    global rd_cForces
    global rd_cPositions
    global rd_jointPos
    global rd_cForcesControl
    global rd_cPositionsControl
    global rd_ForceControl
    global rd_ForceDes
    global rd_Position
    global rd_PositionDes
    global viewer
    global pushTime
    global Rs
    global ps

    np.set_printoptions(precision=4, linewidth=200)
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_legs('legs_robust.bvh')
    motion[:] = [motion[0]]*1000
    mcfg_motion = mit.normal_mcfg()

    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)

    vpWorld.initialize()
    # vpWorld.SetIntegrator("RK4")
    vpWorld.SetIntegrator("IMPLICIT_EULER_FAST")
    # vpWorld.SetGlobalDamping(0.001)
    # controlModel.initializeHybridDynamics()
    controlModel.initializeForwardDynamics()
    ModelOffset = np.array([0., 2.5, 0.])
    controlModel.translateByOffset(ModelOffset)

    totalDOF = controlModel.getTotalDOF()
    DOFs = controlModel.getDOFs()

    bodyIDsToCheck = range(vpWorld.getBodyNum())

    # flat data structure
    ddth_des_flat = ype.makeFlatList(totalDOF)
    dth_flat = ype.makeFlatList(totalDOF)
    ddth_sol = ype.makeNestedList(DOFs)
    torques_nested = ype.makeNestedList(DOFs)

    rd_cForces = [None]
    rd_cPositions = [None]
    rd_cForcesControl = [None]
    rd_cPositionsControl = [None]
    rd_ForceControl = [None]
    rd_ForceDes = [None]
    rd_Position = [None]
    rd_PositionDes = [None]
    rd_jointPos = [None]

    viewer = hsv.hpSimpleViewer(title='main_LegPush')
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(
        controlModel, CHARACTER_COLOR, yr.POLYGON_FILL))
    viewer.doc.addRenderer('rd_contactForcesControl', yr.VectorsRenderer(
        rd_cForcesControl, rd_cPositionsControl, (255, 0, 0), .1))
    viewer.doc.addRenderer('rd_contactForces', yr.VectorsRenderer(
        rd_cForces, rd_cPositions, (0, 255, 0), .1))
    viewer.doc.addRenderer('rd_contactForceControl', yr.VectorsRenderer(
        rd_ForceControl, rd_Position, (0, 0, 255), .1))
    viewer.doc.addRenderer('rd_contactForceDes', yr.VectorsRenderer(
        rd_ForceDes, rd_PositionDes, (255, 0, 255), .1))
    # viewer.doc.addRenderer('rd_jointPos', yr.PointsRenderer(rd_jointPos))

    viewer.objectInfoWnd.add1DSlider(
        'PD gain', minVal=0., maxVal=1000., initVal=180., valStep=.1)
    viewer.objectInfoWnd.add1DSlider(
        'Joint Damping', minVal=1., maxVal=2000., initVal=35., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'steps per frame', minVal=1., maxVal=200., initVal=config['stepsPerFrame'], valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        '1/simul speed', minVal=1., maxVal=100., initVal=config['simulSpeedInv'], valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'normal des force min', minVal=0., maxVal=3000., initVal=80., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'normal des force max', minVal=0., maxVal=3000., initVal=80., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'des force begin', minVal=0., maxVal=len(motion) - 1, initVal=70., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'des force dur', minVal=0., maxVal=len(motion) - 1, initVal=20., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'force weight', minVal=-10., maxVal=10., initVal=0., valStep=.01)
    viewer.objectInfoWnd.add1DSlider(
        'tracking weight', minVal=-10., maxVal=10., initVal=0., valStep=.01)
    viewer.objectInfoWnd.add1DSlider(
        'tau weight', minVal=-10., maxVal=10., initVal=0., valStep=.01)
    viewer.objectInfoWnd.add1DSlider(
        'push x', minVal=-500., maxVal=500., initVal=0., valStep=10.)
    viewer.objectInfoWnd.add1DSlider(
        'push y', minVal=-500., maxVal=500., initVal=0., valStep=10.)
    viewer.objectInfoWnd.add1DSlider(
        'push z', minVal=-500., maxVal=500., initVal=0., valStep=10.)
    viewer.objectInfoWnd.add1DSlider(
        'push time', minVal=0., maxVal=10., initVal=0.1, valStep=.01)

    def pushCallback(ptr):
        global pushTime
        pushTime += viewer.objectInfoWnd.getVal('push time')

    viewer.objectInfoWnd.addBtn('Push', pushCallback)

    viewer.cForceWnd.addDataSet('expForce', FL_BLACK)
    viewer.cForceWnd.addDataSet('desForceMin', FL_RED)
    viewer.cForceWnd.addDataSet('desForceMax', FL_RED)
    viewer.cForceWnd.addDataSet('realForce', FL_GREEN)

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
        global stepsPerFrame
        global wcfg
        global vpWorld

        # reload(tf)
        self.frame = frame
        print "main:frame : ", frame
        # motionModel.update(motion[0])
        self.timeIndex = 0
        self.setTimeStamp()

        # constant setting
        # (Kt, damp, stepsPerFrame, simulSpeedInv) = viewer.objectInfoWnd.getVals()
        getVal = viewer.objectInfoWnd.getVal
        Kt = getVal('PD gain')
        damp = getVal('Joint Damping')
        stepsPerFrame = getVal('steps per frame')
        simulSpeedInv = getVal('1/simul speed')
        wcfg.timeStep = 1 / (30. * simulSpeedInv * stepsPerFrame)
        vpWorld.SetTimeStep(wcfg.timeStep)

        Dt = 2. * (Kt**.5)
        controlModel.SetJointsDamping(damp)

        wForce = math.pow(2., getVal('force weight'))
        wTorque = math.pow(2., getVal('tau weight'))

        # tracking
        th_r = motion.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(frame)
        # config['weightMapTuple'])
        ddth_des = yct.getDesiredDOFAccelerations(
            th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        ddth_c = controlModel.getDOFAccelerations()
        ype.flatten(ddth_des, ddth_des_flat)

        desForceFrameBegin = getVal('des force begin')
        desForceDuration = getVal('des force dur') * simulSpeedInv
        desForceFrame = [
            desForceFrameBegin, desForceFrameBegin + desForceDuration]

        desForceRelFrame = float(frame - desForceFrame[0]) / desForceDuration
        desNormalForceMin = getVal('normal des force min')
        desNormalForceMax = getVal('normal des force max')
        desNormalForce = desNormalForceMin
        if desForceFrame[0] <= frame <= desForceFrame[1]:
            desNormalForce = desNormalForceMin * \
                (1 - desForceRelFrame) + desNormalForceMax * desForceRelFrame

        totalForce = np.array([0., desNormalForce, 0., 0., 0., 0.])
        # totalForce = np.array([0., 720., 0., 0., 0., 0.])
        # totalForce = np.array([50., 150.])

        torques = None
        ddth_des_flat[0:6] = [0.] * 6
        self.setTimeStamp()
        simulContactForces = np.zeros(3)

        if desForceFrame[0] <= frame <= desForceFrame[1]:
            if True:
                # totalForceImpulse = stepsPerFrame * totalForce
                cBodyIDs, cPositions, cPositionLocals, cForcesControl, torques \
                    = hls.calcLCPbasicControl(
                    motion, vpWorld, controlModel, bodyIDsToCheck, 1., totalForce, wForce, wTorque, ddth_des_flat)
                # if cForces is not None:
                #     print "control: ", sum(cForces)

        cBodyIDs = None
        cPositions = None
        cPositionLocals = None
        cForces = None
        timeStamp = None

        torque_None = False
        if torques == None:
            torque_None = True
            torques = ddth_des_flat

        for i in range(int(stepsPerFrame)):
            cBodyIdsReal = []
            cPositionLocalsReal = []
            cForcesReal = []
            if i%20 == 0:
                cBodyIDs, cPositions, cPositionLocals, cForces, timeStamp \
                    = hls.calcLCPForces(motion, vpWorld, controlModel, bodyIDsToCheck, 1., torques, solver='qp')
            cVpBodyIds, cVpPositions, cVpPositionsLocal, cVpVelocities = vpWorld.getContactPoints(bodyIDsToCheck)

            for jj in range(len(cBodyIDs)):
                for kk in range(len(cVpBodyIds)):
                    if cBodyIDs[jj] == cVpBodyIds[kk]:
                        if np.linalg.norm(cPositionLocals[jj]-cVpPositionsLocal[kk]) < mm.LIE_EPS:
                            cBodyIdsReal.append(cBodyIDs[jj])
                            cPositionLocalsReal.append(cPositionLocals[jj])
                            cForcesReal.append(cForces[jj])

            if False and len(cBodyIDs) > 0:
                # apply contact forces
                if False and not torque_None:
                    vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForcesControl)
                    simulContactForces += sum(cForcesControl)
                else:
                    vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForces)
                    simulContactForces += sum(cForces)
                    # simulContactForces += sum(cForces)
            if len(cBodyIdsReal) > 0:
                vpWorld.applyPenaltyForce(cBodyIdsReal, cPositionLocalsReal, cForcesReal)
                simulContactForces += sum(cForcesReal)

            if pushTime > 0.:
                vpWorld.applyPenaltyForce([0], [[0., 0., 0.]], [[getVal('push x'), getVal('push y'), getVal('push z')]])
            ype.nested(torques, torques_nested)
            controlModel.setDOFTorques(torques_nested[1:])
            vpWorld.step()

        self.setTimeStamp()
        # print ddth_des_flat
        # print torques
        print simulContactForces/stepsPerFrame

        self.cBodyIDs, self.cPositions, self.cPositionLocals, self.cForces, torques \
            = hls.calcLCPbasicControl(motion, vpWorld, controlModel, bodyIDsToCheck, 1., totalForce, wForce, wTorque, ddth_des_flat)
        del rd_cForcesControl[:]
        del rd_cPositionsControl[:]
        for i in range(len(self.cBodyIDs)):
            # print expected force
            rd_cForcesControl.append(self.cForces[i].copy() / 50.)
            rd_cPositionsControl.append(self.cPositions[i].copy())
        del rd_ForceControl[:]
        del rd_Position[:]
        if self.cForces is not None:
            # print expected force
            rd_ForceControl.append(sum(self.cForces) / 50.)
            rd_Position.append(np.array([0., 0., 0.1]))
        # graph
        if self.cForces is not None:
            sumForce = sum(self.cForces)
            viewer.cForceWnd.insertData('expForce', frame, sumForce[1])
        else:
            viewer.cForceWnd.insertData('expForce', frame, 0.)

        self.cBodyIDs, self.cPositions, self.cPositionLocals, self.cForces, tmptmp \
            = hls.calcLCPForces(motion, vpWorld, controlModel, bodyIDsToCheck, 1., torques, solver='qp')
        del rd_cForces[:]
        del rd_cPositions[:]
        for i in range(len(self.cBodyIDs)):
            # print calculated force
            rd_cForces.append(self.cForces[i].copy() / 50.)
            rd_cPositions.append(self.cPositions[i].copy())

        del rd_jointPos[:]
        for i in range(motion[0].skeleton.getJointNum()):
            rd_jointPos.append(motion[frame].getJointPositionGlobal(i))

        del rd_ForceDes[:]
        del rd_PositionDes[:]
        # rd_ForceDes.append(totalForce/50.)
        rd_ForceDes.append(totalForce[1] * np.array([0., 1., 0.]) / 50.)
        rd_PositionDes.append(np.array([0., 0., 0.]))
        # if self.cForces is not None:
        #     rd_ForceDes.append(sum(self.cForces)[1]/50. * [0., 1., 0.])
        #     rd_PositionDes.append(np.array([0., 0., -0.1]))
        # graph
        if self.cForces is not None:
            sumForce = sum(self.cForces)
            # viewer.cForceWnd.insertData('realForce', frame, sumForce[1])
            viewer.cForceWnd.insertData('realForce', frame, simulContactForces[1]/stepsPerFrame)
        else:
            viewer.cForceWnd.insertData('realForce', frame, 0.)
        viewer.cForceWnd.insertData('realForce', frame, simulContactForces[1]/stepsPerFrame)
        if desForceFrame[0] <= frame <= desForceFrame[1]:
            viewer.cForceWnd.insertData('desForceMin', frame, totalForce[1] * 1.)
            # viewer.cForceWnd.insertData('desForceMin', frame, totalForce[1] * .9)
            # viewer.cForceWnd.insertData('desForceMax', frame, totalForce[1] * 1.1)
        else:
            viewer.cForceWnd.insertData('desForceMin', frame, 0.)
            viewer.cForceWnd.insertData('desForceMax', frame, 0.)


        self.setTimeStamp()
        # print self.timeStamp
        # print self.LCPTimeStamp


callback = Callback()


viewer.setSimulateCallback(callback.simulateCallback)

viewer.startTimer(1 / 30.)
viewer.show()

Fl.run()
