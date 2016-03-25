from fltk import *
import numpy as np
import time

import sys
sys.path.append('../PyCommon/modules')

import math
import Math.mmMath as mm
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Simulator.hpLCPSimulator as hls
import GUI.hpSimpleViewer as hsv
import Util.ysPythonEx as ype
import ArticulatedBody.ysControl as yct
import GUI.hpSplineEditor as hse

import VirtualPhysics.vpBody as vpB
import VirtualPhysics.LieGroup as vpL

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

    np.set_printoptions(precision=4, linewidth=200)
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_1()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_5()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_chiken_foot()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_foot('fastswim.bvh')
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_foot_2('simpleJump_2.bvh')
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_foot('simpleJump.bvh')
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_leg('kneeAndFoot.bvh')
    mcfg_motion = mit.normal_mcfg()

    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)

    vpWorld.SetIntegrator("RK4")
    # vpWorld.SetIntegrator("IMPLICIT_EULER_FAST")
    vpWorld.SetGlobalDamping(0.9999)
    # controlModel.initializeHybridDynamics()
    controlModel.initializeForwardDynamics()
    ModelOffset = np.array([0., .5, 0.])
    controlModel.translateByOffset(ModelOffset)

    vpWorld.initialize()

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

    viewer = hsv.hpSimpleViewer(title='main_Test')
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
        'PD gain', minVal=0., maxVal=200., initVal=10., valStep=.1)
    viewer.objectInfoWnd.add1DSlider(
        'Joint Damping', minVal=1., maxVal=2000., initVal=35., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'steps per frame', minVal=1., maxVal=200., initVal=config['stepsPerFrame'], valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        '1/simul speed', minVal=1., maxVal=100., initVal=config['simulSpeedInv'], valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'normal des force min', minVal=0., maxVal=1000., initVal=80., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'normal des force max', minVal=0., maxVal=1000., initVal=80., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'des force begin', minVal=0., maxVal=len(motion) - 1, initVal=70., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'des force dur', minVal=0., maxVal=len(motion) - 1, initVal=5., valStep=1.)
    viewer.objectInfoWnd.add1DSlider(
        'force weight', minVal=-10., maxVal=10., initVal=0., valStep=.01)
    viewer.objectInfoWnd.add1DSlider(
        'tracking weight', minVal=-10., maxVal=10., initVal=0., valStep=.01)
    viewer.objectInfoWnd.add1DSlider(
        'tau weight', minVal=-10., maxVal=10., initVal=0., valStep=.01)
    viewer.objectInfoWnd.addBtn('image', viewer.motionViewWnd.dump)

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

        # Dt = 2. * (Kt**.5)
        Dt = 0.
        # controlModel.SetJointsDamping(damp)
        controlModel.SetJointsDamping(1.)

        wForce = math.pow(2., getVal('force weight'))
        wTorque = math.pow(2., getVal('tau weight'))

        # tracking
        th_r = motion.getDOFPositions(0)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(0)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(0)
        weightMapTuple = config['weightMapTuple']
        weightMapTuple = None
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt, weightMapTuple)
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
        # totalForce = np.array([50., 150.])


        torques = None
        ddth_des_flat[0:6] = [0.] * 6
        self.setTimeStamp()
        simulContactForces = np.zeros(3)

        cBodyIDs = None
        cPositions = None
        cPositionLocals = None
        cForces = None

        cBodyIDsControl = None
        cPositionsControl = None
        cPositionLocalsControl = None
        cForcesControl = None

        # if desForceFrame[0] <= frame <= desForceFrame[1]:
        if True:
            # totalForceImpulse = stepsPerFrame * totalForce
            cBodyIDsControl, cPositionsControl, cPositionLocalsControl, cForcesControl, torques \
                = hls.calcLCPbasicControl(
                motion, vpWorld, controlModel, bodyIDsToCheck, .1, totalForce, wForce, wTorque, ddth_des_flat)
            # if cForces is not None:
            #     print "control: ", sum(cForces)

        timeStamp = None

        torque_None = False

        if not (desForceFrame[0] <= frame <= desForceFrame[1]) or torques is None:
            torque_None = True
            torques = ddth_des_flat

        for i in range(int(stepsPerFrame)):
            if i%10 == 0:
                cBodyIDs, cPositions, cPositionLocals, cForces, timeStamp \
                    = hls.calcLCPForces(motion, vpWorld, controlModel, bodyIDsToCheck, .1, torques, solver='qp')

            if i%10 == 0 and len(cBodyIDs) > 0:
                # apply contact forces
                if False and not torque_None:
                    vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForcesControl)
                    simulContactForces += sum(cForcesControl)
                else:
                    vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForces)
                    simulContactForces += sum(cForces)
                    # simulContactForces += sum(cForces)


            ype.nested(torques, torques_nested)
            controlModel.setDOFTorques(torques_nested[1:])
            vpWorld.step()

        self.setTimeStamp()

        # rendering expected force
        del rd_cForcesControl[:]
        del rd_cPositionsControl[:]
        if cBodyIDsControl is not None:
            # print cBodyIDsControl
            for i in range(len(cBodyIDsControl)):
                # print expected force
                rd_cForcesControl.append(cForcesControl[i].copy() /50.)
                rd_cPositionsControl.append(cPositionsControl[i].copy())

        # rendering sum of expected force
        del rd_ForceControl[:]
        del rd_Position[:]
        if cForcesControl is not None:
            # print expected force
            rd_ForceControl.append(sum(cForcesControl) /50.)
            rd_Position.append(np.array([0., 0., 0.1]))

        # graph expected force
        if cForcesControl is not None:
            sumForce = sum(cForcesControl)
            if sumForce[1] > 10000:
                sumForce[1] = 10000
            viewer.cForceWnd.insertData('expForce', frame, sumForce[1])
        else:
            viewer.cForceWnd.insertData('expForce', frame, 0.)


        # rendering calculated forces
        del rd_cForces[:]
        del rd_cPositions[:]
        for i in range(len(cBodyIDs)):
            # print calculated force
            rd_cForces.append(cForces[i].copy() / 50.)
            rd_cPositions.append(cPositions[i].copy())

        # rendering joint position
        del rd_jointPos[:]
        for i in range(motion[0].skeleton.getJointNum()):
            rd_jointPos.append(motion[frame].getJointPositionGlobal(i))

        # rendering desired force
        del rd_ForceDes[:]
        del rd_PositionDes[:]
        # rd_ForceDes.append(totalForce/50.)
        rd_ForceDes.append(totalForce[1] * np.array([0., 1., 0.]) / 50.)
        rd_PositionDes.append(np.array([0., 0., 0.]))
        # if self.cForces is not None:
        #     rd_ForceDes.append(sum(self.cForces)[1]/50. * [0., 1., 0.])
        #     rd_PositionDes.append(np.array([0., 0., -0.1]))

        # graph calculated force
        if cForces is not None:
            sumForce = sum(cForces)
            # viewer.cForceWnd.insertData('realForce', frame, sumForce[1])
            viewer.cForceWnd.insertData('realForce', frame, simulContactForces[1]/stepsPerFrame)
        else:
            viewer.cForceWnd.insertData('realForce', frame, 0.)
        # viewer.cForceWnd.insertData('realForce', frame, simulContactForces[1]/stepsPerFrame)

        # graph desired force
        if desForceFrame[0] <= frame <= desForceFrame[1]:
            viewer.cForceWnd.insertData('desForceMin', frame, totalForce[1])
            # viewer.cForceWnd.insertData('desForceMin', frame, totalForce[1] * 1.0)
            # viewer.cForceWnd.insertData('desForceMax', frame, totalForce[1] * 1.1)
        else:
            viewer.cForceWnd.insertData('desForceMin', frame, 0.)
            viewer.cForceWnd.insertData('desForceMax', frame, 0.)


        self.setTimeStamp()


callback = Callback()


viewer.setSimulateCallback(callback.simulateCallback)

viewer.startTimer(1 / 30.)
viewer.show()
# splineEditor = hse.SplineEditor()
# splineEditor.show()

Fl.run()
