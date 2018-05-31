from fltk import *
import numpy as np
import time
import copy

import sys
import os

sys.path.append('../PyCommon/modules')
sys.path.append('..')

# from PyCommon.modules.pydart import pydart
# from PyCommon.modules import pydart2 as pydart
import pydart2 as pydart

import math
import PyCommon.modules.Math.mmMath as mm
import PyCommon.modules.Renderer.ysRenderer as yr
from PyCommon.modules.Motion import ysMotion as ym
import PyCommon.modules.Renderer.csVpRenderer as cvr
import PyCommon.modules.Simulator.csVpWorld as cvw
import PyCommon.modules.Simulator.csVpModel as cvm
# import PyCommon.modules.Simulator.hpLCPSimulator as hls
import PyCommon.modules.Simulator.hpDartLCPSimulator as hls
import PyCommon.modules.GUI.hpSimpleViewer as hsv
import PyCommon.modules.Util.ysPythonEx as ype
import PyCommon.modules.ArticulatedBody.ysControl as yct
import PyCommon.modules.GUI.hpSplineEditor as hse
import PyCommon.modules.ArticulatedBody.hpInvKine as hik

# import VirtualPhysics.vpBody as vpB
# import VirtualPhysics.LieGroup as vpL

import mtInitialize_Simple as mit
from pdcontroller import PDController
from PyCommon.modules.Simulator import csDartModel as cpm
from PyCommon.modules.ArticulatedBody.pdcontroller import *


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

motion = None # type: ym.JointMotion
mcfg = None
wcfg = None
stepsPerFrame = None
config = None
mcfg_motion = None

vpWorld = None
controlModel = None
motionModel = None
IKModel = None
solver = None

totalDOF = None
DOFs = None

bodyIDsToCheck = None # type: list[int]

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

pydart.init()
data_dir = os.path.dirname(__file__)
print('data_dir = ' + data_dir)
'''
dartWorld = pydart.World(1.0/1800.0, data_dir+'/test.xml')
# dartWorld.test()
q = dartWorld.skeletons[1].q
q['j_Hips_pos_y'] = 0.
q['j_Hips_pos_x'] = 0.
q['j_Hips_pos_z'] = 0.
dartWorld.skeletons[1].set_positions(q)
# dartWorld.skeletons[1].dof
dartWorld.skeletons[1].controller = PDController(dartWorld.skeletons[1], dartWorld.dt)
# pydart.glutgui.run(title='bipedStand', simulation=dartWorld, trans=[0, 0, -3])
'''
dartModel = None # type: cpm.DartModel
pdController = None # type: PDController

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
    global motionModel
    global solver
    global IKModel

    global dartModel
    global pdController

    np.set_printoptions(precision=4, linewidth=200)
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_1()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_5()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_chiken_foot()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_foot('fastswim.bvh')
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_foot_2('simpleJump_2.bvh')
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_capsule('simpleJump_onebody.bvh')
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_foot('simpleJump.bvh')
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_foot('simpleJump_long.bvh')
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_foot('simpleJump_long.bvh')
    mcfg_motion = mit.normal_mcfg()

    vpWorld = cvw.VpWorld(wcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    IKModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)


    dartModel = cpm.DartModel(wcfg, motion[0], mcfg)
    pdController = PDController(dartModel.skeleton, dartModel.world.time_step())
    # dartWorld.skeletons[1].controller = PDController(dartWorld.skeletons[1], dartWorld.dt)

    # solver = hik.numIkSolver(wcfg, motion[0], mcfg)

    vpWorld.SetGlobalDamping(0.9999)
    # controlModel.initializeHybridDynamics()
    controlModel.initializeForwardDynamics()
    ModelOffset = np.array([0., .12, 0.])
    controlModel.translateByOffset(ModelOffset)
    motionModel.translateByOffset(ModelOffset)
    dartModel.translateByOffset(ModelOffset)

    vpWorld.SetIntegrator("RK4")
    # vpWorld.SetIntegrator("IMPLICIT_EULER_FAST")
    # vpWorld.SetIntegrator("EULER")

    controlModel.SetJointsDamping(0.2)
    controlModel.SetJointsElasticity(.01)

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
    # viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(
    #     motionModel, MOTION_COLOR, yr.POLYGON_FILL))
    # viewer.doc.addRenderer('IKModel', cvr.VpModelRenderer(
    #      solver.model, MOTION_COLOR, yr.POLYGON_FILL))

    # viewer.doc.addRenderer('dartModel', yr.DartModelRenderer(dartWorld, CHARACTER_COLOR2))
    viewer.doc.addRenderer('dartModel', yr.DartModelRenderer(dartModel, CHARACTER_COLOR2))

    viewer.doc.addRenderer('rd_contactForcesControl', yr.VectorsRenderer(rd_cForcesControl, rd_cPositionsControl, (255, 0, 0), .1, 'rd_c1'))
    viewer.doc.addRenderer('rd_contactForces', yr.VectorsRenderer(rd_cForces, rd_cPositions, (0, 255, 0), .1, 'rd_c2'))
    viewer.doc.addRenderer('rd_contactForceControl', yr.VectorsRenderer(rd_ForceControl, rd_Position, (0, 0, 255), .1, 'rd_c3'))
    # viewer.doc.addRenderer('rd_contactForceDes', yr.VectorsRenderer(rd_ForceDes, rd_PositionDes, (255, 0, 255), .1))
    # viewer.doc.addRenderer('rd_jointPos', yr.PointsRenderer(rd_jointPos))

    viewer.objectInfoWnd.add1DSlider('PD gain', minVal=0., maxVal=200., initVal=10., valStep=.1)
    viewer.objectInfoWnd.add1DSlider('Joint Damping', minVal=1., maxVal=2000., initVal=35., valStep=1.)
    viewer.objectInfoWnd.add1DSlider('steps per frame', minVal=1., maxVal=200., initVal=config['stepsPerFrame'], valStep=1.)
    viewer.objectInfoWnd.add1DSlider('1/simul speed', minVal=1., maxVal=100., initVal=config['simulSpeedInv'], valStep=1.)
    viewer.objectInfoWnd.add1DSlider('normal des force min', minVal=0., maxVal=1000., initVal=40., valStep=1.)
    viewer.objectInfoWnd.add1DSlider('normal des force max', minVal=0., maxVal=1000., initVal=40., valStep=1.)
    viewer.objectInfoWnd.add1DSlider('des force begin', minVal=0., maxVal=len(motion) - 1, initVal=50., valStep=1.)
    viewer.objectInfoWnd.add1DSlider('des force dur', minVal=1., maxVal=len(motion) - 1, initVal=5., valStep=1.)
    viewer.objectInfoWnd.add1DSlider('force weight', minVal=-10., maxVal=10., initVal=-1.3, valStep=.01)
    viewer.objectInfoWnd.add1DSlider('LCP weight', minVal=-10., maxVal=10., initVal=1.3, valStep=.01)
    viewer.objectInfoWnd.add1DSlider('tau weight', minVal=-10., maxVal=10., initVal=-3., valStep=.01)
    viewer.objectInfoWnd.add1DSlider('ref', minVal=-10., maxVal=10., initVal=0., valStep=.01)
    viewer.objectInfoWnd.addBtn('image', viewer.motionViewWnd.dump)
    viewer.objectInfoWnd.addBtn('image seq dump', viewer.motionViewWnd.dumpMov)

    viewer.cForceWnd.addDataSet('expForce', FL_BLACK)
    viewer.cForceWnd.addDataSet('desForceMin', FL_RED)
    viewer.cForceWnd.addDataSet('desForceMax', FL_RED)
    viewer.cForceWnd.addDataSet('realForce', FL_GREEN)

    for i in range(motion[0].skeleton.getJointNum()):
        print(i, motion[0].skeleton.getJointName(i))

    print("(index, id, name)")
    for i in range(controlModel.getBodyNum()):
        print(i, controlModel.index2vpid(i), controlModel.index2name(i))

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
        motionModel.update(motion[0])
        self.frame = frame
        print("main:frame : ", frame)
        # motionModel.update(motion[0])
        self.timeIndex = 0
        self.setTimeStamp()


        # skel = dartWorld.skels[1]
        # print(skel.q)
        # print(skel.dof('j_foot_1_0_x'))
        # skel.tau = skel.q

        # print(skel.body('root').J)
        # print(skel.body('root').world_com_angular_velocity())
        # print(skel.M)
        # print(skel.body('root').world_jacobian())
        # print(skel.body('Hips').world_jacobian())
        # print(skel.body('Hips').world_linear_jacobian())
        # print(skel.body('Hips').world_angular_jacobian())

        # for c in dartWorld.contacts():
        #     print(c.p)

        # print(skel.q)

        # IK solver
        '''
        solver.clear()
        solver.setInitPose(motion[0])
        cVpBodyIds, cPositions, cPositionsLocal, cVelocities = vpWorld.getContactPoints(bodyIDsToCheck)

        if len(cVpBodyIds) > 1:
            solver.addConstraints(cVpBodyIds[1], cPositionsLocal[1], np.array((0., 0., 0.)), None, (False, True, False, False))
            solver.addConstraints(cVpBodyIds[3], cPositionsLocal[3], np.array((0., 0., 0.)), None, (False, True, False, False))
            solver.addConstraints(cVpBodyIds[5], cPositionsLocal[5], np.array((0., 0., 0.)), None, (False, True, False, False))
        # solver.solve(controlModel, np.array((0., .15 + .05*math.sin(frame/10.), 0.)))
        '''


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
        # Dt = 2. * (Kt**.5)/20.
        # Dt = 0.
        # controlModel.SetJointsDamping(2*math.sqrt(damp))
        # controlModel.SetJointsElasticity(damp)
        # controlModel.SetJointsDamping(1.)

        wLCP = math.pow(2., getVal('LCP weight'))
        wForce = math.pow(2., getVal('force weight'))
        wTorque = math.pow(2., getVal('tau weight'))

        # tracking
        th_r = motion.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(frame)
        weightMapTuple = config['weightMapTuple']
        # weightMapTuple = None
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt, weightMapTuple)
        ype.flatten(ddth_des, ddth_des_flat)

        pdController.setTartgetPose(motion.getDOFPositions(0))


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
        # totalForce = np.array([-desNormalForce, 34.3, 0., 0., 0., 0.])
        # totalForce = np.array([0., 34.3, desNormalForce, 0., 0., 0.])
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

        if desForceFrame[0] <= frame <= desForceFrame[1]:
        # if False:
            # totalForceImpulse = stepsPerFrame * totalForce
            cBodyIDsControl, cPositionsControl, cPositionLocalsControl, cForcesControl, torques \
                = hls.calcLCPbasicControl(
                motion, dartModel.world, dartModel, bodyIDsToCheck, 1., totalForce, [wLCP, wTorque, wForce], ddth_des_flat)
            # if cForces is not None:
            #     print "control: ", sum(cForces)

        sumControlForce = np.array([0.]*6)
        if cForcesControl is not None:
            sumControlForce = np.hstack((sum(cForcesControl), np.array([0., 0., 0.])))

        timeStamp = None

        torque_None = False
        print("torques: ", torques)

        if not (desForceFrame[0] <= frame <= desForceFrame[1]) or (torques is None):
            torque_None = True
            torques = ddth_des_flat
        elif np.linalg.norm(sumControlForce - totalForce) > np.linalg.norm(totalForce):
            print("control failed!")
            torque_None = True
            torques = ddth_des_flat
        else:
            torques *= 1.

        # for i in range(int(stepsPerFrame)):
        #     if i % 5 == 0:
        #         cBodyIDs, cPositions, cPositionLocals, cForces, timeStamp \
        #             = hls.calcLCPForces(motion, vpWorld, controlModel, bodyIDsToCheck, 1., torques, solver='qp')
        #
        #     if i % 5 == 0 and len(cBodyIDs) > 0:
        #         # apply contact forces
        #         if False and not torque_None:
        #             vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForcesControl)
        #             simulContactForces += sum(cForcesControl)
        #         else:
        #             vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForces)
        #             simulContactForces += sum(cForces)
        #             # simulContactForces += sum(cForces)
        #
        #     ype.nested(torques, torques_nested)
        #     controlModel.setDOFTorques(torques_nested[1:])
        #     # vpWorld.step()

        for i in range(int(stepsPerFrame)):
            if i%5 ==0:
                cBodyIDs, cPositions, cPositionLocals, velocities = dartModel.getContactPoints(bodyIDsToCheck)
            if False and i % 5 == 0 and len(cBodyIDs):
                # apply contact forces
                if False and not torque_None:
                    vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForcesControl)
                    simulContactForces += sum(cForcesControl)
                else:
                    vpWorld.applyPenaltyForce(cBodyIDs, cPositionLocals, cForces)
                    simulContactForces += sum(cForces)
                    # simulContactForces += sum(cForces)

            # ype.nested(torques, torques_nested)
            # dartModel.setDOFTorques(torques_nested[1:])
            if torque_None:
                dartModel.skeleton.set_forces(pdController.compute())
            # elif i%5 == 0:
            else:
                dartModel.skeleton.set_forces(torques)
            dartModel.step()
            sumForce = sum([(-contact.force if contact.bodynode1.name == 'ground' else contact.force)
                            for contact in dartModel.world.collision_result.contacts])
            simulContactForces += sumForce


        if False:
            # debug contact force
            for contact in dartModel.world.collision_result.contacts:
                if contact.bodynode2.name == 'ground':
                    print('contact info: ', contact.bodynode1.name, contact)
                    print('contact info: ', contact.bodynode2.name, contact)
                else:
                    print('contact info: ', contact.bodynode1.name, contact)
                    print('contact info: ', contact.bodynode2.name, contact)

        contactPoints = [contact.point for contact in dartModel.world.collision_result.contacts]
        contactForces = [(-contact.force if contact.bodynode1.name == 'ground' else contact.force)
                         for contact in dartModel.world.collision_result.contacts]

        sumForce = sum(contactForces)


        self.setTimeStamp()

        # rendering expected force
        del rd_cForcesControl[:]
        del rd_cPositionsControl[:]
        if cBodyIDsControl is not None:
            # print cBodyIDsControl
            for i in range(len(cBodyIDsControl)):
                # print expected force
                rd_cForcesControl.append(cForcesControl[i].copy() /20.)
                rd_cPositionsControl.append(cPositionsControl[i].copy())

        # rendering sum of expected force
        del rd_ForceControl[:]
        del rd_Position[:]
        if cForcesControl is not None:
            # print expected force
            rd_ForceControl.append(sum(cForcesControl) /20.)
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
        for i in range(len(contactPoints)):
            # print calculated force
            # rd_cForces.append(cForces[i].copy() / 50.)
            rd_cForces.append(contactForces[i].copy() / 20.)
            # rd_cPositions.append(cPositions[i].copy())
            rd_cPositions.append(contactPoints[i].copy())

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
        viewer.cForceWnd.insertData('realForce', frame, simulContactForces[1]/stepsPerFrame)

        # graph desired force
        if desForceFrame[0] <= frame <= desForceFrame[1]:
            viewer.cForceWnd.insertData('desForceMin', frame, totalForce[1])
            # viewer.cForceWnd.insertData('desForceMin', frame, totalForce[1] * 1.0)
            # viewer.cForceWnd.insertData('desForceMax', frame, totalForce[1] * 1.1)
        else:
            viewer.cForceWnd.insertData('desForceMin', frame, 0.)
            viewer.cForceWnd.insertData('desForceMax', frame, 0.)

        viewer.cForceWnd.redraw()
        self.setTimeStamp()

callback = Callback()


viewer.setSimulateCallback(callback.simulateCallback)

viewer.startTimer(1 / 30.)
viewer.show()
# splineEditor = hse.SplineEditor()
# splineEditor.show()

Fl.run()
