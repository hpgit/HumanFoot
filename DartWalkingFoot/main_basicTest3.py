from fltk import *
import numpy as np
import time
import copy

import math
import PyCommon.modules.Math.mmMath as mm
import PyCommon.modules.Renderer.ysRenderer as yr
# import PyCommon.modules.Renderer.csVpRenderer as cvr
# import PyCommon.modules.Simulator.csVpWorld as cvw
# import PyCommon.modules.Simulator.csVpModel as cvm
# import PyCommon.modules.Simulator.hpLCPSimulator as hls
import PyCommon.modules.GUI.hpSimpleViewer as hsv
import PyCommon.modules.Util.ysPythonEx as ype
import PyCommon.modules.ArticulatedBody.ysControl as yct
import PyCommon.modules.GUI.hpSplineEditor as hse
import PyCommon.modules.ArticulatedBody.hpInvKine as hik

# import VirtualPhysics.vpBody as vpB
# import VirtualPhysics.LieGroup as vpL

# import mtInitialize_Simple as mit

import PyCommon.modules.Resource.ysMotionLoader as yf
import PyCommon.modules.Simulator.ysPhysConfig as ypc

# import PyCommon.modules.Simulator.csVpWorld_py as cvw
# import PyCommon.modules.Simulator.csVpModel_py as cvm
# import PyCommon.modules.Simulator.hpLCPSimul2 as hls
# import PyCommon.modules.pyVirtualPhysics as pyv

import PyCommon.modules.Simulator.csDartModel as cdm
from PyCommon.modules.dart.bvh2dartSkel import *
import PyCommon.modules.pydart2 as pydart


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


def basicTest3():
    def create_foot(motionFile='foot3.bvh'):
        massMap = {}
        massMap = massMap.fromkeys(['Hips', 'foot00', 'foot01'], 0.)
        massMap['Hips'] = 1.
        massMap['foot00'] = 1.
        massMap['foot01'] = 1.

        # motion
        motion = yf.readBvhFile(motionFile, .05)
        motion.extend([motion[-1]]*3000)

        # world, model
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = 1.
        # for i in range(motion[0].skeleton.getElementNum()):
        #     mcfg.addNode(motion[0].skeleton.getElementName(i))
        for name in massMap:
            node = mcfg.addNode(name)
            node.mass = massMap[name]
        # totalMass += node.mass

        node = mcfg.getNode('Hips')
        node.geom = 'MyFoot3'
        # node.geom = 'MyBox'
        node.jointType = "B"
        # node.length = 1.
        node.mass = 1.

        node = mcfg.getNode('foot00')
        node.geom = 'MyFoot4'
        # node.geom = 'MyBox'
        node.jointType = "U"
        node.mass = 1.

        node = mcfg.getNode('foot01')
        node.geom = 'MyFoot4'
        # node.geom = 'MyBox'
        node.jointType = "U"
        node.mass = 1.

        def mcfgFix(_mcfg):
            """

            :param _mcfg: ypc.ModelConfig
            :return:
            """
            # for v in _mcfg.nodes.itervalues():
            for k, v in _mcfg.nodes:
                if len(v.geoms) == 0:
                    v.geoms.append(v.geom)
                    v.geomMass.append(v.mass)
                    v.geomTs.append(None)

        # mcfgFix(mcfg)

        wcfg = ypc.WorldConfig()
        wcfg.planeHeight = 0.
        wcfg.useDefaultContactModel = False
        stepsPerFrame = 40
        simulSpeedInv = 1.

        wcfg.timeStep = (1/30.*simulSpeedInv)/stepsPerFrame

        # parameter
        config = dict([])
        config['Kt'] = 20; config['Dt'] = 2*(config['Kt']**.5)  # tracking gain
        config['Kl'] = 1; config['Dl'] = 2*(config['Kl']**.5)  # linear balance gain
        config['Kh'] = 1; config['Dh'] = 2*(config['Kh']**.5)  # angular balance gain
        config['Ks'] = 5000; config['Ds'] = 2*(config['Ks']**.5)  # penalty force spring gain
        config['Bt'] = 1.
        config['Bl'] = 1.
        config['Bh'] = 1.
        config['stepsPerFrame'] = stepsPerFrame
        config['simulSpeedInv'] = simulSpeedInv

        # etc
        config['weightMap'] = {'root': 1., 'foot00': 1., 'foot01': 1.}
        config['weightMapTuple'] = (1., 1., 1.)
        # config['supLink'] = 'link0'

        return motion, mcfg, wcfg, stepsPerFrame, config

    # model setting
    motion, mcfg, wcfg, stpesPerFrame, config = create_foot('test2.bvh')
    pydart.init()
    dartModel = cdm.DartModel(wcfg, motion[0], mcfg)
    q = dartModel.skeleton.q
    q[4] = 2.
    dartModel.skeleton.set_positions(q)
    dartModel.world.set_gravity(np.array((0., 0., 0.)))

    # viewer setting
    viewer = hsv.hpSimpleViewer()
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('dartModel', yr.DartModelRenderer(dartModel, CHARACTER_COLOR))

    jointOrientations = []
    jointPositions = []
    bodyOrientations = []
    bodyPositions = []
    viewer.doc.addRenderer('joint coord', yr.OrientationsRenderer(jointOrientations, jointPositions))
    viewer.doc.addRenderer('body coord', yr.OrientationsRenderer(bodyOrientations, bodyPositions))

    viewer.objectInfoWnd.add1DSlider('joint1 x torque', minVal=-2., maxVal=2., initVal=0., valStep=.01)
    viewer.objectInfoWnd.add1DSlider('joint1 y torque', minVal=-2., maxVal=2., initVal=0., valStep=.01)
    viewer.objectInfoWnd.add1DSlider('joint2 x torque', minVal=-2., maxVal=2., initVal=0., valStep=.01)
    viewer.objectInfoWnd.add1DSlider('joint2 y torque', minVal=-2., maxVal=2., initVal=0., valStep=.01)


    def simulateCallback(frame):
        # tau = np.zeros(dartModel.skeleton.q.shape)
        # tau[6] = viewer.objectInfoWnd.getVal('joint1 x torque')
        # tau[7] = viewer.objectInfoWnd.getVal('joint1 y torque')
        # tau[8] = viewer.objectInfoWnd.getVal('joint2 x torque')
        # tau[9] = viewer.objectInfoWnd.getVal('joint2 y torque')
        vel = np.zeros(dartModel.skeleton.velocities().shape)
        vel[6] = viewer.objectInfoWnd.getVal('joint1 x torque')
        vel[7] = viewer.objectInfoWnd.getVal('joint1 y torque')
        vel[8] = viewer.objectInfoWnd.getVal('joint2 x torque')
        vel[9] = viewer.objectInfoWnd.getVal('joint2 y torque')
        dartModel.skeleton.set_velocities(vel)
        # print mm.logSO3(dartModel.getJoint(2).get_local_transform()[:3, :3])

        for i in range(stpesPerFrame):
            dartModel.skeleton.set_velocities(vel)
            dartModel.step()

        del jointOrientations[:]
        del jointPositions[:]
        del bodyOrientations[:]
        del bodyPositions[:]
        for i in range(1, len(dartModel.skeleton.joints)):
            # jointOrientations.append(dartModel.getJoint(i).get_world_frame_before_transform())
            jointOrientations.append(dartModel.getJoint(i).get_world_frame_after_transform())
            jointPositions.append(dartModel.getJointPositionGlobal(i))
        for i in range(len(dartModel.skeleton.bodynodes)):
            bodyOrientations.append(dartModel.getBodyOrientationGlobal(i))
            bodyPositions.append(dartModel.getBodyPositionGlobal(i))

    viewer.setSimulateCallback(simulateCallback)

    viewer.startTimer(1 / 30.)
    viewer.show()
    # splineEditor = hse.SplineEditor()
    # splineEditor.show()

    Fl.run()

basicTest3()
