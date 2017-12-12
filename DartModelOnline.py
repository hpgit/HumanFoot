from PyCommon.modules.Simulator import csDartModel as cdm
from PyCommon.modules.Math import mmMath as mm
import numpy as np
import os


class DartModelServer(cdm.DartModel):
    def __init__(self, wcfg, posture, mcfg):
        cdm.DartModel.__init__(self, wcfg, posture, mcfg)


class DartModelClient(cdm.DartModel):
    def __init__(self, wcfg, posture, mcfg):
        cdm.DartModel.__init__(self, wcfg, posture, mcfg)


if __name__ == '__main__':
    SEGMENT_FOOT = True
    SEGMENT_FOOT_MAG = .03
    SEGMENT_FOOT_RAD = SEGMENT_FOOT_MAG * .5

    def buildMassMap():
        massMap = {}
        massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips',
                                    'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector',
                                    'LeftLeg', 'LeftShoulder1', 'LeftUpLeg',
                                    'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector',
                                    'RightLeg', 'RightShoulder', 'RightUpLeg',
                                    'Spine', 'Spine1',
                                    'RightFoot_foot_0_0', 'RightFoot_foot_0_1', 'RightFoot_foot_0_1_Effector',
                                    'RightFoot_foot_1_0', 'RightFoot_foot_1_1', 'RightFoot_foot_1_1_Effector',
                                    'RightFoot_foot_2_0', 'RightFoot_foot_2_1', 'RightFoot_foot_2_1_Effector',
                                    'LeftFoot_foot_0_0', 'LeftFoot_foot_0_1', 'LeftFoot_foot_0_1_Effector',
                                    'LeftFoot_foot_1_0', 'LeftFoot_foot_1_1', 'LeftFoot_foot_1_1_Effector',
                                    'LeftFoot_foot_2_0', 'LeftFoot_foot_2_1', 'LeftFoot_foot_2_1_Effector',
                                    ], 0.)

        # torso : 10
        massMap['Hips'] += 2.
        massMap['Spine'] += 8.

        # head : 3
        massMap['Spine1'] += 3.

        # right upper arm : 2
        massMap['RightArm'] += 2.

        # left upper arm : 2
        massMap['LeftArm'] += 2.

        # right lower arm : 1
        massMap['RightForeArm'] = 1.
        #    massMap['RightForeArm'] = 2.

        # left lower arm : 1
        massMap['LeftForeArm'] = 1.
        #    massMap['LeftForeArm'] = 2.

        # right thigh : 7
        massMap['Hips'] += 2.
        massMap['RightUpLeg'] += 5.

        # left thigh : 7
        massMap['Hips'] += 2.
        massMap['LeftUpLeg'] += 5.

        # right shin : 5
        massMap['RightLeg'] += 5.

        # left shin : 5
        massMap['LeftLeg'] += 5.

        # right foot : 4
        massMap['RightFoot'] += 2.
        # massMap['RightFoot'] += .4

        # left foot : 4
        massMap['LeftFoot'] += 2.
        # massMap['LeftFoot'] += .4
        '''
        massMap['RightFoot_foot_0_0'] = .3
        massMap['RightFoot_foot_0_1'] = .3
        massMap['RightFoot_foot_1_0'] = .3
        massMap['RightFoot_foot_1_1'] = .3
        massMap['RightFoot_foot_2_0'] = .3
        massMap['RightFoot_foot_2_1'] = .3
        massMap['LeftFoot_foot_0_0'] = .3
        massMap['LeftFoot_foot_0_1'] = .3
        massMap['LeftFoot_foot_1_0'] = .3
        massMap['LeftFoot_foot_1_1'] = .3
        massMap['LeftFoot_foot_2_0'] = .3
        massMap['LeftFoot_foot_2_1'] = .3
        #'''

        massMap['RightFoot_foot_0_0'] = .1
        massMap['RightFoot_foot_0_1'] = .1
        massMap['RightFoot_foot_0_0_0'] = .1
        massMap['RightFoot_foot_0_1_0'] = .1
        massMap['RightFoot_foot_1_0'] = .1
        massMap['RightFoot_foot_1_1'] = .1
        massMap['RightFoot_foot_1_2'] = .1
        massMap['LeftFoot_foot_0_0'] = .1
        massMap['LeftFoot_foot_0_1'] = .1
        massMap['LeftFoot_foot_0_0_0'] = .1
        massMap['LeftFoot_foot_0_1_0'] = .1
        massMap['LeftFoot_foot_1_0'] = .1
        massMap['LeftFoot_foot_1_1'] = .1
        massMap['LeftFoot_foot_1_2'] = .1

        return massMap


    def buildMcfg():
        massMap = buildMassMap()
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .9

        totalMass = 0.
        for name in massMap:
            node = mcfg.addNode(name)
            node.mass = massMap[name]
            # totalMass += node.mass

        # width : x axis on body frame
        # height: y axis on body frame
        # length: z axis on body frame
        node = mcfg.getNode('Hips')
        node.length = .2
        node.width = .25

        node = mcfg.getNode('Spine1')
        node.length = .2
        node.offset = (0,0,0.1)

        node = mcfg.getNode('Spine')
        node.width = .22

        node = mcfg.getNode('RightFoot')
        node.length = .25
        #    node.length = .27
        #    node.offset = (0,0,0.01)
        node.width = .1
        node.geom = 'MyFoot1'

        node = mcfg.getNode('LeftFoot')
        node.length = .25
        #    node.length = .27
        #    node.offset = (0,0,0.01)
        node.width = .1
        node.geom = 'MyFoot1'

        def capsulize(node_name):
            node_capsule = mcfg.getNode(node_name)
            node_capsule.geom = 'MyFoot4'
            node_capsule.width = 0.01
            node_capsule.density = 200.
            # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., math.pi/4., 0.])], ypc.CapsuleMaterial(1000., .02, .2))
            # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., math.pi/4., 0.])], ypc.CapsuleMaterial(1000., .02, .1))
            # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., 0., 0.])], ypc.CapsuleMaterial(1000., .01, -1))
            # node.addGeom('MyFoot4', None, ypc.CapsuleMaterial(1000., .02, .1))

        # capsulize('RightFoot')
        # capsulize('LeftFoot')

        if SEGMENT_FOOT:
            node = mcfg.getNode('RightFoot')
            node.density = 200.
            node.geom = 'MyFoot5'
            node.width = 0.01
            node.jointType = 'B'

            node = mcfg.getNode('LeftFoot')
            node.density = 200.
            node.geom = 'MyFoot5'
            node.width = 0.01
            node.jointType = 'B'

        # bird foot
        # capsulize('RightFoot_foot_0_0')
        # capsulize('RightFoot_foot_0_1')
        # capsulize('RightFoot_foot_1_0')
        # capsulize('RightFoot_foot_1_1')
        # capsulize('RightFoot_foot_2_0')
        # capsulize('RightFoot_foot_2_1')
        # capsulize('LeftFoot_foot_0_0')
        # capsulize('LeftFoot_foot_0_1')
        # capsulize('LeftFoot_foot_1_0')
        # capsulize('LeftFoot_foot_1_1')
        # capsulize('LeftFoot_foot_2_0')
        # capsulize('LeftFoot_foot_2_1')


        # human foot
        if SEGMENT_FOOT:
            footJointType = 'B'
            capsulDensity = 400.

            # RightFoot_foot_0_0 : outside metatarsals
            capsulize('RightFoot_foot_0_0')
            node = mcfg.getNode('RightFoot_foot_0_0')
            node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-0.3, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])],
                         ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.5 + 2.*SEGMENT_FOOT_RAD))
            node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-0.3-1.2, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])],
                         ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.5 + 2.*SEGMENT_FOOT_RAD))
            # node.addGeom('MyFoot4', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0., 0., 0.])], ypc.CapsuleMaterial(1000., .01, -1))
            node.jointType = footJointType

            # RightFoot_foot_0_0_0 : outside phalanges
            capsulize('RightFoot_foot_0_0_0')
            node = mcfg.getNode('RightFoot_foot_0_0_0')
            node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType

            # RightFoot_foot_0_1 : inside metatarsals
            capsulize('RightFoot_foot_0_1')
            node = mcfg.getNode('RightFoot_foot_0_1')
            node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity,SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType

            # RightFoot_foot_0_1_0 : inside phalanges
            capsulize('RightFoot_foot_0_1_0')
            node = mcfg.getNode('RightFoot_foot_0_1_0')
            node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType

            # RightFoot_foot_1_0 : center heel
            capsulize('RightFoot_foot_1_0')
            node = mcfg.getNode('RightFoot_foot_1_0')
            node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0., 0., .7]), mm.exp([0.]*3)],
                         ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2. + SEGMENT_FOOT_RAD * 2.))
            # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
            node.jointType = footJointType

            # RightFoot_foot_1_1 : inside heel
            capsulize('RightFoot_foot_1_1')
            node = mcfg.getNode('RightFoot_foot_1_1')
            node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType

            # RightFoot_foot_1_2 : outside heel
            capsulize('RightFoot_foot_1_2')
            node = mcfg.getNode('RightFoot_foot_1_2')
            node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType


            capsulize('LeftFoot_foot_0_0')
            node = mcfg.getNode('LeftFoot_foot_0_0')
            node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0.3, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])],
                         ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.5+2.*SEGMENT_FOOT_RAD))
            node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0.3+1.2, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])],
                         ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.5+2.*SEGMENT_FOOT_RAD))
            # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
            node.jointType = footJointType

            capsulize('LeftFoot_foot_0_0_0')
            node = mcfg.getNode('LeftFoot_foot_0_0_0')
            node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType

            capsulize('LeftFoot_foot_0_1')
            node = mcfg.getNode('LeftFoot_foot_0_1')
            node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType

            capsulize('LeftFoot_foot_0_1_0')
            node = mcfg.getNode('LeftFoot_foot_0_1_0')
            node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType

            capsulize('LeftFoot_foot_1_0')
            node = mcfg.getNode('LeftFoot_foot_1_0')
            node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0., 0., .7]), mm.exp([0.]*3)],
                         ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.0+2.*SEGMENT_FOOT_RAD))
            # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
            node.jointType = footJointType

            capsulize('LeftFoot_foot_1_1')
            node = mcfg.getNode('LeftFoot_foot_1_1')
            node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType

            capsulize('LeftFoot_foot_1_2')
            node = mcfg.getNode('LeftFoot_foot_1_2')
            node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
            node.jointType = footJointType

        return mcfg

    current_path = os.path.dirname(os.path.abspath(__file__))
    motionDir = current_path+'/ppmotion/'
    filename = 'wd2_WalkForwardNormal00_REPEATED.bvh'
    if SEGMENT_FOOT:
        filename = 'segfoot_'+filename

    mcfg = buildMcfg()

    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    wcfg.lockingVel = c_locking_vel
    stepsPerFrame = 50
    wcfg.timeStep = frameTime/stepsPerFrame
    dart_model_online = DartModelServer(wcfg)