import copy

import sys
#if '../PyCommon/modules' not in sys.path:
#    sys.path.append('../PyCommon/modules')
if './modules' not in sys.path:
    sys.path.append('./modules')

import os
import PyCommon.modules.Resource.ysMotionLoader as yf
import PyCommon.modules.Simulator.ysPhysConfig as ypc
import PyCommon.modules.Math.mmMath as mm
import PyCommon.modules.Motion.ysHierarchyEdit as yme
import PyCommon.modules.Motion.ysMotion as ym

import numpy as np
import math

def testtest():
    motionName = 'wd2_n_kick.bvh'
    motion = yf.readBvhFile(motionName)
    yf.writeBvhFile()


def create_biped(SEGMENT_FOOT=True, SEGMENT_FOOT_MAG=.03):
    SEGMENT_FOOT_RAD = SEGMENT_FOOT_MAG * .5

    #motion
    motionName = 'wd2_n_kick.bvh'
    #motionName = 'wd2_jump.bvh'
    #motionName = 'wd2_stand.bvh'
    bvh = yf.readBvhFileAsBvh(motionName)
    bvh.set_scale(.01)

    if SEGMENT_FOOT:
        # partBvhFilePath = '../PyCommon/modules/samples/simpleJump_long_test2.bvh'
        current_path = os.path.dirname(os.path.abspath(__file__))
        partBvhFilePath = current_path+'/../../PyCommon/modules/samples/simpleJump_long_test3.bvh'
        partBvh = yf.readBvhFileAsBvh(partBvhFilePath)
        bvh.replaceJointFromBvh('RightFoot', partBvh, SEGMENT_FOOT_MAG)
        partBvh = yf.readBvhFileAsBvh(partBvhFilePath)
        partBvh.mirror('YZ')
        bvh.replaceJointFromBvh('LeftFoot', partBvh, SEGMENT_FOOT_MAG)

    motion = bvh.toJointMotion(1., False)

    # motion.translateByOffset((0., 0.15, 0.))
    # motion.translateByOffset((0., -0.12, 0.))
    # motion.rotateByOffset(mm.rotZ(math.pi*1./18.))

    # motion = yf.readBvhFile(motionName, .01)
    # yme.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
    # yme.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)
    yme.rotateJointLocal(motion, 'Hips', mm.exp(mm.v3(1,0,0), .01), False)
    #yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.0,.3), -.5), False)
    #yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.0,-.3), -.5), False)
    # yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.5,0), -.6), False)
    # yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.5,0), -.6), False)
    # yme.removeJoint(motion, 'RightFoot_foot_1_1')
    # yme.removeJoint(motion, 'RightFoot_foot_1_2')
    # yme.removeJoint(motion, 'LeftFoot_foot_1_1')
    # yme.removeJoint(motion, 'LeftFoot_foot_1_2')

    yme.updateGlobalT(motion)
    motion.translateByOffset((0, 0.04, 0))

    #motion = motion[40:-58]
    #motion[0:0] = [motion[0]]*20
    #motion.extend([motion[-1]]*5000)

    # motion = motion[40:]
    #motion[0:0] = [motion[0]]*50
    #motion.extend([motion[-1]]*5000)

    #motion = motion[30:151]
    motion = motion[30:]
    #motion = motion[30:31]
    #motion[5:5] = [motion[5]]*30
    # motion[0:0] = [motion[0]]*2000
    # motion.extend([motion[-1]]*300)

    # motion = motion[37:]
    motion[0:0] = [motion[0]]*2000
    motion.extend([motion[-1]]*300)

    #motion = motion[40:41]
    #motion[0:0] = [motion[0]]*5000

    #motion = motion[56:-248]

    #motion = motion[-249:-248]
    #motion[0:0] = [motion[0]]*10
    #motion.extend([motion[-1]]*5000)

    # world, model
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .9

    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]


    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 60
    #stepsPerFrame = 30
    frame_rate = 30
    wcfg.timeStep = 1./(frame_rate * stepsPerFrame)
    # wcfg.timeStep = (1/30.)/(stepsPerFrame)
    #wcfg.timeStep = (1/1000.)

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
    #node.length = .2
    #node.width = .15
    node.width = .2
    node.mass = 2.

    node = mcfg.getNode('LeftFoot')
    node.length = .25
    #node.length = .2
    #node.width = .15
    node.width = .2
    node.mass = 2.

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
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-.6, 0., 0.]), mm.exp([0.]*3)],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*1.2+2.*SEGMENT_FOOT_RAD))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([+.6, 0., 0.]), mm.exp([0.]*3)],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*1.2+2.*SEGMENT_FOOT_RAD))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        # RightFoot_foot_1_1 : inside heel
        # capsulize('RightFoot_foot_1_1')
        # node = mcfg.getNode('RightFoot_foot_1_1')
        # node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        # node.jointType = footJointType

        # RightFoot_foot_1_2 : outside heel
        # capsulize('RightFoot_foot_1_2')
        # node = mcfg.getNode('RightFoot_foot_1_2')
        # node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        # node.jointType = footJointType


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
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-.6, 0., .0]), mm.exp([0.]*3)],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*1.2+2.*SEGMENT_FOOT_RAD))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([+.6, 0., .0]), mm.exp([0.]*3)],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*1.2+2.*SEGMENT_FOOT_RAD))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        # capsulize('LeftFoot_foot_1_1')
        # node = mcfg.getNode('LeftFoot_foot_1_1')
        # node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        # node.jointType = footJointType

        # capsulize('LeftFoot_foot_1_2')
        # node = mcfg.getNode('LeftFoot_foot_1_2')
        # node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        # node.jointType = footJointType

    # parameter
    config = {}
    '''
    config['Kt'] = 200;      config['Dt'] = 2*(config['Kt']**.5) # tracking gain
    config['Kl'] = 2.5;       config['Dl'] = 2*(config['Kl']**.5) # linear balance gain
    config['Kh'] = 1;       config['Dh'] = 2*(config['Kh']**.5) # angular balance gain
    config['Ks'] = 20000;   config['Ds'] = 2*(config['Ks']**.5) # penalty force spring gain
    config['Bt'] = 1.
    config['Bl'] = 2.5
    config['Bh'] = 1.
    '''
    config['Kt'] = 200;      config['Dt'] = 2*(config['Kt']**.5) # tracking gain
    config['Kl'] = .10;       config['Dl'] = 2*(config['Kl']**.5) # linear balance gain
    config['Kh'] = 0.1;       config['Dh'] = 2*(config['Kh']**.5) # angular balance gain
    config['Ks'] = 15000;   config['Ds'] = 2*(config['Ks']**.5) # penalty force spring gain
    config['Bt'] = 1.
    config['Bl'] = 1.#0.5
    config['Bh'] = 1.
    #config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
                         #'Spine':1., 'Spine1':1., 'RightFoot':.5, 'LeftFoot':.5, 'Hips':1.5,\
                         #'RightUpLeg':1., 'RightLeg':1., 'LeftUpLeg':1., 'LeftLeg':1.}

    #config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
                         #'Spine':1., 'Spine1':1., 'RightFoot':1.0, 'LeftFoot':1.0, 'Hips':1.5,\
                         #'RightUpLeg':2., 'RightLeg':2., 'LeftUpLeg':2., 'LeftLeg':2.}
    config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,
                         'Spine':.6, 'Spine1':.6, 'RightFoot':.2, 'LeftFoot':.2, 'Hips':0.5,
                         'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.1, 'LeftLeg':.3}
    if SEGMENT_FOOT:
        segfoot_weight = .1
        config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,
                             'Spine':.6, 'Spine1':.6, 'RightFoot':.2, 'LeftFoot':.2, 'Hips':0.5,
                             'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.1, 'LeftLeg':.3,
        'RightFoot_foot_0_0':segfoot_weight, 'RightFoot_foot_0_1':segfoot_weight,
        'RightFoot_foot_1_0':segfoot_weight, 'RightFoot_foot_1_1':segfoot_weight, 'RightFoot_foot_1_2':segfoot_weight,
        'RightFoot_foot_0_0_0':segfoot_weight, 'RightFoot_foot_0_1_0':segfoot_weight,
        'LeftFoot_foot_0_0':segfoot_weight, 'LeftFoot_foot_0_1':segfoot_weight,
        'LeftFoot_foot_1_0':segfoot_weight, 'LeftFoot_foot_1_1':segfoot_weight, 'LeftFoot_foot_1_2':segfoot_weight,
        'LeftFoot_foot_0_0_0':segfoot_weight, 'LeftFoot_foot_0_1_0':segfoot_weight}


    #config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
                         #'Spine':.6, 'Spine1':.6, 'RightFoot':.2, 'LeftFoot':1., 'Hips':0.5,\
                         #'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.5, 'LeftLeg':1.5}

    #success!!
    '''
    config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
                         'Spine':.5, 'Spine1':.5, 'RightFoot':1., 'LeftFoot':1., 'Hips':0.5,\
                         'RightUpLeg':1., 'RightLeg':1., 'LeftUpLeg':1., 'LeftLeg':1.}
    '''

    #config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
                         #'Spine':1.5, 'LeftFoot':1., 'Hips':1.5,\
                         #'RightUpLeg':1., 'RightLeg':1., 'LeftUpLeg':1.5, 'LeftLeg':1.5}

    config['supLink'] = 'LeftFoot'
    config['supLink1'] = 'LeftFoot'
    config['supLink2'] = 'RightFoot'
    #config['end'] = 'Hips'
    config['end'] = 'Spine1'


    return motion, mcfg, wcfg, stepsPerFrame, config, frame_rate

#===============================================================================
# biped config
#===============================================================================

# motion, mesh config
g_motionDirConfigMap = {}
g_motionDirConfigMap['../Data/woody2/Motion/Physics2/'] = \
    {'footRot': mm.exp(mm.v3(1,0,0), .05), 'yOffset': .0, 'scale':1.,\
     'rootRot': mm.I_SO3()}
g_motionDirConfigMap['../Data/woody2/Motion/Balancing/'] = \
    {'footRot': mm.exp(mm.v3(1,-.5,0), -.6), 'yOffset': .0, 'scale':1.,\
     'rootRot': mm.exp(mm.v3(1,0,0), .01)}
g_motionDirConfigMap['../Data/woody2/Motion/VideoMotion/'] = \
    {'footRot': mm.exp(mm.v3(1,0,0), -.05), 'yOffset': .01, 'scale':2.53999905501,\
     'rootRot': mm.exp(mm.v3(1,0,0), .0)}
g_motionDirConfigMap['../Data/woody2/Motion/Samsung/'] = \
    {'footRot': mm.exp(mm.v3(1,0,0), -.03), 'yOffset': .0, 'scale':2.53999905501,\
     'rootRot': mm.exp(mm.v3(1,0,0), .03)}


#===============================================================================
# # reloadable config
#===============================================================================
def buildMassMap():
    massMap = {}
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips',
                                'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector',
                                'LeftLeg', 'LeftShoulder1', 'LeftUpLeg',
                                'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector',
                                'RightLeg', 'RightShoulder', 'RightUpLeg',
                                'Spine', 'Spine1',
                                'RightFoot_foot_0_0', 'RightFoot_foot_0_1',
                                'RightFoot_foot_1_0', 'RightFoot_foot_1_1', 'RightFoot_foot_1_2',
                                'RightFoot_foot_0_0_0', 'RightFoot_foot_0_1_0',
                                'LeftFoot_foot_0_0', 'LeftFoot_foot_0_1',
                                'LeftFoot_foot_1_0', 'LeftFoot_foot_1_1', 'LeftFoot_foot_1_2',
                                'LeftFoot_foot_0_0_0', 'LeftFoot_foot_0_1_0',
                                ], 0.)

    # torso : 10
    massMap['Hips'] += 2.
    #massMap['Spine'] += 8.
    massMap['Spine'] += 8.

    # head : 3
    massMap['Spine1'] += 3.

    # right upper arm : 2
    massMap['RightArm'] += 2.

    # left upper arm : 2
    massMap['LeftArm'] += 2.

    # right lower arm : 1
    massMap['RightForeArm'] = 1.
    # massMap['RightForeArm'] = 2.

    # left lower arm : 1
    massMap['LeftForeArm'] = 1.
    # massMap['LeftForeArm'] = 2.

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

    # left foot : 4
    massMap['LeftFoot'] += 2.

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
massMap = buildMassMap()
