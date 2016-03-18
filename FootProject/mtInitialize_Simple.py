import copy

import sys
#if '../PyCommon/modules' not in sys.path:
#    sys.path.append('../PyCommon/modules')
if './modules' not in sys.path:
    sys.path.append('./modules')

import Resource.ysMotionLoader as yf
import Simulator.ysPhysConfig as ypc
import Math.mmMath as mm
import Motion.ysHierarchyEdit as yme
import Motion.ysMotion as ym
import math

# Constant
HIP = 'Hips'
RIGHT_UP_LEG = 'RightUpLeg'
RIGHT_LEG = 'RightLeg'
RIGHT_FOOT = 'RightFoot'
RIGHT_TOES = 'RightToes'
RIGHT_TOES_END = 'RightToes_Effector'
LEFT_UP_LEG = 'LeftUpLeg'
LEFT_LEG = 'LeftLeg'
LEFT_FOOT = 'LeftFoot'
LEFT_TOES = 'LeftToes'
LEFT_TOES_END = 'LeftToes_Effector'

LEFT_SHOULDER = 'LeftShoulder1'
LEFT_ARM = 'LeftArm'
LEFT_FORE_ARM = 'LeftForeArm'
LEFT_HAND = 'LeftHand'
LEFT_HAND_END = 'LeftHand_Effector'
RIGHT_SHOULDER = 'RightShoulder'
RIGHT_ARM = 'RightArm'
RIGHT_FORE_ARM = 'RightForeArm'
RIGHT_HAND = 'RightHand'
RIGHT_HAND_END = 'RightHand_Effector'

SPINE = 'Spine'
SPINE1 = 'Spine1'
HEAD = 'HEad'
HEAD_END = 'HEad_Effector'

RIGHT_METATARSAL_1 = 'RightMetatarsal_1'
RIGHT_METATARSAL_2 = 'RightMetatarsal_2'
RIGHT_METATARSAL_3 = 'RightMetatarsal_3'
RIGHT_PHALANGE_1 = 'RightPhalange_1'
RIGHT_PHALANGE_2 = 'RightPhalange_2'
RIGHT_PHALANGE_3 = 'RightPhalange_3'
RIGHT_CALCANEUS_1 = 'RightCalcaneus_1'
RIGHT_CALCANEUS_2 = 'RightCalcaneus_2'
RIGHT_CALCANEUS_3 = 'RightCalcaneus_3'

RIGHT_TALUS_1 = 'RightTalus_1'
RIGHT_TALUS_2 = 'RightTalus_2'
RIGHT_TALUS_3 = 'RightTalus_3'

LEFT_METATARSAL_1 = 'LeftMetatarsal_1'
LEFT_METATARSAL_2 = 'LeftMetatarsal_2'
LEFT_METATARSAL_3 = 'LeftMetatarsal_3'
LEFT_PHALANGE_1 = 'LeftPhalange_1'
LEFT_PHALANGE_2 = 'LeftPhalange_2'
LEFT_PHALANGE_3 = 'LeftPhalange_3'
LEFT_TALUS_1 = 'LeftTalus_1'
LEFT_TALUS_2 = 'LeftTalus_2'
LEFT_TALUS_3 = 'LeftTalus_3'
LEFT_CALCANEUS_1 = 'LeftCalcaneus_1'
LEFT_CALCANEUS_2 = 'LeftCalcaneus_2'
LEFT_CALCANEUS_3 = 'LeftCalcaneus_3'
    
STAND = 0
FORWARD_JUMP = 1
TAEKWONDO = 2
TAEKWONDO2 = 3
STAND2 = 4
KICK = 5
WALK = 6
TIPTOE = 7

## Motion File
MOTION = STAND
#MOTION = STAND2
#MOTION = FORWARD_JUMP
#MOTION = TAEKWONDO
#MOTION = TAEKWONDO2
#MOTION = KICK
#MOTION = WALK
#MOTION = TIPTOE

FOOT_PART_NUM = 7


def normal_mcfg():
    massMap = buildMassMap()
        
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .9
        
    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]

    node = mcfg.getNode('Hips')
    node.length = .2
    node.width = .25 
    
    node = mcfg.getNode('Spine1')
    node.length = .2
    node.offset = (0, 0, 0.1)
    
    node = mcfg.getNode('Spine')
    node.width = .22
        
    node = mcfg.getNode('RightFoot')
    node.length = .25
    node.width = .1
        
    node = mcfg.getNode('LeftFoot')
    node.length = .25
    node.width = .1

    return mcfg


def create_vchain_5():
    # motion
    motion = yf.readBvhFile('vchain_5_rotate_root0.bvh', 1)
    
    # world, model
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
        
    node = mcfg.getNode('link0')
    node.width = .3
    node.mass = 6.
        
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 120
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    # parameter
    config = {}
    config['Kt'] = 20; config['Dt'] = 2*(config['Kt']**.5) # tracking gain
    config['Kl'] = 1; config['Dl'] = 2*(config['Kl']**.5) # linear balance gain
    config['Kh'] = 1; config['Dh'] = 2*(config['Kh']**.5) # angular balance gain
    config['Ks'] = 5000; config['Ds'] = 2*(config['Ks']**.5) # penalty force spring gain
    config['Bt'] = 1.
    config['Bl'] = 1.
    config['Bh'] = 1.
    
    # etc
    config['weightMap'] = {}
    config['supLink'] = 'link0'
    
    return motion, mcfg, wcfg, stepsPerFrame, config


def create_vchain_1():
    # motion
    motion = yf.readBvhFile('vchain_1_rotate_root0.bvh', 1)
    
    # world, model
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .8
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
        
    node = mcfg.getNode('Hips')
    #node.width = .3
    #node.mass = 6.
        
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 120
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    # parameter
    config = {}
    config['Kt'] = 20; config['Dt'] = 2*(config['Kt']**.5) # tracking gain
    config['Kl'] = 1; config['Dl'] = 2*(config['Kl']**.5) # linear balance gain
    config['Kh'] = 1; config['Dh'] = 2*(config['Kh']**.5) # angular balance gain
    config['Ks'] = 5000; config['Ds'] = 2*(config['Ks']**.5) # penalty force spring gain
    config['Bt'] = 1.
    config['Bl'] = 1.
    config['Bh'] = 1.
    
    # etc
    config['weightMap'] = {}
    config['supLink'] = 'link0'
    
    return motion, mcfg, wcfg, stepsPerFrame, config


def create_chiken_foot():
    # motion
    motion = yf.readBvhFile('chikenFoot.bvh', 1)
    
    # world, model
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = 1.
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    node = mcfg.getNode('root')
    node.geom = 'MyFoot3'
    node.length = .1
    # node.width = .05
    node.mass = 5.
    
    node = mcfg.getNode('foot00')
    node.geom = 'MyFoot3'
    node.mass = .5
    node = mcfg.getNode('foot10')
    node.geom = 'MyFoot3'
    node.mass = .5
    
    node = mcfg.getNode('foot01')
    node.geom = 'MyFoot4'
    node.mass = .5
    node = mcfg.getNode('foot11')
    node.geom = 'MyFoot4'
    node.mass = .5


    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 120
    wcfg.timeStep = (1/30.)/stepsPerFrame
    
    # parameter
    config = dict([])
    config['Kt'] = 20; config['Dt'] = 2*(config['Kt']**.5)  # tracking gain
    config['Kl'] = 1; config['Dl'] = 2*(config['Kl']**.5)  # linear balance gain
    config['Kh'] = 1; config['Dh'] = 2*(config['Kh']**.5)  # angular balance gain
    config['Ks'] = 5000; config['Ds'] = 2*(config['Ks']**.5)  # penalty force spring gain
    config['Bt'] = 1.
    config['Bl'] = 1.
    config['Bh'] = 1.

    # etc
    config['weightMap'] = {'root': 2., 'foot00': 1., 'foot10': 1., 'foot01': .2, 'foot11': .2}
    config['weightMapTuple'] = (2., 1., .2, 1., .2)
    # config['supLink'] = 'link0'
    
    return motion, mcfg, wcfg, stepsPerFrame, config


def create_foot(motionFile='foot3.bvh'):
    # motion
    motion = yf.readBvhFile(motionFile, .05)

    # world, model
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = 1.
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    node = mcfg.getNode('root')
    node.geom = 'MyFoot3'
    node.length = 1.
    node.mass = 5.

    node = mcfg.getNode('foot_0_0')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot_0_1')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot_1_0')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot_1_1')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot_2_0')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot_2_1')
    node.geom = 'MyFoot4'
    node.mass = .5

    def mcfgFix(_mcfg):
        for v in _mcfg.nodes.itervalues():
            if len(v.geoms) == 0:
                v.geoms.append(v.geom)
                v.geomMass.append(v.mass)
                v.geomTs.append(None)

    mcfgFix(mcfg)

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
    config['weightMap'] = {'root': 2., 'foot00': 1., 'foot10': 1., 'foot20': 1., 'foot01': .2, 'foot11': .2, 'foot21': .2}
    config['weightMapTuple'] = (2., 1., .2, 1., .2, 1., .2)
    # config['supLink'] = 'link0'

    return motion, mcfg, wcfg, stepsPerFrame, config


def create_leg(motionFile='kneeAndFoot.bvh'):
    # motion
    motion = yf.readBvhFile(motionFile, .05)

    # world, model
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = 1.
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))
    node = mcfg.getNode('root')
    # node.geom = ''
    # node.length = 40.
    # node.length = 1.
    # node.mass = 5.

    node = mcfg.getNode('knee')
    # node.geom = 'MyFoot4'
    # node.length = 40.

    node = mcfg.getNode('foot00')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot01')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot10')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot11')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot20')
    node.geom = 'MyFoot4'
    node.mass = .5

    node = mcfg.getNode('foot21')
    node.geom = 'MyFoot4'
    node.mass = .5

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
    config['weightMap'] = {'root': 2., 'knee': 2., 'foot00': 1., 'foot10': 1., 'foot20': 1., 'foot01': .2, 'foot11': .2, 'foot21': .2}
    config['weightMapTuple'] = (2., 2., 1., .2, 1., .2, 1., .2)
    # config['supLink'] = 'link0'

    return motion, mcfg, wcfg, stepsPerFrame, config


def create_legs(motionFile='legs.bvh'):
    # motion
    motion = yf.readBvhFile(motionFile, .06)

    # world, model
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = 1.
    for i in range(motion[0].skeleton.getElementNum()):
        mcfg.addNode(motion[0].skeleton.getElementName(i))

    node = mcfg.getNode('Hips')
    # node.geom = ''
    # node.length = 40.
    # node.length = 1.
    node.mass = 4.

    node = mcfg.getNode('Spine')
    node.geom = 'MyFoot4'
    node.mass = 10.
    node.width = .1

    node = mcfg.getNode('RightUpLeg')
    node.geom = 'MyFoot3'
    node.mass = 10.
    # node.length = 40.

    node = mcfg.getNode('RightLeg')
    node.geom = 'MyFoot4'
    node.mass = 10.
    # node.length = 40.

    node = mcfg.getNode('RightFoot')
    node.geom = 'MyFoot4'
    node.mass = 3.

    node = mcfg.getNode('RightToe')
    node.geom = 'MyFoot4'
    node.mass = 1.

    node = mcfg.getNode('RightCal')
    node.geom = 'MyFoot4'
    node.mass = 1.

    node = mcfg.getNode('RightHeel')
    node.geom = 'MyFoot4'
    node.mass = 1.

    node = mcfg.getNode('RightCune')
    node.geom = 'MyFoot4'
    node.mass = 2.

    node = mcfg.getNode('RightMt5')
    node.geom = 'MyFoot4'
    node.mass = 1.

    node = mcfg.getNode('LeftUpLeg')
    node.geom = 'MyFoot3'
    node.mass = 10.
    # node.length = 40.

    node = mcfg.getNode('LeftLeg')
    node.geom = 'MyFoot4'
    node.mass = 10.
    # node.length = 40.

    node = mcfg.getNode('LeftFoot')
    node.geom = 'MyFoot4'
    node.mass = 3.

    node = mcfg.getNode('LeftToe')
    node.geom = 'MyFoot4'
    node.mass = 1.

    node = mcfg.getNode('LeftCal')
    node.geom = 'MyFoot4'
    node.mass = 1.

    node = mcfg.getNode('LeftHeel')
    node.geom = 'MyFoot4'
    node.mass = 1.

    node = mcfg.getNode('LeftCune')
    node.geom = 'MyFoot4'
    node.mass = 2.

    node = mcfg.getNode('LeftMt5')
    node.geom = 'MyFoot4'
    node.mass = 1.

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
    config['weightMap'] = {'root': 2., 'knee': 2., 'foot00': 1., 'foot10': 1., 'foot20': 1., 'foot01': .2, 'foot11': .2, 'foot21': .2}
    config['weightMapTuple'] = (2., 2., 1., .2, 1., .2, 1., .2)
    # config['supLink'] = 'link0'

    return motion, mcfg, wcfg, stepsPerFrame, config


def create_biped_basic(motionName):
    scale = 1.
    motion = yf.readBvhFile(motionName, scale)
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 120.
    wcfg.timeStep = (1/30.)/stepsPerFrame
    #stepsPerFrame = 10
    #wcfg.timeStep = (1/120.)/(stepsPerFrame)
    #wcfg.timeStep = (1/1800.)


    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = 1.

    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]

    # parameter
    config = {}
    config['Kt'] = 200;      config['Dt'] = 2*(config['Kt']**.5) # tracking gain
    config['Kl'] = .10;       config['Dl'] = 2*(config['Kl']**.5) # linear balance gain
    config['Kh'] = 0.1;       config['Dh'] = 2*(config['Kh']**.5) # angular balance gain
    config['Ks'] = 20000;   config['Ds'] = 2*(config['Ks']**.5) # penalty force spring gain
    config['Bt'] = 1.
    config['Bl'] = 1.#0.5
    config['Bh'] = 1.

    config['weightMap'] = {RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:.3, SPINE1:.2, RIGHT_FOOT:.3, LEFT_FOOT:.3, HIP:.3,
                           RIGHT_UP_LEG:.1, RIGHT_LEG:.2, LEFT_UP_LEG:.1, LEFT_LEG:.2,
                           LEFT_TALUS_1:.1, RIGHT_TALUS_1:.1, LEFT_TALUS_2:.1, RIGHT_TALUS_2:.1,
                           RIGHT_CALCANEUS_1:.2, LEFT_CALCANEUS_1:.2, RIGHT_CALCANEUS_2:.2, LEFT_CALCANEUS_2:.2,
                           LEFT_PHALANGE_1:.1, LEFT_PHALANGE_2:.1, RIGHT_PHALANGE_1:.1, RIGHT_PHALANGE_2:.1}

    config['weightMap2'] = {RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:.5, SPINE1:.3, RIGHT_FOOT:.7, LEFT_FOOT:.7, HIP:.5,
                            RIGHT_UP_LEG:.7, RIGHT_LEG:.7, LEFT_UP_LEG:.7, LEFT_LEG:.7,
                            LEFT_TALUS_1:.7, RIGHT_TALUS_1:.7, LEFT_TALUS_2:.7, RIGHT_TALUS_2:.7,
                            RIGHT_CALCANEUS_1:.7, LEFT_CALCANEUS_1:.7, RIGHT_CALCANEUS_2:.7, LEFT_CALCANEUS_2:.7,
                            LEFT_PHALANGE_1:.4, LEFT_PHALANGE_2:.4, RIGHT_PHALANGE_1:.4, RIGHT_PHALANGE_2:.4}
    '''
    (1, 'RightUpLeg')
    (2, 'RightLeg')
    (3, 'RightFoot')
    (4, 'Spine')
    (5, 'Spine1')
    (6, 'LeftArm')
    (7, 'LeftForeArm')
    (8, 'RightArm')
    (9, 'RightForeArm')
    (10, 'LeftUpLeg')
    (11, 'LeftLeg')
    (12, 'LeftFoot')
    (13, 'LeftTalus_1')
    (14, 'LeftTalus_2')
    (15, 'RightTalus_1')
    (16, 'RightTalus_2')
    (17, 'LeftPhalange_1')
    (18, 'LeftPhalange_2')
    (19, 'RightPhalange_1')
    (20, 'RightPhalange_2')
    (21, 'LeftCalcaneus_1')
    (22, 'LeftCalcaneus_2')
    (23, 'RightCalcaneus_1')
    (24, 'RightCalcaneus_2')
    '''
    config['trackWMap']={10, 5, .1, 20, 10, 5, 2, 10, 5, .1,
                         0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.10, 0.01}

    config['supLink'] = LEFT_FOOT
    config['supLink2'] = RIGHT_FOOT
    # config['end'] = HIP
    config['end'] = SPINE1

    config['trunk'] = SPINE
    config['const'] = HIP
    config['root'] = HIP

    config['FootPartNum'] = FOOT_PART_NUM

    config['FootLPart'] = [LEFT_FOOT, LEFT_CALCANEUS_1, LEFT_CALCANEUS_2, LEFT_TALUS_1, LEFT_TALUS_2, LEFT_PHALANGE_1, LEFT_PHALANGE_2]
    config['FootRPart'] = [RIGHT_FOOT, RIGHT_CALCANEUS_1, RIGHT_CALCANEUS_2, RIGHT_TALUS_1, RIGHT_TALUS_2, RIGHT_PHALANGE_1, RIGHT_PHALANGE_2]

    return motion, mcfg, wcfg, stepsPerFrame, config


def create_biped():
    # motion
    # motionName = 'wd2_n_kick.bvh'
    
    if MOTION == STAND:
        # motionName = 'wd2_stand.bvh'
        motionName = 'wd2_stand2.bvh'
        # motionName = 'woddy2_jump0_short.bvh'
    elif MOTION == STAND2:
        motionName = 'ww13_41_V001.bvh'
    elif MOTION == FORWARD_JUMP:
        motionName = 'woddy2_jump0.bvh'       
    elif MOTION == TAEKWONDO  :
        motionName = './MotionFile/wd2_098_V001.bvh'
    elif MOTION == TAEKWONDO2  :
        motionName = './MotionFile/wd2_098_V001.bvh'
    elif MOTION == KICK :
        motionName = 'wd2_n_kick.bvh'
    elif MOTION == WALK :
        motionName = 'wd2_WalkForwardNormal00.bvh'
    elif MOTION == TIPTOE:
        motionName = './MotionFile/cmu/15_07_15_07.bvh'

    # motionName = 'ww13_41_V001.bvh'
    scale = 0.01
    if MOTION == WALK :
        scale = 1.0
    elif MOTION == TIPTOE:
        scale = 0.01

    motion = yf.readBvhFile(motionName, scale)

    yme.removeJoint(motion, HEAD, False)
    yme.removeJoint(motion, RIGHT_SHOULDER, False)
    yme.removeJoint(motion, LEFT_SHOULDER, False)
    
    yme.removeJoint(motion, RIGHT_TOES, False)
    yme.removeJoint(motion, RIGHT_TOES_END, False)
    yme.removeJoint(motion, LEFT_TOES, False)
    yme.removeJoint(motion, LEFT_TOES_END, False)       

    yme.removeJoint(motion, RIGHT_HAND_END, False)
    yme.removeJoint(motion, LEFT_HAND_END, False)
        
    yme.offsetJointLocal(motion, RIGHT_ARM, (.03, -.05, 0.), False)
    yme.offsetJointLocal(motion, LEFT_ARM, (-.03, -.05, 0.), False)
    yme.rotateJointLocal(motion, HIP, mm.exp(mm.v3(1., 0., 0.), .01), False)
    # yme.rotateJointLocal(motion, HIP, mm.exp(mm.v3(0,0,1), -.01), False)
    
    # addFootSegment
    # Talus
    length1 = .15
    width1 = .03
    mass1 = .4
    # Phalange
    length3 = length1*0.75
    width3  = width1
    mass3   = 0.4

    length3_2 = length1*0.6
    width3_2  = width1
    mass3_2   = 0.4

    #Calcaneus
    length4 = 0.05
    width4  = width1
    mass4   = 0.4
    
    width1 = 0.03

    yme.addJoint(motion, LEFT_FOOT, LEFT_TALUS_1,   (-0.025, -0.06, -0.05))
    yme.addJoint(motion, LEFT_FOOT, LEFT_TALUS_2,   ( 0.025, -0.06, -0.05))
    yme.addJoint(motion, RIGHT_FOOT, RIGHT_TALUS_1, ( 0.025, -0.06, -0.05))
    yme.addJoint(motion, RIGHT_FOOT, RIGHT_TALUS_2, (-0.025, -0.06, -0.05))
    
    yme.addJoint(motion, LEFT_TALUS_1,  LEFT_PHALANGE_1,  (0.0, 0.0, length1-width1-width3  ))
    yme.addJoint(motion, LEFT_TALUS_2,  LEFT_PHALANGE_2,  (0.0, 0.0, length1-width1-width3_2))
    yme.addJoint(motion, RIGHT_TALUS_1, RIGHT_PHALANGE_1, (0.0, 0.0, length1-width1-width3  ))
    yme.addJoint(motion, RIGHT_TALUS_2, RIGHT_PHALANGE_2, (0.0, 0.0, length1-width1-width3_2))

    yme.addJoint(motion, LEFT_PHALANGE_1,  'LEFT_PHALANGE_Effector1',  (0.0, 0.0, length3  -width3  ))
    yme.addJoint(motion, LEFT_PHALANGE_2,  'LEFT_PHALANGE_Effector2',  (0.0, 0.0, length3_2-width3_2))
    yme.addJoint(motion, RIGHT_PHALANGE_1, 'RIGHT_PHALANGE_Effector1', (0.0, 0.0, length3  -width3  ))
    yme.addJoint(motion, RIGHT_PHALANGE_2, 'RIGHT_PHALANGE_Effector2', (0.0, 0.0, length3_2-width3_2))


    yme.addJoint(motion, LEFT_FOOT,  LEFT_CALCANEUS_1,  (-0.025, -0.06, -0.05))
    yme.addJoint(motion, LEFT_FOOT,  LEFT_CALCANEUS_2,  ( 0.025, -0.06, -0.05))
    yme.addJoint(motion, RIGHT_FOOT, RIGHT_CALCANEUS_1, ( 0.025, -0.06, -0.05))
    yme.addJoint(motion, RIGHT_FOOT, RIGHT_CALCANEUS_2, (-0.025, -0.06, -0.05))
    yme.addJoint(motion, LEFT_CALCANEUS_1,  'LEFT_CALCANEUS_Effector1',  (0.0, 0.0, -length4))
    yme.addJoint(motion, LEFT_CALCANEUS_2,  'LEFT_CALCANEUS_Effector2',  (0.0, 0.0, -length4))
    yme.addJoint(motion, RIGHT_CALCANEUS_1, 'RIGHT_CALCANEUS_Effector1', (0.0, 0.0, -length4))
    yme.addJoint(motion, RIGHT_CALCANEUS_2, 'RIGHT_CALCANEUS_Effector2', (0.0, 0.0, -length4))
        




    yme.rotateJointLocal(motion, RIGHT_FOOT, mm.exp(mm.v3(1.0,0.0,0.0), -.5), False)        
    yme.rotateJointLocal(motion, RIGHT_TALUS_1, mm.exp(mm.v3(1.0,0.0,0.0), .5), False)
    yme.rotateJointLocal(motion, RIGHT_TALUS_2, mm.exp(mm.v3(1.0,0.0,0.0), .5), False)
    yme.rotateJointLocal(motion, RIGHT_CALCANEUS_1, mm.exp(mm.v3(0.0,0.0,1.0), 3.14), False)
    yme.rotateJointLocal(motion, RIGHT_CALCANEUS_2, mm.exp(mm.v3(0.0,0.0,1.0), 3.14), False)
    yme.rotateJointLocal(motion, RIGHT_CALCANEUS_1, mm.exp(mm.v3(1.0,0.0,0.0), -.5), False)
    yme.rotateJointLocal(motion, RIGHT_CALCANEUS_2, mm.exp(mm.v3(1.0,0.0,0.0), -.5), False)

    #yme.rotateJointLocal(motion, LEFT_PHALANGE_1, mm.exp(mm.v3(0., 1., 0.), 1.57), False)
        
    yme.rotateJointLocal(motion, LEFT_FOOT, mm.exp(mm.v3(1.0,0.0,0.0), -.5), False)        
    yme.rotateJointLocal(motion, LEFT_TALUS_1, mm.exp(mm.v3(1.0,0.0,0.0), .5), False)
    yme.rotateJointLocal(motion, LEFT_TALUS_2, mm.exp(mm.v3(1.0,0.0,0.0), .5), False)    
    yme.rotateJointLocal(motion, LEFT_CALCANEUS_1, mm.exp(mm.v3(0.0,0.0,1.0), 3.14), False)
    yme.rotateJointLocal(motion, LEFT_CALCANEUS_2, mm.exp(mm.v3(0.0,0.0,1.0), 3.14), False)
    yme.rotateJointLocal(motion, LEFT_CALCANEUS_1, mm.exp(mm.v3(1.0,0.0,0.0), -.5), False)
    yme.rotateJointLocal(motion, LEFT_CALCANEUS_2, mm.exp(mm.v3(1.0,0.0,0.0), -.5), False)
    #yme.rotateJointLocal(motion, LEFT_CALCANEUS_1, mm.exp(mm.v3(0.0,-1.0,0.0), 1.57), False)
        
       
    yme.updateGlobalT(motion)
    
    ################
    if MOTION == FORWARD_JUMP:        
        motion = motion[515:555]
    elif MOTION == TAEKWONDO:
    ## Taekwondo base-step
        motion = motion[0:31]
        #motion = motion[564:600]
    elif MOTION == TAEKWONDO2:
    ## Taekwondo base-step
        #motion = motion[0:31+40]
    ## Taekwondo turning-kick
        motion = motion[108:-1]
        #motion = motion[108:109]
    elif MOTION == KICK:
        #motion = motion[141:-1]
        #motion = motion[100:-1]
        #motion = motion[58:-1]
        motion = motion[82:-1]
        #motion = motion[0:-1]
    elif MOTION == STAND2:
        motion = motion[1:-1]
    elif MOTION == TIPTOE:
        #motion = motion[183:440]
        #motion = motion[350:410]
        motion = motion[350:550]

    motion[0:0] = [motion[0]]*40
    motion.extend([motion[-1]]*2000)
    
    # world, model
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = 1.

    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]
        
    node = mcfg.getNode(HIP)
    node.length = .2
    node.width = .25 
    
    node = mcfg.getNode(SPINE1)
    node.length = .2
    node.offset = (0,0,0.1)
    
    node = mcfg.getNode(SPINE)
    node.width = .22
    # node.length = .2 ####
    
        
    node = mcfg.getNode('RightFoot')
    node.length = .1
    node.width = .1
        
    node = mcfg.getNode('LeftFoot')
    node.length = .1
    node.width = .1
        

    #Talus
    #length1 = .12
    #width1 = 0.03
    #mass1 = 0.4

    #Phalange
    #length3 = length1
    #width3  = width1
    #mass3   = 0.4

    #length3_2 = length1*0.8
    #width3_2  = width1
    #mass3_2   = 0.4

    #Calcaneus1
    #length4 = 0.1
    #width4  = width1
    #mass4   = 0.4
        
    #Foot        
    length8 = .1
    width8 = 0.028*3
    mass8 = .4*1.5

    node = mcfg.getNode(RIGHT_FOOT)
    node.length = length8
    node.width = width8
    node.mass = mass8
                
    node = mcfg.getNode(RIGHT_TALUS_1)
    node.length = length1 - width1
    node.width = width1
    node.mass = mass1

    node = mcfg.getNode(RIGHT_TALUS_2)
    node.length = length1 - width1
    node.width = width1
    node.mass = mass1
      
    node = mcfg.getNode(RIGHT_PHALANGE_1)
    node.length = length3
    node.width = width3
    node.mass = mass3     

    node = mcfg.getNode(RIGHT_PHALANGE_2)
    node.length = length3_2
    node.width = width3_2
    node.mass = mass3_2 
        
    node = mcfg.getNode(RIGHT_CALCANEUS_1)
    node.length = length4 - width4
    node.width = width4
    node.mass = mass4

    node = mcfg.getNode(RIGHT_CALCANEUS_2)
    node.length = length4 - width4
    node.width = width4
    node.mass = mass4


    node = mcfg.getNode(LEFT_FOOT)
    node.length = length8
    node.width = width8
    node.mass = mass8
        
    node = mcfg.getNode(LEFT_TALUS_1)
    node.length = length1 - width1
    node.width = width1
    node.mass = mass1

    node = mcfg.getNode(LEFT_TALUS_2)
    node.length = length1 - width1
    node.width = width1
    node.mass = mass1

    node = mcfg.getNode(LEFT_PHALANGE_1)
    node.length = length3
    node.width = width3
    node.mass = mass3     
      
    node = mcfg.getNode(LEFT_PHALANGE_2)
    node.length = length3_2
    node.width = width3_2
    node.mass = mass3_2     
    
    node = mcfg.getNode(LEFT_CALCANEUS_1)
    node.length = length4# - width4
    node.width = width4
    node.mass = mass4
     
    node = mcfg.getNode(LEFT_CALCANEUS_2)
    node.length = length4# - width4
    node.width = width4
    node.mass = mass4
     
    #node.offset = (0.0, -0.025, 0.0)   
                   
    node = mcfg.getNode('LeftFoot')        

        
    node = mcfg.getNode(RIGHT_TALUS_1)
    node.geom = 'MyFoot3'
    node = mcfg.getNode(RIGHT_TALUS_2)        
    node.geom = 'MyFoot3'

    node = mcfg.getNode(LEFT_TALUS_1)
    node.geom = 'MyFoot3'
    node = mcfg.getNode(LEFT_TALUS_2)        
    node.geom = 'MyFoot3'

    node = mcfg.getNode(RIGHT_METATARSAL_1)
    node.geom = 'MyFoot3'
    node = mcfg.getNode(RIGHT_METATARSAL_2)
    node.geom = 'MyFoot3'
    node = mcfg.getNode(LEFT_METATARSAL_1)
    node.geom = 'MyFoot3'
    node = mcfg.getNode(LEFT_METATARSAL_2)        
    node.geom = 'MyFoot3'

    node = mcfg.getNode(RIGHT_PHALANGE_1) 
    node.geom = 'MyFoot4'
    node = mcfg.getNode(RIGHT_PHALANGE_2)
    node.geom = 'MyFoot4'
    node = mcfg.getNode(LEFT_PHALANGE_1)        
    node.geom = 'MyFoot4'
    node = mcfg.getNode(LEFT_PHALANGE_2)
    node.geom = 'MyFoot4'

    node = mcfg.getNode(RIGHT_CALCANEUS_1)
    node.geom = 'MyFoot4'
    node = mcfg.getNode(RIGHT_CALCANEUS_2)
    node.geom = 'MyFoot4'
    node = mcfg.getNode(LEFT_CALCANEUS_1)
    node.geom = 'MyFoot4'
    node = mcfg.getNode(LEFT_CALCANEUS_2)
    node.geom = 'MyFoot4'
        
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 120
    wcfg.timeStep = (1/30.)/(stepsPerFrame)
    #stepsPerFrame = 10
    #wcfg.timeStep = (1/120.)/(stepsPerFrame)
    #wcfg.timeStep = (1/1800.)
    
    # parameter
    config = {}
    config['Kt'] = 200;      config['Dt'] = 2*(config['Kt']**.5) # tracking gain
    config['Kl'] = .10;       config['Dl'] = 2*(config['Kl']**.5) # linear balance gain
    config['Kh'] = 0.1;       config['Dh'] = 2*(config['Kh']**.5) # angular balance gain
    config['Ks'] = 20000;   config['Ds'] = 2*(config['Ks']**.5) # penalty force spring gain
    config['Bt'] = 1.
    config['Bl'] = 1.#0.5
    config['Bh'] = 1.
    
    config['weightMap'] = {RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:.3, SPINE1:.2, RIGHT_FOOT:.3, LEFT_FOOT:.3, HIP:.3,
                    RIGHT_UP_LEG:.1, RIGHT_LEG:.2, LEFT_UP_LEG:.1, LEFT_LEG:.2, 
                    LEFT_TALUS_1:.1, RIGHT_TALUS_1:.1, LEFT_TALUS_2:.1, RIGHT_TALUS_2:.1, 
                    RIGHT_CALCANEUS_1:.2, LEFT_CALCANEUS_1:.2, RIGHT_CALCANEUS_2:.2, LEFT_CALCANEUS_2:.2, 
                    LEFT_PHALANGE_1:.1, LEFT_PHALANGE_2:.1, RIGHT_PHALANGE_1:.1, RIGHT_PHALANGE_2:.1}
    
    config['weightMap2'] = {RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:.5, SPINE1:.3, RIGHT_FOOT:.7, LEFT_FOOT:.7, HIP:.5,
                    RIGHT_UP_LEG:.7, RIGHT_LEG:.7, LEFT_UP_LEG:.7, LEFT_LEG:.7, 
                    LEFT_TALUS_1:.7, RIGHT_TALUS_1:.7, LEFT_TALUS_2:.7, RIGHT_TALUS_2:.7, 
                    RIGHT_CALCANEUS_1:.7, LEFT_CALCANEUS_1:.7, RIGHT_CALCANEUS_2:.7, LEFT_CALCANEUS_2:.7, 
                    LEFT_PHALANGE_1:.4, LEFT_PHALANGE_2:.4, RIGHT_PHALANGE_1:.4, RIGHT_PHALANGE_2:.4}
    '''                    
    (1, 'RightUpLeg')
    (2, 'RightLeg')
    (3, 'RightFoot')
    (4, 'Spine')
    (5, 'Spine1')
    (6, 'LeftArm')
    (7, 'LeftForeArm')
    (8, 'RightArm')
    (9, 'RightForeArm')
    (10, 'LeftUpLeg')
    (11, 'LeftLeg')
    (12, 'LeftFoot')
    (13, 'LeftTalus_1')
    (14, 'LeftTalus_2')
    (15, 'RightTalus_1')
    (16, 'RightTalus_2')
    (17, 'LeftPhalange_1')
    (18, 'LeftPhalange_2')
    (19, 'RightPhalange_1')
    (20, 'RightPhalange_2')
    (21, 'LeftCalcaneus_1')
    (22, 'LeftCalcaneus_2')
    (23, 'RightCalcaneus_1')
    (24, 'RightCalcaneus_2')
    '''
    config['trackWMap']={10, 5, .1, 20, 10, 5, 2, 10, 5, .1, 
                        0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.10, 0.01}

    config['supLink'] = LEFT_FOOT
    config['supLink2'] = RIGHT_FOOT
    # config['end'] = HIP
    config['end'] = SPINE1

    config['trunk'] = SPINE
    config['const'] = HIP
    config['root'] = HIP
    
    config['FootPartNum'] = FOOT_PART_NUM
    
    config['FootLPart'] = [LEFT_FOOT, LEFT_CALCANEUS_1, LEFT_CALCANEUS_2, LEFT_TALUS_1, LEFT_TALUS_2, LEFT_PHALANGE_1, LEFT_PHALANGE_2]
    config['FootRPart'] = [RIGHT_FOOT, RIGHT_CALCANEUS_1, RIGHT_CALCANEUS_2, RIGHT_TALUS_1, RIGHT_TALUS_2, RIGHT_PHALANGE_1, RIGHT_PHALANGE_2]

    return motion, mcfg, wcfg, stepsPerFrame, config

# ===============================================================================
# biped config
# ===============================================================================

# motion, mesh config
g_motionDirConfigMap = dict()
g_motionDirConfigMap['../Data/woody2/Motion/Physics2/'] = \
    {'footRot': mm.exp(mm.v3(1,0,0), .05), 'yOffset': .0, 'scale': 1.,
     'rootRot': mm.I_SO3()}
g_motionDirConfigMap['../Data/woody2/Motion/Balancing/'] = \
    {'footRot': mm.exp(mm.v3(1,-.5,0), -.6), 'yOffset': .0, 'scale': 1.,
     'rootRot': mm.exp(mm.v3(1,0,0), .01)}
g_motionDirConfigMap['../Data/woody2/Motion/VideoMotion/'] = \
    {'footRot': mm.exp(mm.v3(1,0,0), -.05), 'yOffset': .01, 'scale': 2.53999905501,
     'rootRot': mm.exp(mm.v3(1,0,0), .0)}
g_motionDirConfigMap['../Data/woody2/Motion/Samsung/'] = \
    {'footRot': mm.exp(mm.v3(1,0,0), -.03), 'yOffset': .0, 'scale': 2.53999905501,
     'rootRot': mm.exp(mm.v3(1,0,0), .03)}


#===============================================================================
# # reloadable config
#===============================================================================
def buildMassMap():
    massMap = dict()
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1',
                                    LEFT_METATARSAL_1, LEFT_METATARSAL_3, LEFT_METATARSAL_2, LEFT_PHALANGE_1, LEFT_PHALANGE_2, LEFT_PHALANGE_3, LEFT_CALCANEUS_1, LEFT_CALCANEUS_2, LEFT_CALCANEUS_3, 
                                    LEFT_TALUS_1, LEFT_TALUS_2, LEFT_TALUS_3, RIGHT_TALUS_1, RIGHT_TALUS_2, RIGHT_TALUS_3, 
                                    RIGHT_METATARSAL_1, RIGHT_METATARSAL_3, RIGHT_METATARSAL_2, RIGHT_PHALANGE_1, RIGHT_PHALANGE_2, RIGHT_PHALANGE_3, RIGHT_CALCANEUS_1, RIGHT_CALCANEUS_2, RIGHT_CALCANEUS_3], 0.)
        
    # torso : 10
    massMap[HIP] += 2.
    #massMap[SPINE] += 8.
    massMap[SPINE] += 8.
    
    # head : 3
    massMap[SPINE1] += 3.
    
    # right upper arm : 2
    massMap[RIGHT_ARM] += 2.
    
    # left upper arm : 2
    massMap[LEFT_ARM] += 2.
    
    # right lower arm : 1
    #massMap[RIGHT_FORE_ARM] = 1.
    massMap[RIGHT_FORE_ARM] = 2.
    
    # left lower arm : 1
    #massMap[LEFT_FORE_ARM] = 1.
    massMap[LEFT_FORE_ARM] = 2.
    
    # right thigh : 7
    massMap[HIP] += 2.
    massMap[RIGHT_UP_LEG] += 5.
    
    # left thigh : 7
    massMap[HIP] += 2.
    massMap[LEFT_UP_LEG] += 5.
    
    # right shin : 5
    massMap[RIGHT_LEG] += 5.
    
    # left shin : 5
    massMap[LEFT_LEG] += 5.
    
    # right foot : 4
    massMap[RIGHT_FOOT] += 1.
    
    # left foot : 4
    massMap[LEFT_FOOT] += 1. 
       
                
    return massMap
massMap = buildMassMap()
