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

## Constant
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

LEFT_PHALANGE = 'LeftForeFoot'
RIGHT_PHALANGE = 'RightForeFoot'

LEFT_TARSUS = 'LeftRearFoot'
RIGHT_TARSUS = 'RightRearFoot'

LEFT_METATARSUS = 'LeftMidFoot'
RIGHT_METATARSUS = 'RightMidFoot'

LEFT_FOOT_SIDE_L = 'LeftFootSideL'
LEFT_FOOT_SIDE_R = 'LeftFootSideR'
RIGHT_FOOT_SIDE_L = 'RightFootSideL'
RIGHT_FOOT_SIDE_R = 'RightFootSideR'

'''
HIP = 'hip'
RIGHT_UP_LEG_DUMMY = 'rightuplegdummy'
RIGHT_UP_LEG = 'rightupleg'
RIGHT_LEG = 'rightleg'
RIGHT_FOOT = 'rightfoot'
RIGHT_TOES = 'righttoes'
RIGHT_TOES_END = 'righttoes_Effector'
LEFT_UP_LEG_DUMMY = 'leftuplegdummy'
LEFT_UP_LEG = 'leftupleg'
LEFT_LEG = 'leftleg'
LEFT_FOOT = 'leftfoot'
LEFT_TOES = 'lefttoes'
LEFT_TOES_END = 'lefttoes_Effector'

LEFT_SHOULDER_DUMMY = 'leftshoulder1dummy'
LEFT_SHOULDER = 'leftshoulder1'
LEFT_ARM = 'leftarm'
LEFT_FORE_ARM = 'leftforearm'
LEFT_HAND = 'lefthand'
LEFT_HAND_END = 'lefthand_Effector'
RIGHT_SHOULDER_DUMMY = 'rightshoulderdummy'
RIGHT_SHOULDER = 'rightshoulder'
RIGHT_ARM = 'rightarm'
RIGHT_FORE_ARM = 'rightforearm'
RIGHT_HAND = 'righthand'
RIGHT_HAND_END = 'righthand_Effector'

SPINE_DUMMY = 'spinedummy'
SPINE = 'spine'
SPINE1 = 'spine1'
HEAD_DUMMY = 'headdummy'
HEAD = 'head'
HEAD_END = 'head_Effector'
'''

STAND = 0
FORWARD_JUMP = 1
TAEKWONDO = 2

## Motion File
#MOTION = STAND
#MOTION = FORWARD_JUMP
MOTION = TAEKWONDO

FOOT_PART_NUM = 3

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
    stepsPerFrame = 60
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

def create_biped():            
    # motion
    #motionName = 'wd2_n_kick.bvh'  
    
    if MOTION == STAND:
        motionName = 'wd2_stand.bvh'
    elif MOTION == FORWARD_JUMP:
        motionName = 'woddy2_jump0.bvh'       
    elif MOTION == TAEKWONDO  :
        motionName = './MotionFile/wd2_098_V001.bvh'

    #motionName = 'ww13_41_V001.bvh'
    motion = yf.readBvhFile(motionName, .01)
   
    yme.removeJoint(motion, HEAD, False)
    yme.removeJoint(motion, RIGHT_SHOULDER, False)
    yme.removeJoint(motion, LEFT_SHOULDER, False)
    
    if FOOT_PART_NUM == 1 :
        yme.removeJoint(motion, RIGHT_TOES_END, False)
        yme.removeJoint(motion, LEFT_TOES_END, False)

    yme.removeJoint(motion, RIGHT_HAND_END, False)
    yme.removeJoint(motion, LEFT_HAND_END, False)
        
    yme.offsetJointLocal(motion, RIGHT_ARM, (.03,-.05,0), False)
    yme.offsetJointLocal(motion, LEFT_ARM, (-.03,-.05,0), False)
    yme.rotateJointLocal(motion, HIP, mm.exp(mm.v3(1,0,0), .01), False)
    
    yme.rotateJointLocal(motion, LEFT_FOOT, mm.exp(mm.v3(2.5,-0.0,.3), -.5), False)
    yme.rotateJointLocal(motion, RIGHT_FOOT, mm.exp(mm.v3(2.5,0.0,-.3), -.5), False)    
                    
    if MOTION == FORWARD_JUMP:
        yme.rotateJointLocal(motion, LEFT_UP_LEG, mm.exp(mm.v3(0.0,.0,1.), .08), False)
        yme.rotateJointLocal(motion, LEFT_LEG, mm.exp(mm.v3(0.0,1.0,0.), -.2), False)
        
    if FOOT_PART_NUM > 1:
        yme.addJoint(motion, RIGHT_FOOT, RIGHT_TARSUS)
        yme.addJoint(motion, RIGHT_TARSUS, 'RIGHT_Dummy1')
        yme.addJoint(motion, LEFT_FOOT, LEFT_TARSUS)
        yme.addJoint(motion, LEFT_TARSUS, 'LEFT_Dummy1')
        yme.rotateJointLocal(motion, LEFT_TOES, mm.exp(mm.v3(1.,0.0,0.0), .45), False)
        yme.rotateJointLocal(motion, RIGHT_TOES, mm.exp(mm.v3(1.,0.0,0.0), .45), False)
        yme.rotateJointLocal(motion, LEFT_TARSUS, mm.exp(mm.v3(1.,0.0,0.0), .52), False)
        yme.rotateJointLocal(motion, RIGHT_TARSUS, mm.exp(mm.v3(1.,0.0,0.0), .52), False)
    
    if FOOT_PART_NUM == 5 :
        yme.addJoint(motion, LEFT_FOOT, LEFT_FOOT_SIDE_L)
        yme.addJoint(motion, LEFT_FOOT_SIDE_L, 'LEFT_Dummy2')
        yme.addJoint(motion, LEFT_FOOT, LEFT_FOOT_SIDE_R)
        yme.addJoint(motion, LEFT_FOOT_SIDE_R, 'LEFT_Dummy2')
        yme.addJoint(motion, RIGHT_FOOT, RIGHT_FOOT_SIDE_L)
        yme.addJoint(motion, RIGHT_FOOT_SIDE_L, 'RIGHT_Dummy2')
        yme.addJoint(motion, RIGHT_FOOT, RIGHT_FOOT_SIDE_R)
        yme.addJoint(motion, RIGHT_FOOT_SIDE_R, 'RIGHT_Dummy2')
        yme.rotateJointLocal(motion, LEFT_FOOT_SIDE_L, mm.exp(mm.v3(1.,0.0,0.0), .45), False)
        yme.rotateJointLocal(motion, LEFT_FOOT_SIDE_R, mm.exp(mm.v3(1.,0.0,0.0), .45), False)
        yme.rotateJointLocal(motion, RIGHT_FOOT_SIDE_L, mm.exp(mm.v3(1.,0.0,0.0), .45), False)
        yme.rotateJointLocal(motion, RIGHT_FOOT_SIDE_R, mm.exp(mm.v3(1.,0.0,0.0), .45), False)

    yme.updateGlobalT(motion)
    

    ################
    if MOTION == FORWARD_JUMP:        
        motion = motion[515:555]
    elif MOTION == TAEKWONDO:
    ## Taekwondo base-step
        motion = motion[0:31]
        #motion = motion[564:600]
    ## Taekwondo turning-kick
        #motion = motion[108:-1]
        #motion = motion[108:109]

    motion[0:0] = [motion[0]]*100
    motion.extend([motion[-1]]*5000)
    
    # world, model
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .9

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
    #node.length = .2 ####
    
    if FOOT_PART_NUM == 1 :   
        length1 = .25
        width1 = .2
        mass1 = 4.
    elif FOOT_PART_NUM == 3:        
        length1 = .1
        width1 = .2
        mass1 = 1.5
        length2 = .1
        width2 = .2
        mass2 = 1.5
    elif FOOT_PART_NUM == 5:
        length1 = .1
        width1 = .065
        mass1 = .5
        length2 = .1
        width2 = .2
        mass2 = 1.5

    node = mcfg.getNode(RIGHT_FOOT)
    node.length = length1
    node.width = width1
    node.mass = mass1

    node = mcfg.getNode(LEFT_FOOT)
    node.length = length1
    node.width = width1
    node.mass = mass1
       
    if FOOT_PART_NUM == 5:
        node = mcfg.getNode(LEFT_FOOT_SIDE_L)
        node.length = length1
        node.width = width1
        node.mass = mass1
        node.offset = (0.07,0.0,0.015)
        node = mcfg.getNode(LEFT_FOOT_SIDE_R)
        node.length = length1
        node.width = width1
        node.mass = mass1
        node.offset = (-0.07,0.0,0.015)
            
        node = mcfg.getNode(RIGHT_FOOT_SIDE_L)
        node.length = length1
        node.width = width1
        node.mass = mass1
        node.offset = (0.07,0.0,0.015)
        node = mcfg.getNode(RIGHT_FOOT_SIDE_R)
        node.length = length1
        node.width = width1
        node.mass = mass1
        node.offset = (-0.07,0.0,0.015)

    if FOOT_PART_NUM > 1:
        node = mcfg.getNode(LEFT_TOES)
        node.length = length2
        node.width = width2
        node.mass = mass2
        node.offset = (0,0.0,-0.02)
        
        node = mcfg.getNode(RIGHT_TOES)
        node.length = length2
        node.width = width2
        node.mass = mass2
        node.offset = (0,0.0,-0.02)            
        
        node = mcfg.getNode(LEFT_TARSUS)
        node.length = length2
        node.width = width2
        node.mass = mass2
        node.offset = (0,0.0,-0.08)
        
        node = mcfg.getNode(RIGHT_TARSUS)
        node.length = length2
        node.width = width2
        node.mass = mass2
        node.offset = (0,0.0,-0.08)
        
    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    stepsPerFrame = 30
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
    
    
    if FOOT_PART_NUM == 1:
        config['weightMap']={RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:.3, SPINE1:.3, RIGHT_FOOT:.3, LEFT_FOOT:.3, HIP:.5,
                        RIGHT_UP_LEG:.1, RIGHT_LEG:.3, LEFT_UP_LEG:.1, LEFT_LEG:.3}
        config['weightMap2']={RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:1., SPINE1:.3, RIGHT_FOOT:1., LEFT_FOOT:1., HIP:1.,
                        RIGHT_UP_LEG:1., RIGHT_LEG:1., LEFT_UP_LEG:1., LEFT_LEG:1.}
    elif FOOT_PART_NUM == 3:
        config['weightMap']={RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:.3, SPINE1:.3, RIGHT_FOOT:.3, LEFT_FOOT:.3, HIP:.5,
                        RIGHT_UP_LEG:.3, RIGHT_LEG:.3, LEFT_UP_LEG:.3, LEFT_LEG:.3, LEFT_TOES:.3, RIGHT_TOES:.3}
        config['weightMap2']={RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:1., SPINE1:.3, RIGHT_FOOT:2.5, LEFT_FOOT:2.5, HIP:1.,
                        RIGHT_UP_LEG:1., RIGHT_LEG:1., LEFT_UP_LEG:1., LEFT_LEG:1., LEFT_TOES:.3, RIGHT_TOES:.3}
    elif FOOT_PART_NUM == 5:
        config['weightMap']={RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:.3, SPINE1:.3, RIGHT_FOOT:.3, LEFT_FOOT:.3, HIP:.5,
                        RIGHT_UP_LEG:.1, RIGHT_LEG:.3, LEFT_UP_LEG:.1, LEFT_LEG:.3, LEFT_TOES:.3, RIGHT_TOES:.3, LEFT_TARSUS:.3, RIGHT_TARSUS:.3,
                        LEFT_FOOT_SIDE_L:.3, LEFT_FOOT_SIDE_R:.3, RIGHT_FOOT_SIDE_L:.3, RIGHT_FOOT_SIDE_R:.3}
        config['weightMap2']={RIGHT_ARM:.2, RIGHT_FORE_ARM:.2, LEFT_ARM:.2, LEFT_FORE_ARM:.2, SPINE:1., SPINE1:.3, RIGHT_FOOT:2.5, LEFT_FOOT:2.5, HIP:1.,
                        RIGHT_UP_LEG:1., RIGHT_LEG:1., LEFT_UP_LEG:1., LEFT_LEG:1., LEFT_TOES:.3, RIGHT_TOES:.3, LEFT_TARSUS:.3, RIGHT_TARSUS:.3,
                        LEFT_FOOT_SIDE_L:.3, LEFT_FOOT_SIDE_R:.3, RIGHT_FOOT_SIDE_L:.3, RIGHT_FOOT_SIDE_R:.3}
                    
    config['supLink'] = LEFT_FOOT
    config['supLink2'] = RIGHT_FOOT
    #config['end'] = 'HIP'    
    config['end'] = SPINE1
    config['const'] = HIP
    config['root'] = HIP
    
    config['FootPartNum'] = FOOT_PART_NUM
    config['FootLPart'] = [LEFT_FOOT, LEFT_TOES, LEFT_TARSUS, LEFT_FOOT_SIDE_L, LEFT_FOOT_SIDE_R ]
    config['FootRPart'] = [RIGHT_FOOT, RIGHT_TOES, RIGHT_TARSUS, RIGHT_FOOT_SIDE_L, RIGHT_FOOT_SIDE_R]
        
    return motion, mcfg, wcfg, stepsPerFrame, config



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
    massMap = massMap.fromkeys([HEAD, HEAD_END, HIP, LEFT_ARM, LEFT_FOOT, LEFT_FORE_ARM, LEFT_HAND, LEFT_HAND_END, LEFT_LEG, LEFT_SHOULDER, LEFT_TOES, LEFT_TOES_END, LEFT_UP_LEG, RIGHT_ARM, RIGHT_FOOT, RIGHT_FORE_ARM, RIGHT_HAND, RIGHT_HAND_END, RIGHT_LEG, RIGHT_SHOULDER, RIGHT_TOES, RIGHT_TOES_END, RIGHT_UP_LEG, SPINE, SPINE1, LEFT_PHALANGE, RIGHT_PHALANGE, LEFT_TARSUS, RIGHT_TARSUS
                                , LEFT_FOOT_SIDE_L, LEFT_FOOT_SIDE_R, RIGHT_FOOT_SIDE_L, RIGHT_FOOT_SIDE_R], 0.)
    
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
    massMap[RIGHT_FOOT] += 2.
    
    # left foot : 4
    massMap[LEFT_FOOT] += 2.
    
    massMap[LEFT_TOES] += 2.
    massMap[RIGHT_TOES] += 2.
    
    massMap[LEFT_TARSUS] += 2.
    massMap[RIGHT_TARSUS] += 2.
                
    return massMap
massMap = buildMassMap()
