import copy

import sys

import PyCommon.modules.Resource.ysMotionLoader as yf
import PyCommon.modules.Simulator.ysPhysConfig as ypc
import PyCommon.modules.Math.mmMath as mm
import PyCommon.modules.Motion.ysHierarchyEdit as yme
import PyCommon.modules.Motion.ysMotion as ym

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
    #motionName = 'wd2_jump_ori.bvh'
    if 1:
        #motionName = 'wd2_stand.bvh'        
        motionName = 'woddy2_jump0.bvh'
        motion = yf.readBvhFile(motionName, .01)

        motion.translateByOffset((0., -0.023, 0.))
   
        yme.removeJoint(motion, 'HEad', False)
        yme.removeJoint(motion, 'RightShoulder', False)
        yme.removeJoint(motion, 'LeftShoulder1', False)
        yme.removeJoint(motion, 'RightToes_Effector', False)
        yme.removeJoint(motion, 'LeftToes_Effector', False)
        yme.removeJoint(motion, 'RightHand_Effector', False)
        yme.removeJoint(motion, 'LeftHand_Effector', False)
        yme.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
        yme.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)
        yme.rotateJointLocal(motion, 'Hips', mm.exp(mm.v3(1,0,0), .01), False)
        yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(2.5,-0.0,.3), -.5), False)
        yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(2.5,0.0,-.3), -.5), False)
        #yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.0,.2), -.5), False)
        #yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.0,-.2), -.5), False)
        
        yme.rotateJointLocal(motion, 'LeftUpLeg', mm.exp(mm.v3(0.0,.0,1.), .08), False)
        yme.rotateJointLocal(motion, 'LeftLeg', mm.exp(mm.v3(0.0,1.0,0.), -.2), False)
        
        #yme.rotateJointLocal(motion, 'RightLeg', mm.exp(mm.v3(1.0,0.0,0.), -.1), False)
        yme.updateGlobalT(motion)

        # yf.writeBvhFile('wd2_jump0.bvh', motion)
    else :        
        motionName = 'ww13_41.bvh'
        motion = yf.readBvhFile(motionName, 0.056444)
        yme.removeJoint(motion, 'LHipJoint', False)
        yme.removeJoint(motion, 'RHipJoint', False)
        yme.removeJoint(motion, 'Neck', False)
        yme.removeJoint(motion, 'Neck1', False)
        yme.removeJoint(motion, 'Head', False)
        yme.removeJoint(motion, 'RightShoulder', False)
        yme.removeJoint(motion, 'LeftShoulder', False)
        yme.removeJoint(motion, 'RightToeBase_Effector', False)
        yme.removeJoint(motion, 'LeftToeBase_Effector', False)
        yme.removeJoint(motion, 'LeftHand', False) 
        yme.removeJoint(motion, 'LeftFingerBase', False)
        yme.removeJoint(motion, 'LeftHandIndex1_Effector', False)
        yme.removeJoint(motion, 'LThumb', False)    
        yme.removeJoint(motion, 'RightHand', False)
        yme.removeJoint(motion, 'RightFingerBase', False)
        yme.removeJoint(motion, 'RightHandIndex1_Effector', False)
        yme.removeJoint(motion, 'RThumb', False)   
        yme.removeJoint(motion, 'LowerBack', False)

        yme.offsetJointLocal(motion, 'RightArm', (-.03,-.05,0), False)
        yme.offsetJointLocal(motion, 'LeftArm', (.03,-.05,0), False)
        #yme.rotateJointLocal(motion, 'Hips', mm.exp(mm.v3(1,0,0), .01), False)
        yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(-1.5,0,1), .4), False)
        yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1.5,0,1), -.4), False)
        yme.updateGlobalT(motion)
        #motion = motion[50:-1]     
        motion = motion[240:-1]

    
    #motion = motion[40:-58]
    #motion = motion[56:-248]
    #motion = motion[-249:-248]
    
    #motion = motion[62:65]
    #motion = motion[216:217]
    #motion = motion[515:555]

    ################
    motion = motion[515:555]
    # motion = motion[164:280]

    #motion = motion[96:97]
    motion[0:0] = [motion[0]]*100
    motion.extend([motion[-1]]*5000)
    
    # world, model
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
    node.offset = (0,0,0.1)
    
    node = mcfg.getNode('Spine')
    node.width = .22
    #node.length = .2 ####
    
    node = mcfg.getNode('RightFoot')
    node.length = .25
    node.width = .2
    node.mass = 4.
    
    node = mcfg.getNode('LeftFoot')
    node.length = .25
    node.width = .2
    node.mass = 4.
    
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
    config['Ks'] = 20000;   config['Ds'] = 2*(config['Ks']**.5) # penalty force spring gain
    config['Bt'] = 1.
    config['Bl'] = 1.#0.5
    config['Bh'] = 1.

    config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
                         'Spine':.3, 'Spine1':.3, 'RightFoot':.3, 'LeftFoot':.3, 'Hips':.5,\
                         'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.1, 'LeftLeg':.3}
    
    config['IKweightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
                         'Spine':.3, 'Spine1':.3, 'RightFoot':.3, 'LeftFoot':.3, 'Hips':.5,\
                         'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.1, 'LeftLeg':.3}

    '''
    config['IKweightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
                         'Spine':0.5, 'Spine1':0.5, 'RightFoot':1.2, 'LeftFoot':1.2, 'Hips':1.2,\
                         'RightUpLeg':.9, 'RightLeg':.9, 'LeftUpLeg':.9, 'LeftLeg':.9}
    '''
    
    config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
                         'Spine':1.5, 'LeftFoot':1., 'Hips':1.5,\
                         'RightUpLeg':1., 'RightLeg':1., 'LeftUpLeg':1.5, 'LeftLeg':1.5}
        
    config['supLink'] = 'LeftFoot'
    config['supLink2'] = 'RightFoot'
    #config['end'] = 'Hips'    
    config['end'] = 'Spine1'

        
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
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips', 'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector', 'LeftLeg', 'LeftShoulder1', 'LeftToes', 'LeftToes_Effector', 'LeftUpLeg', 'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector', 'RightLeg', 'RightShoulder', 'RightToes', 'RightToes_Effector', 'RightUpLeg', 'Spine', 'Spine1'], 0.)
    
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
    #massMap['RightForeArm'] = 1.
    massMap['RightForeArm'] = 2.
    
    # left lower arm : 1
    #massMap['LeftForeArm'] = 1.
    massMap['LeftForeArm'] = 2.
    
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
    
    return massMap
massMap = buildMassMap()
