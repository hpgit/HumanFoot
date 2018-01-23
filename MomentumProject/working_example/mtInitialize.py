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

def create_vchain_5():
	# motion
	motion = yf.readBvhFile('vchain_5_rotate_root0.bvh', 1)
	
	# world, model
	mcfg = ypc.ModelConfig()
	mcfg.defaultDensity = 1000.
	mcfg.defaultBoneRatio = .8
	for i in range(motion[0].skeleton.getElementNum()):
		mcfg.addNode(motion[0].skeleton.getElementName(i))
		
	#node = mcfg.getNode('link0')
	node = mcfg.getNode('Hips')
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
	#config['supLink'] = 'link0'
	config['supLink'] = 'Hips'
	
	return motion, mcfg, wcfg, stepsPerFrame, config

def create_jump_biped():
		
	# motion
	#motionName = 'wd2_n_kick.bvh'
	#motionName = 'wd2_jump.bvh'
	#motionName = 'wd2_stand.bvh'
	motionName = 'woddy2_jump_ori.bvh'
	motion = yf.readBvhFile(motionName, .01)
	#yme.removeJoint(motion, 'Head', False)
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
	#yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.0,.3), -.5), False)
	#yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.0,-.3), -.5), False)
	yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.5,0), -.6), False)
	yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.5,0), -.6), False)
  
	yme.updateGlobalT(motion)
	motion.translateByOffset((0, -0.05, 0))

	#motion = motion[130:]
	#motion = motion[120:]
	motion = motion[67:162]

	#motion = motion[-249:-248]
	motion[0:0] = [motion[0]]*200
	motion.extend([motion[-1]]*500)
	
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
	node.width = .25
	
	node = mcfg.getNode('RightFoot')
	node.length = .25
	node.width = .15
	node.mass = 2.
	#node.width = .2
	#node.mass = 1.
	
	node = mcfg.getNode('LeftFoot')
	node.length = .25
	node.width = .15
	node.mass = 2.
	
	wcfg = ypc.WorldConfig()
	wcfg.planeHeight = 0.
	wcfg.useDefaultContactModel = False
	stepsPerFrame = 60
	#stepsPerFrame = 30
	wcfg.timeStep = (1/30.)/(stepsPerFrame)
	#wcfg.timeStep = (1/1000.)
	
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
						 'Spine':1., 'Spine1':1., 'RightFoot':.5, 'LeftFoot':.5, 'Hips':1.,\
						 'RightUpLeg':.5, 'RightLeg':.5, 'LeftUpLeg':.5, 'LeftLeg':.5}
	
	#config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 #'Spine':1., 'Spine1':1., 'RightFoot':1.0, 'LeftFoot':1.0, 'Hips':1.5,\
						 #'RightUpLeg':2., 'RightLeg':2., 'LeftUpLeg':2., 'LeftLeg':2.}
	#config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 #'Spine':.6, 'Spine1':.6, 'RightFoot':.2, 'LeftFoot':.2, 'Hips':1.,\
						 #'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.1, 'LeftLeg':.3}
	
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

		
	return motion, mcfg, wcfg, stepsPerFrame, config

def create_biped():
	#motion
	motionName = 'wd2_n_kick.bvh'
	#motionName = 'wd2_jump.bvh'
	#motionName = 'wd2_stand.bvh'
	motion = yf.readBvhFile(motionName, .01)
	# yme.removeJoint(motion, 'Head', False)
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
	#yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.0,.3), -.5), False)
	#yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.0,-.3), -.5), False)
	yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.5,0), -.6), False)
	yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.5,0), -.6), False)
  
	yme.updateGlobalT(motion)
	#motion.translateByOffset((0, -0.07, 0))
	
	# motion = motion[40:-58]
	#motion[0:0] = [motion[0]]*20
	#motion.extend([motion[-1]]*5000)

	# motion = motion[40:]
	#motion[0:0] = [motion[0]]*50
	#motion.extend([motion[-1]]*5000)

	#motion = motion[30:151]
	motion = motion[30:]
	# motion = motion[30:31]
	#motion[5:5] = [motion[5]]*30
	motion[0:0] = [motion[0]]*30
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
	
	wcfg = ypc.WorldConfig()
	wcfg.planeHeight = 0.
	wcfg.useDefaultContactModel = False
	stepsPerFrame = 60
	#stepsPerFrame = 30
	wcfg.timeStep = (1/30.)/(stepsPerFrame)
	#wcfg.timeStep = (1/1000.)
	
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
	config['Kt'] = 200;       config['Dt'] = 2.*(config['Kt']**.5) # tracking gain
	config['Kl'] = .10;       config['Dl'] = 2.*(config['Kl']**.5) # linear balance gain
	config['Kh'] = 0.1;       config['Dh'] = 2.*(config['Kh']**.5) # angular balance gain
	config['Ks'] = 20000;     config['Ds'] = 2.*(config['Ks']**.5) # penalty force spring gain
	config['Bt'] = 1.
	config['Bl'] = 1.#0.5
	config['Bh'] = 1.
	#config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 #'Spine':1., 'Spine1':1., 'RightFoot':.5, 'LeftFoot':.5, 'Hips':1.5,\
						 #'RightUpLeg':1., 'RightLeg':1., 'LeftUpLeg':1., 'LeftLeg':1.}
	
	#config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 #'Spine':1., 'Spine1':1., 'RightFoot':1.0, 'LeftFoot':1.0, 'Hips':1.5,\
						 #'RightUpLeg':2., 'RightLeg':2., 'LeftUpLeg':2., 'LeftLeg':2.}
	config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 'Spine':.6, 'Spine1':.6, 'RightFoot':.2, 'LeftFoot':.2, 'Hips':0.5,\
						 'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.1, 'LeftLeg':.3}

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

		
	return motion, mcfg, wcfg, stepsPerFrame, config


def create_walking_biped():
	#motion
	motionName = 'wd2_WalkForwardNormal00.bvh'
	#motionName = '../motions/wd2_WalkForwardNormal00.bvh'
	#motionName = '../motions/wd2_WalkForwardFast00.bvh'
	#motionName = 'wd2_jump.bvh'
	#motionName = 'wd2_stand.bvh'
	motion = yf.readBvhFile(motionName, .01)
	yme.removeJoint(motion, 'Head', False)
	#yme.removeJoint(motion, 'HEad', False)
	yme.removeJoint(motion, 'RightShoulder', False)
	yme.removeJoint(motion, 'LeftShoulder1', False)
	yme.removeJoint(motion, 'RightToes_Effector', False)
	yme.removeJoint(motion, 'LeftToes_Effector', False)
	yme.removeJoint(motion, 'RightHand_Effector', False)
	yme.removeJoint(motion, 'LeftHand_Effector', False)
	yme.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
	yme.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)
	yme.rotateJointLocal(motion, 'Hips', mm.exp(mm.v3(1,0,0), .01), False)
	#yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.0,.3), -.5), False)
	#yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.0,-.3), -.5), False)
	yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.5,0), -.45), False)
	yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.5,0), -.45), False)
  
	yme.updateGlobalT(motion)
	motion.translateByOffset((0, 0.0, 0))
	
	#motion = motion[40:-58]
	#motion[0:0] = [motion[0]]*20
	#motion.extend([motion[-1]]*5000)

	motion = motion[40:]
	#motion[0:0] = [motion[0]]*50
	#motion.extend([motion[-1]]*100)
	#motion.extend([motion[-1]]*100)

	#motion = motion[30:151]
	#motion = motion[30:]
	#motion[5:5] = [motion[5]]*30
	#motion[0:0] = [motion[0]]*100
	#motion.extend([motion[-1]]*5000)

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
	node.width = .12
	#node.width = .2
	node.mass = 2.
	#node.mass = 1.
	
	node = mcfg.getNode('LeftFoot')
	node.length = .25
	#node.length = .2
	node.width = .12
	#node.width = .2
	node.mass = 2.
	
	wcfg = ypc.WorldConfig()
	wcfg.planeHeight = 0.
	wcfg.useDefaultContactModel = False
	stepsPerFrame = 60
	#stepsPerFrame = 30
	wcfg.timeStep = (1/30.)/(stepsPerFrame)
	#wcfg.timeStep = (1/1000.)
	
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
	#config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 #'Spine':.6, 'Spine1':.6, 'RightFoot':.2, 'LeftFoot':.2, 'Hips':0.5,\
						 #'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.1, 'LeftLeg':.3}
	#config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 #'Spine':1., 'Spine1':1., 'RightFoot':.5, 'LeftFoot':.5, 'Hips':1.5,\
						 #'RightUpLeg':1., 'RightLeg':1., 'LeftUpLeg':1., 'LeftLeg':1.}
	
	#config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 #'Spine':1., 'Spine1':1., 'RightFoot':1.0, 'LeftFoot':1.0, 'Hips':1.5,\
						 #'RightUpLeg':2., 'RightLeg':2., 'LeftUpLeg':2., 'LeftLeg':2.}
	config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 'Spine':.6, 'Spine1':.6, 'RightFoot':.2, 'LeftFoot':.2, 'Hips':0.5,\
						 'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.1, 'LeftLeg':.3}
	
	#success!!
	#config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 #'Spine':.5, 'Spine1':.5, 'RightFoot':1., 'LeftFoot':1., 'Hips':0.5,\
						 #'RightUpLeg':1., 'RightLeg':1., 'LeftUpLeg':1., 'LeftLeg':1.}

	#config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 #'Spine':1.5, 'LeftFoot':1., 'Hips':1.5,\
						 #'RightUpLeg':1., 'RightLeg':1., 'LeftUpLeg':1.5, 'LeftLeg':1.5}
	
	config['supLink'] = 'LeftFoot'
	config['supLink1'] = 'LeftFoot'
	config['supLink2'] = 'RightFoot'
	#config['end'] = 'Hips'    
	config['end'] = 'Spine1'

		
	return motion, mcfg, wcfg, stepsPerFrame, config

def create_any_motion(motion):
	#motion
	yme.removeJoint(motion, 'Head', False)
	#yme.removeJoint(motion, 'HEad', False)
	yme.removeJoint(motion, 'RightShoulder', False)
	yme.removeJoint(motion, 'LeftShoulder1', False)
	yme.removeJoint(motion, 'RightToes_Effector', False)
	yme.removeJoint(motion, 'LeftToes_Effector', False)
	yme.removeJoint(motion, 'RightHand_Effector', False)
	yme.removeJoint(motion, 'LeftHand_Effector', False)
	yme.offsetJointLocal(motion, 'RightArm', (.03,-.05,0), False)
	yme.offsetJointLocal(motion, 'LeftArm', (-.03,-.05,0), False)
	yme.rotateJointLocal(motion, 'Hips', mm.exp(mm.v3(1,0,0), .01), False)
	#yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.0,.3), -.5), False)
	#yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.0,-.3), -.5), False)
	yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(1,-0.5,0), -.45), False)
	yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(1,0.5,0), -.45), False)
  
	yme.updateGlobalT(motion)
	
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
	
	node = mcfg.getNode('RightFoot')
	node.length = .25
	#node.length = .2
	#node.width = .12
	#node.width = .2
	node.width = .15
	node.mass = 2.
	#node.mass = 1.
	
	node = mcfg.getNode('LeftFoot')
	node.length = .25
	#node.length = .2
	#node.width = .12
	node.width = .15
	#node.width = .2
	node.mass = 2.
	
	wcfg = ypc.WorldConfig()
	wcfg.planeHeight = 0.
	wcfg.useDefaultContactModel = False
	stepsPerFrame = 60
	wcfg.timeStep = (1/30.)/(stepsPerFrame)
	
	# parameter
	config = {}
	config['Kt'] = 200;      config['Dt'] = 2*(config['Kt']**.5) # tracking gain
	config['Kl'] = .10;       config['Dl'] = 2*(config['Kl']**.5) # linear balance gain
	config['Kh'] = 0.1;       config['Dh'] = 2*(config['Kh']**.5) # angular balance gain
	config['Ks'] = 20000;   config['Ds'] = 2*(config['Ks']**.5) # penalty force spring gain
	config['Bt'] = 1.
	config['Bl'] = 1.#0.5
	config['Bh'] = 1.

	config['weightMap']={'RightArm':.2, 'RightForeArm':.2, 'LeftArm':.2, 'LeftForeArm':.2,\
						 'Spine':.6, 'Spine1':.6, 'RightFoot':.2, 'LeftFoot':.2, 'Hips':0.5,\
						 'RightUpLeg':.1, 'RightLeg':.3, 'LeftUpLeg':.1, 'LeftLeg':.3}
		
	return mcfg, wcfg, stepsPerFrame, config

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
