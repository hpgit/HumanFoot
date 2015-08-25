from fltk import *
import copy
import numpy as np
import numpy.linalg as npl

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm
import Resource.ysMotionLoader as yf
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import GUI.ysSimpleViewer as ysv
import Optimization.ysAnalyticConstrainedOpt as yac
import ArticulatedBody.ysJacobian as yjc
import Util.ysPythonEx as ype
import ArticulatedBody.ysReferencePoints as yrp
import ArticulatedBody.ysMomentum as ymt
import ArticulatedBody.ysControl as yct

class QPSimulator:
	def __init__(self):
		self.qp = yac.QP()
		self.qp.clear()

		# parameter
		self.Kt = 200.
		self.Dt = 2*(self.Kt**.5)
		self.Kl = 1000.
		self.Dl = 2*(self.Kl**.5)
		self.Kh = 1000.
		self.Dh = 2*(self.Kh**.5)
		#self.Ke = 100
		#self.De = 2*(self.Ke**.5)
		self.Ke = 10
		self.De = 2*(self.Ke**.5)

		self.Bt = 10.
		self.Btau = 1.
		self.Bcon = 1.
		self.Bl = 100.
		self.Bh = 100.
		self.Be = 100.

		# constants
		self.mu = 2.
		self.contactPerSide = 4 # vertices of boxes always checked

		# flat data structure
		self.ddth_des_flat = None
		self.dth_flat = None
		self.dth_r_flat = None
		self.ddth_r_flat = None
		self.ddth_sol = None
		self.totalDOF = 0

		self.extForce = None
		self.extForceBody = None
		self.extDuration = 0

		self.Vc_tmp = np.zeros((4,3))
		self.Vc_tmp[0] = mm.normalize2(( self.mu, 1,        0))
		self.Vc_tmp[1] = mm.normalize2((-self.mu, 1,        0))
		self.Vc_tmp[2] = mm.normalize2((       0, 1,  self.mu))
		self.Vc_tmp[3] = mm.normalize2((       0, 1, -self.mu))
		
		self.CP_old = None

		#self.dH_des = None
		#self.dL_des_plane = None

	def setupWeight(self, Kt, Kl, Kh, Ke, Bt, Btau, Bcon, Bl=0., Bh=0., Be=0.):
		self.Kt = Kt
		self.Dt = 2*(self.Kt**.5)
		self.Kl = Kl
		self.Dl = 2*(self.Kl**.5)
		self.Kh = Kh
		self.Dh = 2*(self.Kh**.5)
		self.Ke = Ke
		self.De = 2*(self.Ke**.5)

		self.Bt = Bt
		self.Btau = Btau
		self.Bcon = Bcon
		self.Bl = Bl
		self.Bh = Bh
		self.Be = Be

	def addExternalForces(self, force, forceBody, duration):
		self.extForce = np.hstack( (force, np.zeros(3)) )
		self.extForceBody = forceBody
		self.extDuration = duration

	def setupQP(self, frame, motion, mcfg, model, world, config, timestep):
		motionModel = cvm.VpMotionModel(world, motion[frame], mcfg)

		# constants
		invdt = 1./timestep

		# dofs and flat data structure
		totalDOF = model.getTotalDOF()
		DOFs = model.getDOFs()
		self.totalDOF = totalDOF
		self.ddth_des_flat = ype.makeFlatList(totalDOF)
		self.ddth_r_flat = ype.makeFlatList(totalDOF)
		self.dth_flat = ype.makeFlatList(totalDOF)
		self.dth_r_flat = ype.makeFlatList(totalDOF)
		self.ddth_sol = ype.makeNestedList(DOFs)

		# momentum matrix
		linkMasses = model.getBodyMasses()
		totalMass = model.getTotalMass()
		TO = ymt.make_TO(linkMasses) 
		dTO = ymt.make_dTO(len(linkMasses))

		# optimization 
		self.qp.clear()

		# jacobian
		Jsup = yjc.makeEmptyJacobian(DOFs, 1)
		dJsup = Jsup.copy()
		
		Jsys = yjc.makeEmptyJacobian(DOFs, model.getBodyNum())
		dJsys = Jsys.copy()

		Vc_tmp = self.Vc_tmp


		# tracking
		w = self.getTrackingWeight(DOFs, motion[0].skeleton, config['weightMap'])

		th_r = motion.getDOFPositionsLocal(frame)
		th = model.getDOFPositionsLocal()
		dth_r = motion.getDOFVelocitiesLocal(frame)
		dth = model.getDOFVelocitiesLocal()
		ddth_r = motion.getDOFAccelerationsLocal(frame)
		ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, self.Kt, self.Dt)

		linkPositions = model.getBodyPositionsGlobal()
		linkVelocities = model.getBodyVelocitiesGlobal()
		linkAngVelocities = model.getBodyAngVelocitiesGlobal()
		linkInertias = model.getBodyInertiasGlobal()

		jointPositions = model.getJointPositionsGlobal()
		jointAxeses = model.getDOFAxesesLocal()

		#linkPositions_ref = motionModel.getBodyPositionsGlobal()
		#linkVelocities_ref = motionModel.getBodyVelocitiesGlobal()
		#linkAngVelocities_ref = motionModel.getBodyAngVelocitiesGlobal()
		#linkInertias_ref = motionModel.getBodyInertiasGlobal()

		#jointPositions_ref = motionModel.getJointPositionsGlobal()
		#jointAxeses_ref = motionModel.getDOFAxesesLocal()

		ype.flatten(ddth_des, self.ddth_des_flat)
		ype.flatten(dth, self.dth_flat)
		ype.flatten(dth_r, self.dth_r_flat)
		ype.flatten(ddth_r, self.ddth_r_flat)

		# get CoM
		CM = yrp.getCM(linkPositions, linkMasses, totalMass)
		dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)
		CM_plane = copy.copy(CM); CM_plane[1]=0.
		dCM_plane = copy.copy(dCM); dCM_plane[1]=0.
		
		P = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions, CM, linkInertias)
		dP = ymt.getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities, dCM, linkAngVelocities, linkInertias)

		#CM_ref = yrp.getCM(linkPositions_ref, linkMasses, totalMass)
		#dCM_ref = yrp.getCM(linkVelocities_ref, linkMasses, totalMass)
		#CM_ref_plane = copy.copy(CM_ref); CM_ref_plane[1]=0.
		#dCM_ref_plane = copy.copy(dCM_ref); dCM_ref_plane[1]=0.
		
		#P_ref = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions_ref, CM_ref, linkInertias_ref)
		#dP_ref = ymt.getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities_ref, dCM_ref, linkAngVelocities_ref, linkInertias_ref)

		# get EoM
		totalActuator = totalDOF

		invM = np.zeros((totalActuator,totalDOF))
		invMc = np.zeros(totalDOF)
		model.getInverseEquationOfMotion(invM, invMc)

		# contact detection
		Ks = 1
		Ds = 1
		supsupR = motion[0].skeleton.getJointIndex('RightLeg')
		supsupL = motion[0].skeleton.getJointIndex('LeftLeg')
		supR = motion[0].skeleton.getJointIndex('RightFoot')
		supL = motion[0].skeleton.getJointIndex('LeftFoot')
		#bodyIDsToCheck = range(world.getBodyNum())
		#bodyIDsToCheck = [supsupR, supsupL]
		bodyIDsToCheck = [supR, supL]
		mus = [.5]*len(bodyIDsToCheck)
		bodyIDs, contactPositions, contactPositionLocals, contactForces, contactVelocities = world.calcManyPenaltyForce(self.contactPerSide, bodyIDsToCheck, mus, Ks, Ds)
		#bodyIDs, contactPositions, contactPositionLocals, contactForces, contactVelocities = world.calcOnePenaltyForce(bodyIDsToCheck, mus, Ks, Ds)

		footCenterL = model.getBodyPositionGlobal(supL)
		footCenterR = model.getBodyPositionGlobal(supR)
		footCenter = footCenterL.copy()

		footRefCenterL = motionModel.getBodyPositionGlobal(supL)
		footRefCenterR = motionModel.getBodyPositionGlobal(supR)
		#if supL in bodyIDs:
			#if supR in bodyIDs:
				#footCenter = footCenterL + (footCenterR-footCenterL)/2.
			#else:
				#footCenter = footCenterL.copy()
		#else:
			#if supR in bodyIDs:
				#footCenter = footCenterR.copy()
			#else:
				#footCenter = np.array((0,0,0))

		contactL = 1
		contactR = 1

		if footRefCenterL[1] < 0.2:
			if footRefCenterR[1] < 0.2:
				footCenter = footCenterL + (footCenterR-footCenterL)/2.
			else:
				footCenter = footCenterL.copy()
				contactR = 0
		else:
			contactL = 0
			if footRefCenterR[1] < 0.2:
				footCenter = footCenterR.copy()
			else:
				footCenter = np.array((0,0,0))
				contactR = 0
		#print(contactR, contactL)
		footCenter[1] = 0.

		# linear momentum
		CM_ref = footCenter
		#CM_ref = 
		#dL_des_plane = self.Kl*totalMass*(CM_ref - CM) + self.Dl*totalMass*(dCM_ref - dCM)
		dL_des_plane = self.Kl*totalMass*(CM_ref - CM) + self.Dl*totalMass*(-dCM)
		dL_des_plane[1] = 0.
		
		# angular momentum
		CP_ref = footCenter
		#bodyIDs, contactPositions, contactPositionLocals, contactForces = world.calcManyPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
		#CP = yrp.getCP(contactPositions, contactForces)
		CP = yrp.getSimpleCP(contactPositions)
		if self.CP_old==None or CP==None:
			dCP = None
		else:
			dCP = (CP - self.CP_old[0])/(1/30.)
		self.CP_old = CP
		
		if CP!=None and dCP!=None:
			ddCP_des = self.Kh*(CP_ref - CP) - self.Dh*(dCP)
			CP_des = CP + dCP*(1/30.) + .5*ddCP_des*((1/30.)**2)
			dH_des = np.cross((CP_des - CM), (dL_des_plane + totalMass*np.array((0,-9.8,0))))
			#if contactChangeCount >0: # and contactChangeType == 'DtoS':
				##dH_des *= (maxContactChangeCount - contactChangeCount)/(maxContactChangeCount*10)
				#dH_des *= (self.maxContactChangeCount - self.contactChangeCount)/(self.maxContactChangeCount)
				##dH_des *= (contactChangeCount)/(maxContactChangeCount)*.9+.1
		else:
			dH_des = None
		H = np.dot(P, np.dot(Jsys, self.dth_flat))
		dH_des = -self.Kh*H[3:]



		# equality constraints
		JcTVc_append = np.zeros((totalDOF, 0))
		VcTJc_list = []
		VcTdJc_list = []
		dVcTJc_list = []
		ac_offset_list = []
		totalContact = 4*len(bodyIDs)
		totalProblem = totalDOF+totalActuator+totalContact
		
		preSup = -1
		for i in range(len(contactPositions)):
			sup = bodyIDs[i]
			supJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, sup)]

			if preSup != sup:
				bodyPos = linkPositions[sup]
				bodyVel = linkVelocities[sup]
				yjc.computeJacobian2(Jsup, DOFs, jointPositions, jointAxeses, [bodyPos], supJointMasks)
				yjc.computeJacobianDerivative2(dJsup, DOFs, jointPositions, jointAxeses, linkAngVelocities, [bodyPos], supJointMasks)

			R_dAd = np.hstack( (np.vstack( (np.eye(3), mm.getCrossMatrixForm(-bodyPos)) ), np.vstack( (np.zeros((3,3)), np.eye(3)) ) ) )
			dR_dAd = np.hstack( (np.vstack( (np.eye(3), mm.getCrossMatrixForm(-bodyVel)) ), np.vstack( (np.zeros((3,3)), np.eye(3)) ) ) )
			#R_dAd = np.hstack( (np.vstack( (np.eye(3), mm.getCrossMatrixForm(-contactPositions[i])) ), np.vstack( (np.zeros((3,3)), np.eye(3)) ) ) )
			#dR_dAd = np.hstack( (np.vstack( (np.eye(3), mm.getCrossMatrixForm(-contactVelocities[i])) ), np.vstack( (np.zeros((3,3)), np.eye(3)) ) ) )

			p = contactPositions[i]
			dotp = contactVelocities[i]
			VcT_tmp = np.zeros((4,6)) 
			dVcT_tmp = VcT_tmp.copy()
			for ii in range(4):
				n = Vc_tmp[ii]
				pcn = np.cross(contactPositions[i], Vc_tmp[ii])
				VcT_tmp[ii][:3] =n
				VcT_tmp[ii][3:] =pcn
				dotpcn = np.cross(contactVelocities[i], Vc_tmp[ii])
				dVcT_tmp[ii][3:] = dotpcn

			Vc = np.dot(R_dAd, VcT_tmp.T)
			dVc = np.dot(R_dAd, dVcT_tmp.T) + np.dot(dR_dAd, VcT_tmp.T)

			JcTVc = np.dot( Jsup.T, Vc)
			JcTVc_append = np.hstack((JcTVc_append, JcTVc))
			VcTJc_list.append( JcTVc.T )
			VcTdJc_list.append( np.dot(Vc.T, dJsup) )
			dVcTJc_list.append( np.dot(dVc.T, Jsup) )

			#TODO:
			# when friction cones and velocity cones differ?
			#JcTVc = np.dot( Jsup.T, VcT.T)
			#JcTVc_append = np.hstack((JcTVc_append, JcTVc))
			#VcTJc_list.append( JcTVc.T )
			#VcTdJc_list.append( np.dot(VcT, dJsup) )
			#dVcTJc_list.append( np.dot(dVcT, Jsup) )

			penDepth = -0.5-contactPositions[i][1]
			if penDepth < 0.:
				penDepth = 0.
			penDepth = 0.
			ac_offset = 1000.*penDepth*np.ones(4)
			ac_offset_list.append(ac_offset)
			preSup = sup

		extForce = np.zeros(totalActuator)
		if self.extDuration > 0:
			Jext = yjc.makeEmptyJacobian(DOFs, 1)
			extForcePos = model.getBodyPositionGlobal(self.extForceBody)
			extJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, self.extForceBody)]
			yjc.computeJacobian2(Jext, DOFs, jointPositions, jointAxeses, [extForcePos], extJointMasks)
			extForce = np.dot(Jext.T, self.extForce)
			
			self.extDuration -= timestep
			if self.extDuration < 0:
				self.extDuration = 0

		self.addQPEqualityInverseEomConstraint(totalProblem, totalDOF, totalActuator, totalContact, invM, invMc, JcTVc_append, extForce)
		
		# inequality constraints

		if totalContact> 0:
			self.addQPInequalityContactForceConstraint(totalProblem, totalDOF, totalActuator, totalContact) 
			self.addQPInequalityVelocityConstraint(totalProblem, totalDOF, totalActuator, totalContact, VcTJc_list, VcTdJc_list,dVcTJc_list, self.dth_flat, ac_offset_list, invdt)
			#self.addQPInequalityVelocityConstraint(totalProblem, totalDOF, totalActuator, totalContact, VcTJc_vel_list, VcTdJc_vel_list,dVcTJc_vel_list, self.dth_flat, ac_offset_list, 30.)
		torqueMax = 1000.*np.ones(totalActuator-6)
		torqueMin = -torqueMax
		self.addQPInequalityTorqueConstraint(totalProblem, totalDOF, totalActuator, totalContact, torqueMax, torqueMin)
		
		# objectives
		self.addQPTrackingTerms(totalProblem, 0, totalDOF, self.Bt, w, self.ddth_des_flat)
		self.addQPTorqueTerms(totalProblem, totalDOF, totalActuator, self.Btau, w)
		if totalContact > 0:
			self.addQPContactForceTerms(totalProblem, totalDOF+totalActuator, totalContact, self.Bcon)
			if dH_des !=None:
				allLinkJointMasks = yjc.getAllLinkJointMasks(motion[0].skeleton)
				yjc.computeJacobian2(Jsys, DOFs, jointPositions, jointAxeses, linkPositions, allLinkJointMasks)
				yjc.computeJacobianDerivative2(dJsys, DOFs, jointPositions, jointAxeses, linkAngVelocities, linkPositions, allLinkJointMasks)
				self.addLinearAndAngularBalancigTerms(totalProblem, 0, totalDOF, self.Bl, self.Bh, P, Jsys, self.dth_flat, dP, dJsys, dL_des_plane, dH_des)

		# end effector
		#TODO:
		eeList = [supR, supL]

		#if totalContact > 0:
		for ee in eeList:
			eeCenter = model.getBodyPositionGlobal(ee)
			eeJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, ee)]
			yjc.computeJacobian2(Jsup, DOFs, jointPositions, jointAxeses, [eeCenter], eeJointMasks)
			yjc.computeJacobianDerivative2(dJsup, DOFs, jointPositions, jointAxeses, linkAngVelocities, [eeCenter], eeJointMasks, False)
			ee_genvel_ref = np.dot(Jsup, self.dth_r_flat)
			ee_genacc_ref = np.dot(Jsup, self.ddth_r_flat) + np.dot(dJsup, self.dth_r_flat)

			ee_pos_ref = motionModel.getBodyPositionGlobal(ee)
			ee_pos = model.getBodyPositionGlobal(ee)
			ee_vel_ref = ee_genvel_ref[:3]
			ee_vel = model.getBodyVelocityGlobal(ee)
			ee_acc_ref = ee_genacc_ref[:3]
			ddp_des_pos = self.Ke*( (ee_pos_ref-th_r[0][0]) - (ee_pos-th[0][0]) )
			ddp_des_pos += self.De*(ee_vel_ref - ee_vel) 
			ddp_des_pos += ee_acc_ref

			eeOri = model.getBodyOrientationGlobal(ee)
			eeAngVel = model.getBodyAngVelocityGlobal(ee)
			ee_angvel_ref = ee_genvel_ref[3:]
			ee_angacc_ref = ee_genacc_ref[3:]
			a_ori_diff = mm.logSO3(mm.getSO3FromVectors(np.dot(eeOri, np.array([0,1,0])), np.array([0,1,0])))
			ddp_des_ang = self.Ke*a_ori_diff + self.De*(-eeAngVel)
			#ddp_des_ang = self.Ke*a_ori_diff + self.De*(ee_angvel_ref-eeAngVel)
			#ddp_des_ang += ee_angacc_ref

			ddp_des = np.hstack( (ddp_des_pos, ddp_des_ang) )

			self.addEndEffectorTerms(totalProblem, 0, totalDOF, Jsup, dJsup, self.dth_flat, ddp_des, self.Be)
			#self.addEqualityEndEffectorTerms(totalProblem, 0, totalDOF, Jsup, dJsup, self.dth_flat, ddp_des, self.Be)

		return contactPositions, CP, CM, footCenter, dL_des_plane, CM_ref


	def stepQP(self, model, timestep):
		totalDOF = self.totalDOF
		# optimization
		x = self.qp.solve()
		ype.nested(np.array(x[:totalDOF].T).flatten(), self.ddth_sol)
		test_ddq = np.array(x[:totalDOF].T).flatten()
		test_tau = np.array(x[totalDOF:2*totalDOF].T).flatten()
		test_lambda = np.array(x[2*totalDOF:].T).flatten()

		# integrate
		if self.ddth_sol != None:
			model.stepKinematics(timestep, self.ddth_sol)
			self.ddth_des_flat = None
			self.dth_flat = None
			self.ddth_sol = None
			#np.dot(Vc_tmp.T, np.array(test_lambda[4*i:4*i+4]))
			if test_lambda != []:
				contactForces = []
				for i in range(len(test_lambda)/4):
					contactForces.append( np.dot(self.Vc_tmp.T, test_lambda[4*i:4*i+4]))

				return x, contactForces
			else:
				return x, None
		else:
			print("setup QP first!")
			return None

	# objectives
	def getTrackingWeight(self, DOFs, skeleton, weightMap, rootPositionWeight=0.):
		weights = [1.]*skeleton.getJointNum()
		for name, weight in weightMap.items():
			index = skeleton.getJointIndex(name)
			weights[index] = weight
		
		totalDOF = 0
		for dof in DOFs:
			totalDOF += dof
		
		weights_ext = [None]*totalDOF
		ype.repeatListElements(weights, weights_ext, DOFs)
		weights_ext[0:3] = [rootPositionWeight, rootPositionWeight, rootPositionWeight]  

		return weights_ext

	def addQPTrackingTerms(self, totalProblemSize, si, totalDOF, weight, jointWeights, ddth_des_flat):
		# minimize | Wt(ddth_des - ddth) |^2    
		#jointWeights[0] = 1.
		#jointWeights[1] = 1.
		#jointWeights[2] = 1.
		A = np.diag( np.append([jointWeights[i] for i in range(len(jointWeights))], np.zeros(totalProblemSize-totalDOF)) )
		b = np.append(np.array([jointWeights[i]*ddth_des_flat[i] for i in range(len(jointWeights))]), np.zeros(totalProblemSize-totalDOF))
		self.qp.addObjective(A,b,weight)

	def addQPTorqueTerms(self, totalProblemSize, si, totalActuator, weight, jointTorqueWeights):
		# minimize |tau|^2
		A = np.diag(np.append(np.append(np.zeros((si)), 1.*np.ones((totalActuator))), np.zeros(totalProblemSize-si-totalActuator)))
		b = np.zeros(totalProblemSize)
		self.qp.addObjective(A,b,weight)

	def addQPContactForceTerms(self, totalProblemSize, si, totalContact, weight):
		# minimize |lambda|^2
		A = np.diag(np.append(np.zeros(si), 1.*np.ones(totalContact)))
		b = np.zeros(totalProblemSize)
		self.qp.addObjective(A,b,weight)
	
	def addLinearAndAngularBalancigTerms(self, totalProblemSize, si, totalDOF, linearWeight, angularWeight, P, Jsys, dth, dotP, dotJsys, dotLdes, dotHdes):
		#minimize |Bl(dotL - dotLdes)|^2 and |Bh(dotH - dotHdes)|^2
		Wl = linearWeight ** .5
		Wh = angularWeight ** .5
		W =  np.diag( np.append( Wl*np.ones(3), Wh*np.ones(3) ) )
		A = np.hstack( (np.dot(W, np.dot(P, Jsys)), np.zeros( (6, totalProblemSize - totalDOF))))
		b = np.hstack( (dotLdes, dotHdes) ) - np.dot(dotP, np.dot(Jsys, dth)) - np.dot(P, np.dot(dotJsys, dth))
		self.qp.addObjective(A, np.dot(W,b))

	def addEndEffectorTerms(self, totalProblemSize, si, totalDOF, Jee, dJee, dth, ddp_des, weight):
		#minimize |ddp - ddp_des|^2 = |J * ddth - (ddp_des-dJ * dth)|^2
		#foot should be parallel to ground
		A = np.hstack( ( Jee.copy(), np.zeros((6, totalProblemSize-totalDOF)) ) )
		b = ddp_des - np.dot(dJee, dth)
		#self.qp.addEqualityConstraint(A[3:], b[3:])
		self.qp.addObjective(A, b, weight)
		#self.qp.addObjective(A[:0], b[:0], weight)
		#self.qp.addObjective(A[2:], b[2:], weight)

	# constraints
	def addQPEqualityEomConstraint(self, totalProblemSize, totalDOF, totalActuator, totalContact, M, c, JcTVc_append):
		# subject to Mq'' -tau - JcTVclambda = -b
		#                  tau[0:6)          =  0
		# [M -I -JcTVc]
		S = np.diag(np.append(np.zeros(6), 1.*np.ones(totalActuator-6)))
		A = np.vstack(( np.zeros((6,totalProblemSize)), np.hstack(( M,-S,-JcTVc_append )) ))
		for i in range(0, 6):
			A[i, totalDOF+i] = 1.
		b = np.append(np.zeros(6), -c)
		self.qp.addEqualityConstraint(A, b)

	def addEqualityEndEffectorTerms(self, totalProblemSize, si, totalDOF, Jee, dJee, dth, ddp_des, weight):
		#minimize J * ddth = ddp_des - dJ * dth
		#foot should be parallel to ground
		A = np.hstack( ( Jee.copy(), np.zeros((6, totalProblemSize-totalDOF)) ) )
		b = ddp_des - np.dot(dJee, dth)
		self.qp.addEqualityConstraint(A[3:], b[3:])
		#self.qp.addEqualityConstraint(A, b)

	def addQPEqualityInverseEomConstraint(self, totalProblemSize, totalDOF, totalActuator, totalContact, invM, invMc, JcTVc_append, extForce):
		# subject to Mq'' -tau - JcTVclambda = -b
		#                  tau[0:6)          =  0
		# [I -M^-1 -M^-1*JcTVc]
		S = np.diag(np.append(np.zeros(6), 1.*np.ones(totalActuator-6)))
		#A = np.vstack(( np.zeros((6,totalProblemSize)), np.hstack(( np.eye(totalDOF),-np.dot(invM, S),np.dot(invM, -JcTVc_append) )) ))
		A = np.vstack(( np.zeros((6,totalProblemSize)), np.hstack(( np.eye(totalDOF),-invM,-np.dot(invM, JcTVc_append) )) ))
		for i in range(0, 6):
			A[i, totalDOF+i] = 1.
		b = np.append(np.zeros(6), -invMc) 
		b += np.append(np.zeros(6), np.dot(invM, extForce))
		#print(A[totalDOF:totalDOF+6])
		self.qp.addEqualityConstraint(A, b)

	def addQPEqualityContactConstraint(self, totalProblemSize, totalDOF, totalActuator, totalContact, Jc, dJc, dth, a_c):
		# subject to Jc q'' = -dJc q' + a_sup
		A = np.hstack( (Jc , np.zeros((6, totalActuator+totalContact))) )
		b = -np.dot(dJc, dth) + a_c
		self.qp.addEqualityConstraint(A, b)

	def addQPInequalityTorqueConstraint(self, totalProblemSize, totalDOF, totalActuator, totalContact, torqueMax, torqueMin):
		# subject to tau <= max and -tau <= min
		G_max = np.hstack((np.zeros((totalActuator-6, totalDOF+6)), np.diag(1.*np.ones(totalActuator-6)), np.zeros((totalActuator-6, totalContact)) ))
		G_min = -G_max
		G = np.vstack((G_max, G_min))
		h = np.append( torqueMax, -torqueMin)
		if G.shape[0] != h.shape[0]:
			print('Inequality Torque : G and h shapes are not matched') 

		self.qp.addInequalityConstraint(G, h)

	def addQPInequalityContactForceConstraint(self, totalProblemSize, totalDOF, totalActuator, totalContact):
		# subject to -lambda <= 0
		G = -np.hstack((np.zeros((totalContact, totalDOF+totalActuator)), np.diag(1.*np.ones(totalContact)) ))
		h = np.zeros(totalContact)
		if G.shape[0] != h.shape[0]:
			print('Inequality Contact : G and h shapes are not matched') 
		self.qp.addInequalityConstraint(G, h)

	def addQPInequalityVelocityConstraint(self, totalProblemSize, totalDOF, totalActuator, totalContact, VcTJc_list, VcTdJc_list, dVcTJc_list, dq, ac_offset_list, invdt):
		# subject to -(VcTJcq'' + VcTJc'q' + VcT'Jcq') <= 0
		#TODO:
		# assume that Vc' = 0 <- is it right? check it!
		G = None 
		h = None
		for i in range(len(VcTJc_list)):
			G_temp = np.hstack( (-VcTJc_list[i], np.zeros((4, totalActuator+totalContact))) )
			#h_temp = np.dot(VcTdJc_list[i], dq) + (-.05)*np.ones(4)
			#h_temp = (-1/30.)*np.dot(VcTJc_list[i], dq)+ np.dot(VcTdJc_list[i], dq) + (-1.)*ac_offset_list[i]
			#h_temp = (-1/30.)*np.dot(VcTJc_list[i], dq)+ np.dot(VcTdJc_list[i], dq)
			h_temp = np.dot(dVcTJc_list[i], dq) +  np.dot(VcTdJc_list[i], dq) + np.dot(VcTJc_list[i], dq) * invdt + (-1.)*ac_offset_list[i]
			if G == None:
				G = G_temp.copy()
				h = h_temp.copy()
			else:
				G = np.vstack( (G, G_temp) )
				h = np.append( h, h_temp )

		if G.shape[0] != h.shape[0]:
			print('Inequality Velocity : G and h shapes are not matched') 
		self.qp.addInequalityConstraint(G, h)
