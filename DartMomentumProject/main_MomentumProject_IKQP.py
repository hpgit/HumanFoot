#import psyco; psyco.full()
from fltk import *
import copy
import numpy as np
import time

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')
#if './modules' not in sys.path:
#    sys.path.append('./modules')
    
import Math.mmMath as mm
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import GUI.ysSimpleViewer as ysv
import Optimization.ysAnalyticConstrainedOpt as yac
import ArticulatedBody.ysJacobian as yjc
import Util.ysPythonEx as ype
import ArticulatedBody.ysReferencePoints as yrp
import ArticulatedBody.ysMomentum as ymt
import ArticulatedBody.ysControl as yct

import GUI.ysMultiViewer as ymv

import Motion.ysHierarchyEdit as yme
import Simulator.ysPhysConfig as ypc
import Simulator.hpQPSimulator as hqp

import numpy.linalg as npl

import mtOptimize as mot
import mtInitialize_Simple as mit

MULTI_VIEWER = False

MOTION_COLOR = (213,111,162)
CHARACTER_COLOR = (20,166,188)
FEATURE_COLOR = (255,102,0)
CHARACTER_COLOR2 = (200,200,200)


contactState = 0
g_applyForce = False

g_initFlag = 0

softConstPoint = [0, 0, 0]

forceShowFrame = 0
forceApplyFrame = 0

JsysPre = 0
JsupPreL = 0
JsupPreR = 0
JsupPre = 0

stage = 0

desCOMOffset = 0

contactRendererName = []

leftHipTimer =0
landingTimer = 0
landingTimerOn = False

## Constant
STATIC_BALANCING = 0
MOTION_TRACKING = 1
DYNAMIC_BALANCING = 2
POWERFUL_BALANCING = 3
POWERFUL_MOTION_TRACKING = 4
FLYING = 5

def checkAll(list, value) :
    for i in range(len(list)) :
        if list[i] != value :
            return 0
    return 1

def getDesFootLinearAcc(refModel, controlModel, footIndex, ModelOffset, CM_ref, CM, Kkk, Dkk, restPosY = 0.0) :
        
    desLinearAcc = [0,0,0]

    refPos = refModel.getBodyPositionGlobal(footIndex)  
    curPos = controlModel.getBodyPositionGlobal(footIndex)
    refPos[0] += ModelOffset[0]
                                
    refVel = refModel.getBodyVelocityGlobal(footIndex) 
    curVel = controlModel.getBodyVelocityGlobal(footIndex)
    refAcc = refModel.getBodyAccelerationGlobal(footIndex)
         
    refPos[1] = restPosY#0.032
        
    if refPos[1] < 0.0 :
        refPos[1] = restPosY#0.032
        
    desLinearAcc = yct.getDesiredAcceleration(refPos, curPos, refVel, curVel, refAcc, Kkk, Dkk)         

    return desLinearAcc, refPos

def getDesFootAngularAcc(refModel, controlModel, footIndex, Kk, Dk, axis = [0,1,0], desAng = [0,1,0]) :
    desAngularAcc = [0,0,0]

    curAng = [controlModel.getBodyOrientationGlobal(footIndex)]
    refAngVel = refModel.getBodyAngVelocityGlobal(footIndex)
    curAngVel = controlModel.getBodyAngVelocityGlobal(footIndex)
    refAngAcc = (0,0,0)
                        
    curAngY = np.dot(curAng, np.array(axis))
    refAngY = np.array(desAng)
    if stage == MOTION_TRACKING+10:    
        refAng = [refModel.getBodyOrientationGlobal(footIndex)]
        refAngY2 = np.dot(refAng, np.array([0,1,0]))
        refAngY = refAngY2[0]
    
    aL = mm.logSO3(mm.getSO3FromVectors(curAngY[0], refAngY))
    desAngularAcc = Kk*aL + Dk*(refAngVel-curAngVel)

    return desAngularAcc

def getPartJacobian(_Jsys, _jIdx):
    # warning : only Jsys works.
    return _Jsys[6*_jIdx : 6*_jIdx+6].copy()

def getBodyGlobalPos(model, motion, name):
    return model.getBodyPositionGlobal(motion[0].skeleton.getJointIndex(name))

def getBodyGlobalOri(model, motion, name):
    return model.getBodyOrientationGlobal(motion[0].skeleton.getJointIndex(name))

def main():

    np.set_printoptions(precision=4, linewidth=200)
    
#    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_5()
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped()
    mcfg_motion = mit.normal_mcfg()
        
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    motionModel.recordVelByFiniteDiff()

    motionOriModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)

    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    
    footPartNum = config['FootPartNum']

    if footPartNum > 1:
        elasticity = 2000
        damping = 2*(elasticity**.5)
    
        springBody1 = 5
        springBody2 = 6        
        springBody1Pos = motionModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody1]))
        springBody2Pos = motionModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody2]))

        initialDist = mm.length(springBody1Pos - springBody2Pos)*1.
        node = mcfg.getNode(mit.LEFT_PHALANGE_1)
        initialDist -= node.width#0.084
        v1 = (-node.width*0.5,0.0,node.length*0.4)
        v2 = (node.width*0.5,0.0,node.length*0.4)
        controlModel.setSpring(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody1]), motion[0].skeleton.getJointIndex(config['FootLPart'][springBody2]), elasticity, damping, v2, v1, initialDist)
        controlModel.setSpring(motion[0].skeleton.getJointIndex(config['FootRPart'][springBody1]), motion[0].skeleton.getJointIndex(config['FootRPart'][springBody2]), elasticity, damping, v1, v2, initialDist)

        
        #elasticity = 10
        #damping = 2*(elasticity**.5)
        #springBody1 = 3
        #springBody2 = 4    
        #node = mcfg.getNode(mit.LEFT_PHALANGE_1)
        #springBody1Pos = motionModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody1]))
        #springBody2Pos = motionModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody2]))
        #initialDist = mm.length(springBody1Pos - springBody2Pos)*1.
        #initialDist -= node.width#0.084
        #v1 = (-node.width*0.5,0.0,-node.length*0.4)
        #v2 = (node.width*0.5,0.0,-node.length*0.4)
        ##controlModel.setSpring(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody1]), motion[0].skeleton.getJointIndex(config['FootLPart'][springBody2]), elasticity, damping, v2, v1, initialDist)
        ##controlModel.setSpring(motion[0].skeleton.getJointIndex(config['FootRPart'][springBody1]), motion[0].skeleton.getJointIndex(config['FootRPart'][springBody2]), elasticity, damping, v1, v2, initialDist)
        
    
    vpWorld.initialize()
    controlModel.initializeHybridDynamics()
    
    #ModelOffset = (1.5, -0.01, 0)
    ModelOffset = (1.5, 0.04, 0)
    controlModel.translateByOffset(ModelOffset)
    
    totalDOF = controlModel.getTotalDOF()
    DOFs = controlModel.getDOFs()
        
    # parameter 
    Kt = config['Kt']; Dt = config['Dt'] # tracking gain
    Kl = config['Kl']; Dl = config['Dl'] # linear balance gain
    Kh = config['Kh']; Dh = config['Dh'] # angular balance gain
    Ks = config['Ks']; Ds = config['Ds']  # penalty force spring gain
    
    Bt = config['Bt']
    Bl = config['Bl']
    Bh = config['Bh']
        
    w = mot.getTrackingWeight(DOFs, motion[0].skeleton, config['weightMap'])
    w2 = mot.getTrackingWeight(DOFs, motion[0].skeleton, config['weightMap2'])
    #w_IK = mot.getTrackingWeight(DOFs, motion[0].skeleton, config['IKweightMap'])
    supL =  motion[0].skeleton.getJointIndex(config['supLink'])
    supR =  motion[0].skeleton.getJointIndex(config['supLink2'])
    rootB = motion[0].skeleton.getJointIndex(config['root'])

    selectedBody = motion[0].skeleton.getJointIndex(config['end'])
    #constBody = motion[0].skeleton.getJointIndex('LeftForeArm')
    constBody = motion[0].skeleton.getJointIndex(config['const'])
    
    # jacobian 
    Jsup = yjc.makeEmptyJacobian(DOFs, 1)
    dJsup = Jsup.copy()
    JsupPre = Jsup.copy()

    Jsys_IK = yjc.makeEmptyJacobian(DOFs, controlModel.getBodyNum())

    Jsys = yjc.makeEmptyJacobian(DOFs, controlModel.getBodyNum())
    dJsys = Jsys.copy()
    JsysPre = Jsys.copy()
        
    Jconst = yjc.makeEmptyJacobian(DOFs, 1)
    dJconst = Jconst.copy()

    Jcom = yjc.makeEmptyJacobian(DOFs, 1, False)
    dJcom = Jcom.copy()

    JcomAng = yjc.makeEmptyJacobian(DOFs, 1, False)
    dJcomAng = JcomAng.copy()

    ###############

    jFootL_IK = [None]*footPartNum
    jFootR_IK = [None]*footPartNum
    
    indexFootL = [None]*footPartNum
    indexFootR = [None]*footPartNum
    jFootL = [None]*footPartNum
    dJFootL = [None]*footPartNum
    jFootR = [None]*footPartNum
    dJFootR = [None]*footPartNum
    jointMasksFootL = [None]*footPartNum
    jointMasksFootR = [None]*footPartNum
    
    for i in range(footPartNum) :
        jFootL[i] = yjc.makeEmptyJacobian(DOFs, 1)
        dJFootL[i] = jFootL[i].copy()
        jFootR[i] = yjc.makeEmptyJacobian(DOFs, 1)
        dJFootR[i] = jFootR[i].copy()
        
        indexFootL[i] = motion[0].skeleton.getJointIndex(config['FootLPart'][i])
        indexFootR[i] = motion[0].skeleton.getJointIndex(config['FootRPart'][i])

        jointMasksFootL[i] = [yjc.getLinkJointMask(motion[0].skeleton, indexFootL[i])]
        jointMasksFootR[i] = [yjc.getLinkJointMask(motion[0].skeleton, indexFootR[i])]       
    
    constJointMasks = [yjc.getLinksJointMask(motion[0].skeleton, [indexFootL[0], indexFootR[0]])]
    #constJointMasks = [yjc.getLinksJointMask(motion[0].skeleton, [indexFootL[0]])]
    #constJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, constBody)]
    allLinkJointMasks = yjc.getAllLinkJointMasks(motion[0].skeleton)
    
    #comLowerJointMasks = [yjc.getLinksJointMask(motion[0].skeleton, [motion[0].skeleton.getJointIndex('LeftLeg'), motion[0].skeleton.getJointIndex('RightLeg')])]
    comUpperJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, selectedBody)]  
    #comLowerJointMasks = [yjc.getLinksJointMask(motion[0].skeleton, [motion[0].skeleton.getJointIndex('LeftLeg'), motion[0].skeleton.getJointIndex('RightLeg')])]
    comUpperJointMasks[0][0] = 0
    #comUpperJointMasks[0][1] = 1
    #comUpperJointMasks[0][10] = 1
    comUpperJointMasks[0][2] = 1
    comUpperJointMasks[0][11] = 1
    
    #print(comUpperJointMasks)
    
    comLowerJointMasks = [yjc.getLinksJointMask(motion[0].skeleton, [motion[0].skeleton.getJointIndex('LeftLeg'), motion[0].skeleton.getJointIndex('RightLeg')])]
    
    '''
    maskArray = [foreSupLJointMasks, foreSupRJointMasks, rearSupLJointMasks, rearSupRJointMasks]
    parentArray = [supL, supR, supL, supR]
    effectorArray = [foreSupL, foreSupR, rearSupL, rearSupR]
    for j in range(4) :
        for i in range(len(foreSupLJointMasks)) :
            if i == parentArray[j] or i == effectorArray[j] :
                maskArray[j][0][i] = 1
            else :
                maskArray[j][0][i] = 0
    '''     
    # momentum matrix
    linkMasses = controlModel.getBodyMasses()
    totalMass = controlModel.getTotalMass()
    TO = ymt.make_TO(linkMasses) 
    dTO = ymt.make_dTO(len(linkMasses))
    
    # optimization
    qps = hqp.QPSimulator()
    problem = yac.LSE(totalDOF, 6)
    a_sup = (0,0,0, 0,0,0) #L
    #a_sup2 = (0,0,0, 0,0,0)#R
    a_sup2 = [0,0,0, 0,0,0]#R
    a_sup_2 = [0,0,0, 0,0,0, 0,0,0, 0,0,0]
    CP_old = [mm.v3(0.,0.,0.)]
        
    # penalty method 
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [1.]*len(bodyIDsToCheck)

    # flat data structure
    ddth_des_flat = ype.makeFlatList(totalDOF)
    dth_flat = ype.makeFlatList(totalDOF)
    ddth_sol = ype.makeNestedList(DOFs)

    dth_IK = ype.makeNestedList(DOFs)

    d_th_IK = ype.makeNestedList(DOFs)
    d_th_IK_L = ype.makeNestedList(DOFs)
    d_th_IK_R = ype.makeNestedList(DOFs)
    dd_th_IK = ype.makeNestedList(DOFs)
    dd_th_IK_flat = ype.makeFlatList(totalDOF)
    d_th_IK_flat = ype.makeFlatList(totalDOF)
    ddth_c_flat = ype.makeFlatList(totalDOF)


    # viewer
    rd_footCenter = [None]
    rd_footCenter_ref = [None]
    rd_footCenterL = [None]
    rd_footCenterR = [None]
    rd_CM_plane = [None]
    rd_CM_plane_ref = [None]
    rd_CM_ref = [None]
    rd_CM_des = [None]
    rd_CM = [None]
    rd_CM_vec = [None]
    rd_CM_ref_vec = [None]
    rd_CP = [None]
    rd_CP_des = [None]
    rd_dL_des_plane = [None]
    rd_dH_des = [None]
    rd_grf_des = [None]
    rd_footCenter_des = [None]
    rd_exf_des = [None]
    rd_root_des = [None]
    rd_soft_const_vec = [None]
    
    rd_root = [None]
    
    rd_footL_vec = [None]
    rd_footR_vec = [None]
        
    rd_CMP = [None]
    
    rd_DesPosL = [None]
    rd_DesPosR = [None]
    
    rd_DesForePosL = [None]
    rd_DesForePosR = [None]
    rd_DesRearPosL = [None]
    rd_DesRearPosR = [None]

    rd_Joint = [None]
    rd_Joint2 = [None]
    rd_Joint3 = [None]
    rd_Joint4 = [None]
    rd_desPoints = [None]
        
    #rd_contactForces = [None]*10000
    #rd_contactPositions = [None]*10000
    rd_virtualForce = [None]

    rootPos = [None]
    selectedBodyId = [selectedBody]
    extraForce = [None]
    applyedExtraForce = [None]
    applyedExtraForce[0] = [0,0,0]

    normalVector = [[0,2,0]]
    
    if MULTI_VIEWER:
        viewer = ymv.MultiViewer(800, 655)
        #viewer = ymv.MultiViewer(1600, 1255)
        viewer.setRenderers1([cvr.VpModelRenderer(motionModel, CHARACTER_COLOR, yr.POLYGON_FILL)])
        viewer.setRenderers2([cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, yr.POLYGON_FILL)])
 
    else:
        viewer = ysv.SimpleViewer()
    #    viewer.record(False)
    #    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
        viewer.doc.addObject('motion', motion)
        viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (100,100,100), yr.POLYGON_FILL)) #(150,150,255)
        viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, yr.POLYGON_FILL))
        #viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, yr.POLYGON_LINE))
        #viewer.doc.addRenderer('rd_footCenter', yr.PointsRenderer(rd_footCenter))    
        #viewer.doc.addRenderer('rd_footCenter_des', yr.PointsRenderer(rd_footCenter_des, (150,0,150))    )
        #viewer.doc.addRenderer('rd_footCenterL', yr.PointsRenderer(rd_footCenterL))  
        #viewer.doc.addRenderer('rd_footCenterR', yr.PointsRenderer(rd_footCenterR))
        viewer.doc.addRenderer('rd_CM_plane', yr.PointsRenderer(rd_CM_plane, (255,255,0)))
        viewer.doc.addRenderer('rd_CM', yr.PointsRenderer(rd_CM, (255,0,255)))
        viewer.doc.addRenderer('rd_CM_des', yr.PointsRenderer(rd_CM_des, (64,64,255)))
        viewer.doc.addRenderer('rd_CM_vec', yr.VectorsRenderer(rd_CM_vec, rd_CM_plane, (255,0,0), 3))
        #viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (0,255,0)))
        viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (255,0,128)))
    #    viewer.doc.addRenderer('rd_dL_des_plane', yr.VectorsRenderer(rd_dL_des_plane, rd_CM, (255,255,0)))
    #    viewer.doc.addRenderer('rd_dH_des', yr.VectorsRenderer(rd_dH_des, rd_CM, (0,255,0)))
        #viewer.doc.addRenderer('rd_grf_des', yr.ForcesRenderer(rd_grf_des, rd_CP, (0,255,255), .001))

        viewer.doc.addRenderer('rd_exf_des', yr.ForcesRenderer(rd_exf_des, rd_root_des, (0,255,0), .009, 0.04))
        
        #viewer.doc.addRenderer('rd_CMP', yr.PointsRenderer(rd_CMP, (0,0,255)))
    
        #viewer.doc.addRenderer('rd_DesPosL', yr.PointsRenderer(rd_DesPosL, (0,0,255)))
        #viewer.doc.addRenderer('rd_DesPosR', yr.PointsRenderer(rd_DesPosR, (0,100,255)))
    
        #viewer.doc.addRenderer('rd_DesForePosL', yr.PointsRenderer(rd_DesForePosL, (150,0,200)))
        #viewer.doc.addRenderer('rd_DesForePosR', yr.PointsRenderer(rd_DesForePosR, (150,0,250)))
        #viewer.doc.addRenderer('rd_DesRearPosL', yr.PointsRenderer(rd_DesRearPosL, (0,150,200)))
        #viewer.doc.addRenderer('rd_DesRearPosR', yr.PointsRenderer(rd_DesRearPosR, (0,150,250)))

        #viewer.doc.addRenderer('softConstraint', yr.VectorsRenderer(rd_soft_const_vec, rd_CMP, (150,100,100), 3))
        
        #viewer.doc.addRenderer('rd_footLVec', yr.VectorsRenderer(rd_footL_vec, rd_footCenterL, (255,0,0), 3))
        #viewer.doc.addRenderer('rd_footRVec', yr.VectorsRenderer(rd_footR_vec, rd_footCenterR, (255,255,0), 3))

        #viewer.doc.addRenderer('rd_footCenter_ref', yr.PointsRenderer(rd_footCenter_ref))    
        #viewer.doc.addRenderer('rd_CM_plane_ref', yr.PointsRenderer(rd_CM_plane_ref, (255,255,0)))
        
        #viewer.doc.addRenderer('rd_refNormalVec', yr.VectorsRenderer(normalVector, rd_footCenter_ref, (255,0,0), 3))
        #viewer.doc.addRenderer('rd_refCMVec', yr.VectorsRenderer(rd_CM_ref_vec, rd_footCenter_ref, (255,0,255), 3))
    
        #viewer.doc.addRenderer('rd_curNormalVec', yr.VectorsRenderer(normalVector, rd_footCenter, (255,0,0), 3))
        #viewer.doc.addRenderer('rd_CMVec', yr.VectorsRenderer(rd_CM_vec, rd_footCenter, (255,0,255), 3))
            
        #viewer.doc.addRenderer('rd_contactForces', yr.ForcesRenderer(rd_contactForces, rd_contactPositions, (0,255,0), .009, 0.009))
    
        #viewer.doc.addRenderer('rd_virtualForce', yr.ForcesRenderer(rd_virtualForce, rd_CM, (50,255,0), 0.5, 0.02))
    
        #viewer.doc.addRenderer('rd_Joint', yr.PointsRenderer(rd_Joint, (255,0,0)))
        #viewer.doc.addRenderer('rd_Joint2', yr.PointsRenderer(rd_Joint2, (0,255,0)))
        #viewer.doc.addRenderer('rd_Joint3', yr.PointsRenderer(rd_Joint3, (0,0,255)))
        #viewer.doc.addRenderer('rd_Joint4', yr.PointsRenderer(rd_Joint4, (255,255,0)))

        viewer.doc.addRenderer('rd_desPoints', yr.PointsRenderer(rd_desPoints, (255,0,0)))

    stage = STATIC_BALANCING

    contactRendererName = []
    
    for i in range (motion[0].skeleton.getJointNum()):
        print(i, motion[0].skeleton.getJointName(i))       
    
    desCOMOffset = 0.0
    
    pt = [0.]

    timeReport = [0.]*7

    viewer.objectInfoWnd.comOffsetY.value(-0.05)
    viewer.objectInfoWnd.comOffsetZ.value(0.00)

    viewer.objectInfoWnd.begin()
    viewer.objectInfoWnd.Bc = Fl_Value_Input(100, 450, 40, 10, 'Bc')
    viewer.objectInfoWnd.Bc.value(0.1)

    viewer.objectInfoWnd.ankleAngleX = Fl_Value_Input(50,  510, 40, 10, 'Ankle X')
    viewer.objectInfoWnd.ankleAngleX.value(0)
    viewer.objectInfoWnd.ankleAngleY = Fl_Value_Input(110,  510, 40, 10, 'Y')
    viewer.objectInfoWnd.ankleAngleY.value(1)
    viewer.objectInfoWnd.ankleAngleZ = Fl_Value_Input(170, 510, 40, 10, 'Z')
    viewer.objectInfoWnd.ankleAngleZ.value(0)

    viewer.objectInfoWnd.end()
    viewer.objectInfoWnd.labelKt.value(50)
    viewer.objectInfoWnd.labelKk.value(17)




    config['Phalange'] = [  motion[0].skeleton.getJointIndex('LeftPhalange_1'),\
                            motion[0].skeleton.getJointIndex('LeftPhalange_2'),\
                            motion[0].skeleton.getJointIndex('RightPhalange_1'),\
                            motion[0].skeleton.getJointIndex('RightPhalange_2')]
    config['Talus'] = [ motion[0].skeleton.getJointIndex('LeftTalus_1'),\
                        motion[0].skeleton.getJointIndex('LeftTalus_2'),\
                        motion[0].skeleton.getJointIndex('RightTalus_1'),\
                        motion[0].skeleton.getJointIndex('RightTalus_2')]
    config['Calcaneus'] = [ motion[0].skeleton.getJointIndex('LeftCalcaneus_1'),\
                            motion[0].skeleton.getJointIndex('LeftCalcaneus_2'),\
                            motion[0].skeleton.getJointIndex('RightCalcaneus_1'),\
                            motion[0].skeleton.getJointIndex('RightCalcaneus_2')]
    pose = motion[0].copy()
    timeReport = [0.]*2

    def simulateCallback(frame):
        curTime = time.time()
        Ke = 0.0      
        Kt, Kk, Kl, Kh, Ksc, Bt, Bl, Bh, B_CM, B_CMSd, B_Toe = viewer.GetParam()        
        motionModel.update(motion[frame])
        controlToMotionOffset = [-2.0, 0., 0.]
        motionModel.translateByOffset(controlToMotionOffset)
        
        stepsPerFrame = 10
        for i in range(stepsPerFrame):
            Kt, Kk, Kl, Kh, Ksc, Bt, Bl, Bh, B_CM, B_CMSd, B_Toe = viewer.GetParam()
            #Kt, Kl, Kh, Bl, Bh, Ke = viewer.GetParam()
            #qps.setupWeight(Kt, Kl, Kh, Ke, Bt, Btau, Bcon, Bl, Bh, Be)
            qps.setupWeight(Kt, Kl, Kh, Ke, 10., .1, .1, Bl, Bh, 10.)
            cPositions, CP, CM, footCenter, dL_des, CM_ref= qps.setupQP(frame, motion, mcfg, controlModel, vpWorld, config, 1./(30.*stepsPerFrame))
            CM_ref[1] = 0.
            timeReport[0] += time.time() - curTime
            curTime = time.time()
            #forceforce = np.array([viewer.objectInfoWnd.labelForceX.value(), viewer.objectInfoWnd.labelForceY.value(), viewer.objectInfoWnd.labelForceZ.value()])
            #extraForce[0] = viewer.objectInfoWnd.labelFm.value() * mm.normalize2(forceforce)
            #extraForcePos[0] = controlModel.getBodyPositionGlobal(selectedBody)
            #if viewer.GetForceState() :
            #    qps.addExternalForces(extraForce[0], selectedBody, viewer.objectInfoWnd.labelForceDur.value());
            #    viewer.ResetForceState()
            x, cForce = qps.stepQP(controlModel, 1./(30.*stepsPerFrame))
            timeReport[1] += time.time() - curTime
            curTime = time.time()

        print timeReport
            
        if frame%30==0: print 'elapsed time for 30 frames:', time.time()-pt[0]
        # rendering        

        #rd_footCenter[0] = footCenter
        #
        #rd_CM[0] = CM.copy()
        #
        #rd_CM_plane[0] = CM_plane.copy()
        #
        #rd_footCenter_ref[0] = footCenter_ref
        #rd_CM_plane_ref[0] = CM_ref.copy()
        #rd_CM_ref[0] = CM_ref.copy()
        #rd_CM_ref_vec[0] = (CM_ref - footCenter_ref)*3.
        #rd_CM_vec[0] = (CM - CM_plane)
        #rd_CM_des[0] = CM_ref_plane.copy()
        #rd_CM_des[0][1] = .01

        #rd_CM_plane[0][1] = 0.
        
        #if CP!=None and dCP!=None:
#        #    rd_CP[0] = CP
#        #    rd_CP_des[0] = CP_des
#        #
#        #rd_dL_des_plane[0] = dL_des_plane
#        #rd_dH_des[0] = dH_des
#        #
#        #rd_grf_des[0] = totalNormalForce# - totalMass*mm.s2v(wcfg.gravity)#dL_des_plane - totalMass*mm.s2v(wcfg.gravity)
#        #        
#        #rd_exf_des[0] = applyedExtraForce[0]
#        #rd_root_des[0] = rootPos[0]
#
#        #rd_CMP[0] = softConstPoint
#
#        #rd_soft_const_vec[0] = controlModel.getBodyPositionGlobal(constBody)-softConstPoint
#
#
#        ##indexL = motion[0].skeleton.getJointIndex('Hips')
#        ##indexR = motion[0].skeleton.getJointIndex('Spine1')
#        #indexL = indexFootL[0]        
#        #indexR = indexFootR[0]
#
#        #curAng = [controlModel.getBodyOrientationGlobal(indexL)]                        
#        #curAngY = np.dot(curAng, np.array([0,0,1]))
#
#        #rd_footL_vec[0] = np.copy(curAngY[0])
#        #rd_footCenterL[0] = controlModel.getBodyPositionGlobal(indexL)
#        #        
#        #curAng = [controlModel.getBodyOrientationGlobal(indexR)]                        
#        #curAngY = np.dot(curAng, np.array([0,0,1]))
#
        #rd_footR_vec[0] = np.copy(curAngY[0])
        #rd_footCenterR[0] = controlModel.getBodyPositionGlobal(indexR)
        #
        #if (forceApplyFrame == 0) :
        #    applyedExtraForce[0] = [0, 0, 0]


    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
    
main()