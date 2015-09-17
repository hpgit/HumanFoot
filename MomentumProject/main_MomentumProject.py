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

import numpy.linalg as npl

import mtOptimize as mot
import mtInitialize as mit

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

def getDesFootLinearAcc(refModel, controlModel, footIndex, ModelOffset, CM_ref, CM, Kk, Dk, restPosY = 0.0) :
        
    desLinearAcc = [0,0,0]

    refPos = refModel.getBodyPositionGlobal(footIndex)  
    curPos = controlModel.getBodyPositionGlobal(footIndex)
    refVecL = refPos - CM_ref
    if stage == MOTION_TRACKING+10:
        refPos = CM + refVecL
        #refPos[1] += 0.05
        #refPos[0] -= 0.05
    elif stage == POWERFUL_BALANCING:
        refPos = copy.copy(curPos)
        refPos[1] = restPosY
    elif stage == DYNAMIC_BALANCING:
        refPos = CM + refVecL
    else:
        refPos[0] += ModelOffset[0]
        #refPos = copy.copy(curPos)
        #refPos[1] = restPosY
        
                                
    refVel = refModel.getBodyVelocityGlobal(footIndex) 
    curVel = controlModel.getBodyVelocityGlobal(footIndex)
    #refAcc = (0,0,0)
    refAcc = refModel.getBodyAccelerationGlobal(footIndex)
         
    if stage != MOTION_TRACKING:
        refPos[1] = restPosY#0.032
        #refPos[1] = 0.0416
        
        #refPos[1] = 0.0
        
    if refPos[1] < 0.0 :
        refPos[1] = restPosY#0.032
        #refPos[1] = 0.0416
        
        #refPos[1] = 0.0

    desLinearAcc = yct.getDesiredAcceleration(refPos, curPos, refVel, curVel, refAcc, Kk, Dk)         

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
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    
    footPartNum = config['FootPartNum']

    if footPartNum > 1:
        elasticity = 2000
        damping = 2*(elasticity**.5)
    
        springBody1 = 1
        springBody2 = 2        
        springBody1Pos = motionModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody1]))
        springBody2Pos = motionModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody2]))

        initialDist = mm.length(springBody1Pos - springBody2Pos)*1.
        node = mcfg.getNode(mit.LEFT_METATARSAL_1)
        initialDist -= node.width#0.084
        v1 = (-node.width*0.5,0.0,node.length*0.4)
        v2 = (node.width*0.5,0.0,node.length*0.4)
        #controlModel.setSpring(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody1]), motion[0].skeleton.getJointIndex(config['FootLPart'][springBody2]), elasticity, damping, v2, v1, initialDist)
        #controlModel.setSpring(motion[0].skeleton.getJointIndex(config['FootRPart'][springBody1]), motion[0].skeleton.getJointIndex(config['FootRPart'][springBody2]), elasticity, damping, v1, v2, initialDist)

        
        elasticity = 10
        damping = 2*(elasticity**.5)
        springBody1 = 3
        springBody2 = 4    
        node = mcfg.getNode(mit.LEFT_PHALANGE_1)
        springBody1Pos = motionModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody1]))
        springBody2Pos = motionModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody2]))
        initialDist = mm.length(springBody1Pos - springBody2Pos)*1.
        initialDist -= node.width#0.084
        v1 = (-node.width*0.5,0.0,-node.length*0.4)
        v2 = (node.width*0.5,0.0,-node.length*0.4)
        #controlModel.setSpring(motion[0].skeleton.getJointIndex(config['FootLPart'][springBody1]), motion[0].skeleton.getJointIndex(config['FootLPart'][springBody2]), elasticity, damping, v2, v1, initialDist)
        #controlModel.setSpring(motion[0].skeleton.getJointIndex(config['FootRPart'][springBody1]), motion[0].skeleton.getJointIndex(config['FootRPart'][springBody2]), elasticity, damping, v1, v2, initialDist)
        
    
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

    
    indexFootL = [None]*footPartNum
    indexFootR = [None]*footPartNum
    jFootL = [None]*footPartNum
    dJFootL = [None]*footPartNum
    jFootR = [None]*footPartNum
    dJFootR = [None]*footPartNum
    jointMasksFootL = [None]*footPartNum
    jointMasksFootR = [None]*footPartNum
    
    jAngFootL = [None]*footPartNum
    dJAngFootL = [None]*footPartNum
    jAngFootR = [None]*footPartNum
    dJAngFootR = [None]*footPartNum

    for i in range(footPartNum) :
        jFootL[i] = yjc.makeEmptyJacobian(DOFs, 1)
        dJFootL[i] = jFootL[i].copy()
        jFootR[i] = yjc.makeEmptyJacobian(DOFs, 1)
        dJFootR[i] = jFootR[i].copy()
        
        jAngFootL[i] = yjc.makeEmptyJacobian(DOFs, 1, False)
        dJAngFootL[i] = jAngFootL[i].copy()
        jAngFootR[i] = yjc.makeEmptyJacobian(DOFs, 1, False)
        dJAngFootR[i] = jAngFootR[i].copy()
        
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
        #viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (100,100,100), yr.POLYGON_FILL)) #(150,150,255)
        viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, CHARACTER_COLOR, yr.POLYGON_FILL))
        #viewer.doc.addRenderer('rd_footCenter', yr.PointsRenderer(rd_footCenter))    
        #viewer.doc.addRenderer('rd_footCenter_des', yr.PointsRenderer(rd_footCenter_des, (150,0,150))    )
        #viewer.doc.addRenderer('rd_footCenterL', yr.PointsRenderer(rd_footCenterL))  
        #viewer.doc.addRenderer('rd_footCenterR', yr.PointsRenderer(rd_footCenterR))
        viewer.doc.addRenderer('rd_CM_plane', yr.PointsRenderer(rd_CM_plane, (255,255,0)))
        viewer.doc.addRenderer('rd_CM', yr.PointsRenderer(rd_CM, (255,0,255)))
        viewer.doc.addRenderer('rd_CM_des', yr.PointsRenderer(rd_CM_des, (64,64,255)))
        viewer.doc.addRenderer('rd_CM_vec', yr.VectorsRenderer(rd_CM_vec, rd_CM_plane, (255,0,0), 3))
        #viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (0,255,0)))
        #viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (255,0,255)))
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
    
        viewer.doc.addRenderer('rd_Joint', yr.PointsRenderer(rd_Joint, (255,0,0)))
        viewer.doc.addRenderer('rd_Joint2', yr.PointsRenderer(rd_Joint2, (0,255,0)))
        viewer.doc.addRenderer('rd_Joint3', yr.PointsRenderer(rd_Joint3, (0,0,255)))
        viewer.doc.addRenderer('rd_Joint4', yr.PointsRenderer(rd_Joint4, (255,255,0)))

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
    viewer.objectInfoWnd.end()
    viewer.objectInfoWnd.labelKt.value(50)
    viewer.objectInfoWnd.labelKk.value(17)
    def simulateCallback(frame):              

        curTime = time.time()


        if frame%30==1: pt[0] = time.time()

        global g_initFlag
        global forceShowFrame
        global forceApplyFrame
        global JsysPre
        global JsupPreL
        global JsupPreR
        global JsupPre
        global softConstPoint
        global stage
        global contactRendererName
        global desCOMOffset

        motionModel.update(motion[frame])

        Kt, Kk, Kl, Kh, Ksc, Bt, Bl, Bh, B_CM, B_CMSd, B_Toe = viewer.GetParam()
        
        Dt = 2*(Kt**.5)
        Dk = 2*(Kk**.5)
        Dl = 2*(Kl**.5)
        Dh = 2*(Kh**.5)
        Dsc = 2*(Ksc**.5)
                
        '''
        if Bsc == 0.0 :
            viewer.doc.showRenderer('softConstraint', False)
            viewer.motionViewWnd.update(1, viewer.doc)
        else:
            viewer.doc.showRenderer('softConstraint', True)
            renderer1 = viewer.doc.getRenderer('softConstraint')
            renderer1.rc.setLineWidth(0.1+Bsc*3)
            viewer.motionViewWnd.update(1, viewer.doc)
        '''
                        
        # tracking
        th_r_ori = motion.getDOFPositions(frame)
        th_r = copy.copy(th_r_ori)


       ############################
        #Reference motion modulation
        
        '''
        for i in range (motion[0].skeleton.getJointNum()):
            print(i, motion[0].skeleton.getJointName(i))  
        '''
        dCM_k = 10.
        linkVelocities = controlModel.getBodyVelocitiesGlobal()
        dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)
        dCM_plane = copy.copy(dCM); dCM_plane[1]=0.
        #B_CM = dCM_plane[1] * dCM_k

        #newR = mm.ZYX2R([0, 0, 1*B_CM])
        newR = mm.exp(mm.v3(1.0,0.0,0.0), 3.14*0.5*B_CM/100.)
        newR2 = mm.exp(mm.v3(1.0,0.0,0.0), -3.14*0.2*B_CM/100.)

        toeR = mm.exp(mm.v3(1.0,0.0,0.0), 3.14*0.25*B_Toe/100.)        
        
        if B_CM < 0.0:
            idxs = ['LeftTalus_1', 'LeftTalus_2', 'LeftTalus_3', 'RightTalus_1', 'RightTalus_2', 'RightTalus_3' ]
            idxs = ['LeftMetatarsal_1', 'LeftMetatarsal_2', 'LeftMetatarsal_3', 'RightMetatarsal_1', 'RightMetatarsal_2', 'RightMetatarsal_3' ]        
            #idxs = ['LeftPhalange_1', 'LeftPhalange_2', 'LeftPhalange_3', 'RightPhalange_1', 'RightPhalange_2', 'RightPhalange_3' ]
            idxs2 = ['LeftMetatarsal_1', 'LeftMetatarsal_2', 'LeftMetatarsal_3', 'RightMetatarsal_1', 'RightMetatarsal_2', 'RightMetatarsal_3' ]        
        else :            
            idxs = ['LeftTalus_1', 'LeftTalus_2', 'LeftTalus_3', 'RightTalus_1', 'RightTalus_2', 'RightTalus_3' ]
            idxs = ['LeftMetatarsal_1', 'LeftMetatarsal_2', 'LeftMetatarsal_3', 'RightMetatarsal_1', 'RightMetatarsal_2', 'RightMetatarsal_3' ]        
            #idxs = ['LeftPhalange_1', 'LeftPhalange_2', 'LeftPhalange_3', 'RightPhalange_1', 'RightPhalange_2', 'RightPhalange_3' ]
            idxs2 = ['LeftPhalange_1', 'LeftPhalange_2', 'LeftPhalange_3', 'RightPhalange_1', 'RightPhalange_2', 'RightPhalange_3' ]

        idxs2 = ['LeftTalus_1', 'LeftTalus_2', 'LeftTalus_3', 'RightTalus_1', 'RightTalus_2', 'RightTalus_3' ]
        idxs = ['LeftMetatarsal_1', 'LeftMetatarsal_2', 'LeftMetatarsal_3', 'RightMetatarsal_1', 'RightMetatarsal_2', 'RightMetatarsal_3' ]        
        idxs3 = ['LeftPhalange_1', 'LeftPhalange_2', 'LeftPhalange_3', 'RightPhalange_1', 'RightPhalange_2', 'RightPhalange_3' ]

        #idxs2 = ['LeftTalus_1', 'LeftTalus_3', 'RightTalus_1', 'RightTalus_3' ]
        #idxs = ['LeftMetatarsal_1', 'LeftMetatarsal_3', 'RightMetatarsal_1', 'RightMetatarsal_3' ]
        #idxs3 = ['LeftPhalange_1', 'LeftPhalange_3', 'RightPhalange_1', 'RightPhalange_3' ]

        for i in range(len(idxs)):
            idx = motion[0].skeleton.getJointIndex(idxs[i])
            th_r[idx] = np.dot(th_r[idx], newR)
            idx2 = motion[0].skeleton.getJointIndex(idxs2[i])
            th_r[idx2] = np.dot(th_r[idx2], newR2)
            
            idx3 = motion[0].skeleton.getJointIndex(idxs3[i])
            th_r[idx3] = np.dot(th_r[idx3], toeR)

            
        newR1 = mm.exp(mm.v3(1.0,0.0,0.0), 3.14*0.5*B_CMSd/100.)
        newR2 = mm.exp(mm.v3(1.0,0.0,0.0), 3.14*0.25*B_CMSd/100.)
        newR3 = mm.exp(mm.v3(1.0,0.0,0.0), 3.14*0.125*B_CMSd/100.)
        
        #idxs = ['LeftMetatarsal_1', 'LeftMetatarsal_2', 'LeftMetatarsal_3', 'RightMetatarsal_3', 'RightMetatarsal_2', 'RightMetatarsal_1' ]

        if B_CMSd > 0.0:
            idxs = ['LeftTalus_1', 'LeftTalus_2', 'LeftTalus_3', 'LeftCalcaneus_1', 'LeftCalcaneus_2', 'LeftCalcaneus_3']
        else:
            idxs = ['RightTalus_3', 'RightTalus_2', 'RightTalus_1', 'RightCalcaneus_3', 'RightCalcaneus_2', 'RightCalcaneus_1']
        
        for i in range(0, len(idxs), 3):
            idx = motion[0].skeleton.getJointIndex(idxs[i])
            th_r[idx] = np.dot(th_r[idx], newR1)
            idx = motion[0].skeleton.getJointIndex(idxs[i+1])
            th_r[idx] = np.dot(th_r[idx], newR2)
            idx = motion[0].skeleton.getJointIndex(idxs[i+2])
            th_r[idx] = np.dot(th_r[idx], newR3)
        
        global leftHipTimer
        if viewer.objectInfoWnd.onLeftHip:
            leftHipTimer = 45
            viewer.objectInfoWnd.onLeftHip = False
        if leftHipTimer > 0:
            viewer.objectInfoWnd.comOffsetX.value(0.04*np.sin(2*3.14*leftHipTimer/45.))
            viewer.objectInfoWnd.comOffsetZ.value(0.02*np.cos(2*3.14*leftHipTimer/45.))
            #B_Hipd = viewer.objectInfoWnd.labelLeftHip.value()
            #newR1 = mm.exp(mm.v3(0.0,1.0,0.0), 3.14*0.5*B_Hipd/100.)
            #idx = motion[0].skeleton.getJointIndex('LeftUpLeg')
            #th_r[idx] = np.dot(th_r[idx], newR1)
            #idx = motion[0].skeleton.getJointIndex('RightUpLeg')
            #th_r[idx] = np.dot(th_r[idx], newR1)
            leftHipTimer -= 1


        timeReport[0] += time.time() -curTime
        curTime = time.time()
    
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        ddth_c = controlModel.getDOFAccelerations()

        ype.flatten(ddth_des, ddth_des_flat)
        ype.flatten(dth, dth_flat)

        ype.flatten(ddth_c, ddth_c_flat)
        
        # jacobian                                 
        refFootL = motionModel.getBodyPositionGlobal(supL)        
        refFootR = motionModel.getBodyPositionGlobal(supR)
               
        positionFootL = [None]*footPartNum
        positionFootR = [None]*footPartNum
        for i in range(footPartNum):
            positionFootL[i] = controlModel.getBodyPositionGlobal(indexFootL[i])
            positionFootR[i] = controlModel.getBodyPositionGlobal(indexFootR[i])
        
        linkPositions = controlModel.getBodyPositionsGlobal()
        linkVelocities = controlModel.getBodyVelocitiesGlobal()
        linkAngVelocities = controlModel.getBodyAngVelocitiesGlobal()
        linkInertias = controlModel.getBodyInertiasGlobal()

        jointPositions = controlModel.getJointPositionsGlobal()
        jointAxeses = controlModel.getDOFAxeses()
        
        CM = yrp.getCM(linkPositions, linkMasses, totalMass)
        dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)
        CM_plane = copy.copy(CM); CM_plane[1]=0.
        dCM_plane = copy.copy(dCM); dCM_plane[1]=0.
                
        linkPositions_ref = motionModel.getBodyPositionsGlobal()
        linkVelocities_ref = motionModel.getBodyVelocitiesGlobal() 
        linkAngVelocities_ref = motionModel.getBodyAngVelocitiesGlobal()
        linkInertias_ref = motionModel.getBodyInertiasGlobal()

        CM_ref = yrp.getCM(linkPositions_ref, linkMasses, totalMass)
        CM_plane_ref = copy.copy(CM_ref)
        CM_plane_ref[1] = 0.

        
        
        P = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions, CM, linkInertias)
        dP = ymt.getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities, dCM, linkAngVelocities, linkInertias)

        timeReport[1] += time.time() -curTime
        curTime = time.time()

        yjc.computeJacobian2(Jsys, DOFs, jointPositions, jointAxeses, linkPositions, allLinkJointMasks)       
        timeReport[2] += time.time() -curTime
        curTime = time.time()
        
        # yjc.computeJacobianDerivative2(dJsys, DOFs, jointPositions, jointAxeses, linkAngVelocities, linkPositions, allLinkJointMasks)
        if frame > 0:
            dJsys = (Jsys - JsysPre)*30.
        else:
            dJsys = (Jsys - Jsys)
        JsysPre = Jsys.copy()

        timeReport[3] += time.time() -curTime
        curTime = time.time()


        bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
        CP = yrp.getCP(contactPositions, contactForces)

        for i in range(len(bodyIDsToCheck)) :
            controlModel.SetBodyColor(bodyIDsToCheck[i], 0, 0, 0, 255)
        


        contactFlagFootL = [0]*footPartNum
        contactFlagFootR = [0]*footPartNum
        partialDOFIndex = [22, 22]

        for i in range(len(bodyIDs)) :
            controlModel.SetBodyColor(bodyIDs[i], 255, 105, 105, 200)
            index = controlModel.id2index(bodyIDs[i])
            for j in range(len(indexFootL)):
                if index == indexFootL[j]:
                    contactFlagFootL[j] = 1
            for j in range(len(indexFootR)):
                if index == indexFootR[j]:
                    contactFlagFootR[j] = 1

        for j in range(0, footPartNum):
            jAngFootR[j] = Jsys[6*indexFootR[j] : 6*indexFootR[j]+6][3:]#.copy()
            jAngFootL[j] = Jsys[6*indexFootL[j] : 6*indexFootL[j]+6][3:]#.copy()
            dJAngFootR[j] = dJsys[6*indexFootR[j] : 6*indexFootR[j]+6][3:]#.copy()
            dJAngFootL[j] = dJsys[6*indexFootL[j] : 6*indexFootL[j]+6][3:]#.copy()
            jFootR[j] = Jsys[6*indexFootR[j] : 6*indexFootR[j]+6]#.copy()
            jFootL[j] = Jsys[6*indexFootL[j] : 6*indexFootL[j]+6]#.copy()
            dJFootR[j] = dJsys[6*indexFootR[j] : 6*indexFootR[j]+6]#.copy()
            dJFootL[j] = dJsys[6*indexFootL[j] : 6*indexFootL[j]+6]#.copy()
        if footPartNum == 1:
            desFCL = (controlModel.getBodyPositionGlobal(supL))
            desFCR = (controlModel.getBodyPositionGlobal(supR))
        else :
            r = .5+desCOMOffset
            desFCL = (controlModel.getBodyPositionGlobal(indexFootL[0])*r + controlModel.getBodyPositionGlobal(indexFootL[1])*(1.0-r))#controlModel.getBodyPositionGlobal(indexFootL[1])
            desFCR = (controlModel.getBodyPositionGlobal(indexFootR[0])*r + controlModel.getBodyPositionGlobal(indexFootR[1])*(1.0-r))#controlModel.getBodyPositionGlobal(indexFootR[1])
        desFC = desFCL + (desFCR - desFCL)/2.0  
        desFC[1] = 0
        rd_footCenter_des[0] = desFC.copy()
        curRelCMVec = CM_plane - desFC
        vecRatio = mm.length(curRelCMVec)*0.
        #print(frame, vecRatio)
        footCenter = desFC - curRelCMVec*(vecRatio)#/10.0

        footCenter = (getBodyGlobalPos(controlModel, motion, 'LeftCalcaneus_1') + getBodyGlobalPos(controlModel, motion, 'LeftPhalange_1') + getBodyGlobalPos(controlModel, motion, 'RightCalcaneus_1') + getBodyGlobalPos(controlModel, motion, 'RightPhalange_1'))/4.
        #footCenter = (getBodyGlobalPos(controlModel, motion, 'LeftCalcaneus_1') + getBodyGlobalPos(controlModel, motion, 'LeftTalus_1') + getBodyGlobalPos(controlModel, motion, 'RightCalcaneus_1') + getBodyGlobalPos(controlModel, motion, 'RightTalus_1'))/4.

        footCenter_ref = refFootL + (refFootR - refFootL)/2.0
        #footCenter_ref[1] = 0.    
        footCenter[1] = 0.  
        footCenterOffset = np.array([viewer.objectInfoWnd.comOffsetX.value(), 0, viewer.objectInfoWnd.comOffsetZ.value()])
        #footCenter += footCenterOffset
        
        vecRatio = mm.length(curRelCMVec)*0.
        softConstPointOffset = -curRelCMVec*(vecRatio)#/10.0
        #print(frame, vecRatio, softConstPointOffset)

        desForeSupLAcc = [0,0,0]
        desForeSupRAcc = [0,0,0]
                
        totalNormalForce = [0,0,0]    
        
        for i in range(len(contactForces)):
            totalNormalForce[0] += contactForces[i][0]
            totalNormalForce[1] += contactForces[i][1]
            totalNormalForce[2] += contactForces[i][2]
                                   
        #print((totalMass*mm.s2v(wcfg.gravity))[1])

        footCenterOffset = np.array([viewer.objectInfoWnd.comOffsetX.value(), viewer.objectInfoWnd.comOffsetY.value(), viewer.objectInfoWnd.comOffsetZ.value()])



        ######################
        # optimization terms
        ######################

        # linear momentum
        CM_ref_plane = footCenter + footCenterOffset
        dL_des_plane = Kl*totalMass*(CM_ref_plane - CM_plane) - Dl*totalMass*dCM_plane
        dL_des_plane[1] = Kl*totalMass*(CM_ref[1] + footCenterOffset[1] - CM[1]) - Dl*totalMass*dCM[1]
    
        # angular momentum
        CP_ref = footCenter + footCenterOffset

        timeStep = 30.
        if CP_old[0]==None or CP==None:
            dCP = None
        else:
            dCP = (CP - CP_old[0])/(1/timeStep)
        CP_old[0] = CP            
        
        if CP!=None and dCP!=None:
            ddCP_des = Kh*(CP_ref - CP) - Dh*(dCP)
            CP_des = CP + dCP*(1/timeStep) + .5*ddCP_des*((1/timeStep)**2)
            #dH_des = np.cross((CP_des - CM), (dL_des_plane + totalMass*mm.s2v(wcfg.gravity)))            
            dH_des = np.cross((CP_des - CM_plane), (dL_des_plane + totalMass*mm.s2v(wcfg.gravity)))
        else:
            dH_des = None
 
        # momentum matrix
        RS = np.dot(P, Jsys)
        R, S = np.vsplit(RS, 2)
        
        rs = np.dot((np.dot(dP, Jsys) + np.dot(P, dJsys)), dth_flat)
        r_bias, s_bias = np.hsplit(rs, 2)

         
        flagContact = True
        if dH_des==None or np.any(np.isnan(dH_des)) == True:
            flagContact = False 
            #viewer.doc.showRenderer('rd_grf_des', False)
            #viewer.motionViewWnd.update(1, viewer.doc)
        #else:
            #viewer.doc.showRenderer('rd_grf_des', True)
            #viewer.motionViewWnd.update(1, viewer.doc)
        
        '''
        0 : initial
        1 : contact
        2 : fly
        3 : landing
        '''

        #MOTION = FORWARD_JUMP
        if mit.MOTION == mit.FORWARD_JUMP :
            frame_index = [136, 100]
            #frame_index = [100000, 100000]
        elif mit.MOTION == mit.TAEKWONDO:
            frame_index = [130, 100]
            #frame_index = [100000, 100000]
        elif mit.MOTION == mit.TAEKWONDO2:
            frame_index = [130+40, 100]
        elif mit.MOTION == mit.WALK:
            frame_index = [10000, 60]
        elif mit.MOTION == mit.TIPTOE:
            frame_index = [1000000, 1000000]
            #frame_index = [10000, 165]
        else :
            frame_index = [1000000, 1000000]
        
        #MOTION = TAEKWONDO 
        #frame_index = [135, 100]

        '''
        if frame > 300 :
            if stage != DYNAMIC_BALANCING:
                print("#", frame,"-DYNAMIC_BALANCING")
            stage = DYNAMIC_BALANCING
            Kk = Kk*1
            Dk = 2*(Kk**.5)        
        '''
        if frame > frame_index[0] :
            if stage != POWERFUL_BALANCING:
                print("#", frame,"-POWERFUL_BALANCING")
            stage = POWERFUL_BALANCING
            Kk = Kk*2
            Dk = 2*(Kk**.5)            
        elif frame > frame_index[1]:
            if stage != MOTION_TRACKING:
                print("#", frame,"-MOTION_TRACKING")
            stage = MOTION_TRACKING

        trackingW = w

        #if checkAll(contactFlagFootR, 0) != 1 :
        if 0:#stage == MOTION_TRACKING:
            trackingW = w2
            #stage = POWERFUL_BALANCING
            Bt = Bt*2

        # optimization
                
        mot.addTrackingTerms(problem, totalDOF, Bt, trackingW, ddth_des_flat)
                
        #mot.addSoftPointConstraintTerms(problem, totalDOF, Bsc, ddP_des1, Q1, q_bias1)

        if flagContact == True:
            if stage != MOTION_TRACKING+10:
                mot.addLinearTerms(problem, totalDOF, Bl, dL_des_plane, R, r_bias) 
                mot.addAngularTerms(problem, totalDOF, Bh, dH_des, S, s_bias)
            
        a_sup_2 = None
        Jsup_2  = None
        dJsup_2 = None

        ##############################
        # Hard constraint        
        if stage != MOTION_TRACKING:
            Kk2 = Kk * 2.0
        else :
            Kk2 = Kk * 1.5
                    
        Dk2 = 2*(Kk2**.5)

        ankleW = 0
        ankleOffset = ankleW*curRelCMVec[2]
        metatarW = 0
        metatarOffset = metatarW*curRelCMVec[2]
        
        
        ##############################
        
        ##############################
        # Additional constraint      

        if stage != MOTION_TRACKING:
            # ankle strategy
            idx = 0 #LEFT/RIGHT_TOES 
            if mit.FOOT_PART_NUM == 1 :
                yOffset = 0.03
            else :
                yOffset = 0.069
            # ankleOffset = (footCenter - CM_plane)*4.
            ankleOffset = -footCenterOffset*10.
            ankleOffset[1] = 0.
            ankleOffsetL = ankleOffset.copy()
            ankleOffsetR = ankleOffset.copy()
            #ankleOffset= np.array((0,0,0))

            ankleOffsetL[0] = - ankleOffsetL[0]
            ankleOffsetR[0] = - ankleOffsetR[0]
            if footCenterOffset[0] > 0.0:
                ankleOffsetL[0] = 0.
            else:
                ankleOffsetR[0] = 0.

            print 'ankleOffset=', ankleOffset
                
            desLinearAccL, desPosL = getDesFootLinearAcc(motionModel, controlModel, indexFootL[idx], ModelOffset, CM_ref, CM, Kk2, Dk2, yOffset)#0.076) #0.14)
            desLinearAccR, desPosR = getDesFootLinearAcc(motionModel, controlModel, indexFootR[idx], ModelOffset, CM_ref, CM, Kk2, Dk2, yOffset)
                                
            ax = [0,0,-1]
            if mit.FOOT_PART_NUM == 1 :
                ax = [0,1,0]
                
            desAngularAccL = getDesFootAngularAcc(motionModel, controlModel, indexFootL[idx], Kk2, Dk2, ax, [0,1,0]+ankleOffsetL)
            desAngularAccR = getDesFootAngularAcc(motionModel, controlModel, indexFootR[idx], Kk2, Dk2, ax, [0,1,0]+ankleOffsetR)
                                
            a_sup_2 = np.hstack(( np.hstack((desLinearAccL, desAngularAccL)), np.hstack((desLinearAccR, desAngularAccR)) )) 
            Jsup_2 = np.vstack((jFootL[idx], jFootR[idx]))
            dJsup_2 = np.vstack((dJFootL[idx], dJFootR[idx]))   
            #mot.addConstraint(problem, totalDOF, Jsup_2, dJsup_2, dth_flat, a_sup_2)
            #mot.addConstraint(problem, totalDOF, Jsup_2[:1], dJsup_2[:1], dth_flat, a_sup_2[:1])
            #mot.addConstraint(problem, totalDOF, Jsup_2[2:], dJsup_2[2:], dth_flat, a_sup_2[2:])
            #mot.addConstraint(problem, totalDOF, Jsup_2[3:], dJsup_2[3:], dth_flat, a_sup_2[3:])
            #mot.addAnotherTerms(problem, totalDOF, viewer.objectInfoWnd.Bc.value(), Jsup_2[3:], a_sup_2[3:] - np.dot(dJsup_2[3:] , dth_flat))
            mot.addAnotherTerms(problem, totalDOF, viewer.objectInfoWnd.Bc.value(), Jsup_2[0:0], a_sup_2[0:0] - np.dot(dJsup_2[0:0] , dth_flat))
            mot.addAnotherTerms(problem, totalDOF, viewer.objectInfoWnd.Bc.value(), Jsup_2[2:], a_sup_2[2:] - np.dot(dJsup_2[2:] , dth_flat))
            desCOMOffset = 0.0
            
            rd_DesPosL[0] = desPosL.copy()
            rd_DesPosR[0] = desPosR.copy()

    
        
        if stage == STATIC_BALANCING:
            # foot strategy
            #Kk2 = Kk * 2.5
            Kk2 = Kk #* 1.5
            Dk2 = 2*(Kk2**.5)
            desForePosL = [0,0,0]
            desForePosR = [0,0,0]
            desRearPosL = [0,0,0]
            desRearPosR = [0,0,0]
            footPartPos = []
            footPartPos.append(controlModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex('LeftCalcaneus_1')))
            footPartPos.append(controlModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex('LeftPhalange_1')))
            footPartPos.append(controlModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex('RightCalcaneus_1')))
            footPartPos.append(controlModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex('RightPhalange_1')))
            #for i in range(1, 4) : 
            #    # Calcaneus
            #    contactFlagFootL[i] = 1
            #    contactFlagFootR[i] = 1
            #for i in range(4, 7) : 
            #    # Talus
            #    contactFlagFootL[i] = 0
            #    contactFlagFootR[i] = 0
            #for i in range(7, 10) : 
            #    # Metatarsal
            #    contactFlagFootL[i] = 0
            #    contactFlagFootR[i] = 0
            #for i in range(10, footPartNum) : 
            #    # Phalange
            #    contactFlagFootL[i] = 1
            #    contactFlagFootR[i] = 1
            for i in range(1, footPartNum) : 
                contactFlagFootL[i] = 1
                contactFlagFootR[i] = 1
            SupPts = np.vstack((np.array((footPartPos[0][0], footPartPos[1][0], footPartPos[2][0], footPartPos[3][0])), 
                   np.array((footPartPos[0][2], footPartPos[1][2], footPartPos[2][2], footPartPos[3][2])), 
                   np.array((1., 1., 1., 1.))))
            
            coordWidthLen = 3.
            coordLengthLen = 2.
            SupUV = np.vstack((np.array((-coordWidthLen, -coordWidthLen, coordWidthLen, coordWidthLen)), np.array((-coordLengthLen, coordLengthLen, -coordLengthLen, coordLengthLen)), np.array((1., 1., 1., 1.))))
            SupMap = np.dot(np.dot(SupUV, SupUV.T), np.linalg.inv(np.dot(SupPts, SupUV.T)))
            #print SupMap
            desFootCenter = footCenter + footCenterOffset
            footCenterPts = np.array((desFootCenter[0], desFootCenter[2], 1))

            #print np.dot(SupMap, footCenterPts)
            #print np.dot(getBodyGlobalOri(controlModel, motion, 'LeftMetatarsal_1'), np.array((0,1,0)))

            CM_plane_2D = np.array( (CM[0], CM[2], 1) )
            # CM_plane_UV = np.dot(SupMap, CM_plane_2D)
            CM_plane_UV = np.dot(SupMap, footCenterPts)
            # print CM_plane_UV
            # for i in range(1, footPartNum):
            if CM_plane_UV[1] > 1.:
                for i in range(1, 7):
                    contactFlagFootL[i] = 0
                    contactFlagFootR[i] = 0
            elif CM_plane_UV[1] < -1.:
                for i in range(7, footPartNum):
                    contactFlagFootL[i] = 0
                    contactFlagFootR[i] = 0
            else:
                for i in range(4, 10):
                    contactFlagFootL[i] = 0
                    contactFlagFootR[i] = 0

            contactFlagFoot = contactFlagFootL
            if CM_plane_UV[0] < 0.:
                contactFlagFoot = contactFlagFootR
                # CM_plane_UV[0] = -CM_plane_UV[0]

            if abs(CM_plane_UV[0]) > 1.:
                for j in range(0, 4):
                    contactFlagFoot[3*j+3] = 0
            if abs(CM_plane_UV[0]) > 2.:
                for j in range(0, 4):
                    contactFlagFoot[3*j+2] = 0

            # print 'footL : ',contactFlagFootL
            # print 'footR : ',contactFlagFootR

            for i in range(1, footPartNum) :
                axis = [0,1,0]
                desAng = [0,1,0]
                desY = 0.03
                if contactFlagFootL[i] == 1:
                    desLinearAccL, desForePosL = getDesFootLinearAcc(motionModel, controlModel, indexFootL[i], ModelOffset, CM_ref, CM, Kk2, Dk2, desY) 
                    desAngularAccL = getDesFootAngularAcc(motionModel, controlModel, indexFootL[i], Kk2, Dk2, axis, desAng)
                    a_sup_2 = np.hstack((desLinearAccL, desAngularAccL))
                    Jsup_2 = jFootL[i].copy()
                    dJsup_2 = dJFootL[i].copy()
                    #mot.addConstraint(problem, totalDOF, Jsup_2, dJsup_2, dth_flat, a_sup_2)      
                    mot.addAnotherTerms(problem, totalDOF, viewer.objectInfoWnd.Bc.value(), Jsup_2, a_sup_2 - np.dot(dJsup_2, dth_flat))
                    #mot.addAnotherTerms(problem, totalDOF, viewer.objectInfoWnd.Bc.value(), Jsup_2[3:], a_sup_2[3:] - np.dot(dJsup_2[3:] , dth_flat))

                if contactFlagFootR[i] == 1:
                    desLinearAccR, desForePosR = getDesFootLinearAcc(motionModel, controlModel, indexFootR[i], ModelOffset, CM_ref, CM, Kk2, Dk2, desY) 
                    desAngularAccR = getDesFootAngularAcc(motionModel, controlModel, indexFootR[i], Kk2, Dk2, axis, desAng)
                    a_sup_2 = np.hstack((desLinearAccR, desAngularAccR))
                    Jsup_2 = jFootR[i].copy()
                    dJsup_2 = dJFootR[i].copy()
                    #mot.addConstraint(problem, totalDOF, Jsup_2, dJsup_2, dth_flat, a_sup_2)      
                    mot.addAnotherTerms(problem, totalDOF, viewer.objectInfoWnd.Bc.value(), Jsup_2, a_sup_2 - np.dot(dJsup_2, dth_flat))
                    #mot.addAnotherTerms(problem, totalDOF, viewer.objectInfoWnd.Bc.value(), Jsup_2[3:], a_sup_2[3:] - np.dot(dJsup_2[3:], dth_flat))
                    

            rd_DesForePosL[0] = desForePosL
            rd_DesForePosR[0] = desForePosR
            rd_DesRearPosL[0] = desRearPosL
            rd_DesRearPosR[0] = desRearPosR
        ##############################
        
        #if Jsup_2 != None:
        #    mot.addConstraint(problem, totalDOF, Jsup_2, dJsup_2, dth_flat, a_sup_2)
        

        timeReport[4] += time.time() -curTime
        curTime = time.time()


        r = problem.solve()
        problem.clear()
        ype.nested(r['x'], ddth_sol)
                      
        rootPos[0] = controlModel.getBodyPositionGlobal(selectedBody)
        localPos = [[0, 0, 0]]   


        ###########################################
        ##Jacobian Transpose control

        # COM Position control
        #fCom = Wcp*(pHatComDes - pHatCom) + Wcv*(vComDes - vCom) + Wcm*(footCenter_plane - CM_plane)

        w1 = 10#10.1
        w2 = 1#1#2*(w1**.5)

        if frame > 100 :
            w1 = 10.1#10.1
            w2 = 1
                
        footToCMVec = CM - footCenter
        desCMPos = [footCenter[0], mm.length(footToCMVec), footCenter[2]]
        #print("desCMPos", desCMPos)
        #print("CM", CM)
        fCom = w1*(desCMPos - CM) + w2*(-dCM)
        #print("fCom", fCom)
        #fCom[0] = 0.
        #fCom[1] = 0
        #fCom[2] = 0
        rd_virtualForce[0] = fCom.copy()

        #hipPos = controlModel.getBodyPositionGlobal(rootB)
        headPos = controlModel.getBodyPositionGlobal(selectedBody)
        hipPos = controlModel.getBodyPositionGlobal(rootB)
        yjc.computeJacobian2(Jcom, DOFs, jointPositions, jointAxeses, [headPos], comUpperJointMasks)
        #yjc.computeJacobianDerivative2(dJcom, DOFs, jointPositions, jointAxeses, linkAngVelocities, [CM], comUpperJointMasks, False)
        JcomT = Jcom.T
        TauJT = np.dot(JcomT, fCom)

        # Angular Momentum 
        Hc = ymt.getAngularMomentum(CM, linkInertias, linkAngVelocities, linkPositions, linkMasses, linkVelocities)
        Href = ymt.getAngularMomentum(CM_ref, linkInertias_ref, linkAngVelocities_ref, linkPositions_ref, linkMasses, linkVelocities_ref)

        Wam = .05
        Tam = Wam*(Href - Hc)
        #print("Tam", Tam)
        
        yjc.computeAngJacobian2(JcomAng, DOFs, jointPositions, jointAxeses, [headPos], comUpperJointMasks)        
        TauAM = np.dot(JcomAng.T, Tam)                        
        

        timeReport[5] += time.time() -curTime
        curTime = time.time()


        for i in range(stepsPerFrame):
            # apply penalty force
            bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
     
            vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)                      
            
            extraForce[0] = viewer.GetForce()
            if (extraForce[0][0] != 0 or extraForce[0][1] != 0 or extraForce[0][2] != 0) :
                forceApplyFrame += 1
                #vpWorld.applyPenaltyForce(selectedBodyId, localPos, extraForce)
                controlModel.applyBodyForceGlobal(selectedBody, extraForce[0])
                applyedExtraForce[0] = extraForce[0]
            
            if forceApplyFrame*wcfg.timeStep > 0.1:
                viewer.ResetForce()
                forceApplyFrame = 0            
                
            controlModel.setDOFAccelerations(ddth_sol)
            
            controlModel.solveHybridDynamics()            
            
            '''
            if (frame > 5):
                tau = controlModel.getJointTorqueLocal(indexFootL[3])
                tau2 = controlModel.getJointTorqueLocal(indexFootL[4])
                tau3 = controlModel.getJointTorqueLocal(indexFootR[3])
                tau4 = controlModel.getJointTorqueLocal(indexFootR[4])
                
                torques = controlModel.getInternalJointTorquesLocal()   
            
                if (frame > 100 and frame < 110) or (frame > 165 and frame < 190):
                    Wcal1 = 0.05
                    Wcal2 = 0.05
                    dC = fCom[2]*Wcal1-dCM[2]*Wcal2
                    print("dC", dC)
                    torques[indexFootL[5]-1]+= (dC, 0.0, 0.0)
                    torques[indexFootR[5]-1]+= (dC, 0.0, 0.0)

                if (frame > 50 and frame < 75) or (frame > 110 and frame <140) or (frame > 185 and frame < 220):
                    
                    metatarR = [controlModel.getBodyOrientationGlobal(indexFootL[1])]
                    phalangeR = [controlModel.getBodyOrientationGlobal(indexFootL[3])]
                    metatarR2 = np.dot(metatarR, np.array([0,0,1]))
                    phalangeR2 = np.dot(phalangeR, np.array([0,0,1]))

                    metatarRZ = mm.normalize(metatarR2[0])
                    phalangeRZ = mm.normalize(phalangeR2[0])
                    lean = np.dot(metatarRZ, phalangeRZ)
                    Wlean = 2
                    
                    dt = -0.02*(lean)*Wlean
                    torques[indexFootL[3]-1]+= (dt, 0.0, 0.0)
                    torques[indexFootL[4]-1]+= (dt, 0.0, 0.0)
                    torques[indexFootR[3]-1]+= (dt, 0.0, 0.0)
                    torques[indexFootR[4]-1]+= (dt, 0.0, 0.0)
                       
                                               
                i = 0
                t = 6
                while t < len(TauJT) :
                    torques[i] += (TauJT[t]+TauAM[t], TauJT[t+1]+TauAM[t+1], TauJT[t+2]+TauAM[t+2])
                    i+=1
                    t+=3

                #totalTorques = [a + b for a, b in zip(torques, TauJT)]
                #print("torques2", torques)
                #print("TauJT", TauJT[16], TauJT[17], TauJT[18])
                #print("torques", torques[16])
                #print("totalTorques", totalTorques[16])
                controlModel.setInternalJointTorquesLocal(torques)
            '''

            '''
            extraForce[0] = viewer.GetForce()
            if (extraForce[0][0] != 0 or extraForce[0][1] != 0 or extraForce[0][2] != 0) :
                forceApplyFrame += 1
                vpWorld.applyPenaltyForce(selectedBodyId, localPos, extraForce)
                applyedExtraForce[0] = extraForce[0]
            
            if forceApplyFrame*wcfg.timeStep > 0.1:
                viewer.ResetForce()
                forceApplyFrame = 0            
            '''
            vpWorld.step()                    
            
            
        if frame%30==0: print 'elapsed time for 30 frames:', time.time()-pt[0]
        # rendering        

        rd_footCenter[0] = footCenter
        
        rd_CM[0] = CM.copy()
        
        rd_CM_plane[0] = CM_plane.copy()
        
        rd_footCenter_ref[0] = footCenter_ref
        rd_CM_plane_ref[0] = CM_ref.copy()
        rd_CM_ref[0] = CM_ref.copy()
        rd_CM_ref_vec[0] = (CM_ref - footCenter_ref)*3.
        rd_CM_vec[0] = (CM - CM_plane)
        rd_CM_des[0] = CM_ref_plane.copy()
        rd_CM_des[0][1] = .01

        #rd_CM_plane[0][1] = 0.
        
        if CP!=None and dCP!=None:
            rd_CP[0] = CP
            rd_CP_des[0] = CP_des
        
        rd_dL_des_plane[0] = dL_des_plane
        rd_dH_des[0] = dH_des
        
        rd_grf_des[0] = totalNormalForce# - totalMass*mm.s2v(wcfg.gravity)#dL_des_plane - totalMass*mm.s2v(wcfg.gravity)
                
        rd_exf_des[0] = applyedExtraForce[0]
        rd_root_des[0] = rootPos[0]

        rd_CMP[0] = softConstPoint

        rd_soft_const_vec[0] = controlModel.getBodyPositionGlobal(constBody)-softConstPoint


        #indexL = motion[0].skeleton.getJointIndex('Hips')
        #indexR = motion[0].skeleton.getJointIndex('Spine1')
        indexL = indexFootL[0]        
        indexR = indexFootR[0]

        curAng = [controlModel.getBodyOrientationGlobal(indexL)]                        
        curAngY = np.dot(curAng, np.array([0,0,1]))

        rd_footL_vec[0] = np.copy(curAngY[0])
        rd_footCenterL[0] = controlModel.getBodyPositionGlobal(indexL)
                
        curAng = [controlModel.getBodyOrientationGlobal(indexR)]                        
        curAngY = np.dot(curAng, np.array([0,0,1]))

        rd_footR_vec[0] = np.copy(curAngY[0])
        rd_footCenterR[0] = controlModel.getBodyPositionGlobal(indexR)
        
        if (forceApplyFrame == 0) :
            applyedExtraForce[0] = [0, 0, 0]


        timeReport[6] += time.time() -curTime
        # print timeReport

    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
    
main()