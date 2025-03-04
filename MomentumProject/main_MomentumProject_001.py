import psyco; psyco.full()
from fltk import *
import copy
import numpy as np

import sys
#if '../PyCommon/modules' not in sys.path:
#    sys.path.append('../PyCommon/modules')
if './modules' not in sys.path:
    sys.path.append('./modules')
    
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

import Motion.ysHierarchyEdit as yme
import Simulator.ysPhysConfig as ypc

import numpy.linalg as npl

import mtOptimize as mot
import mtInitialize as mit

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


## Constant
STATIC_BALANCING = 0
MOTION_TRACKING = 1
DYNAMIC_BALANCING = 2
POWERFUL_BALANCING = 3
POWERFUL_MOTION_TRACKING = 4
FLYING = 5

def getDesFootLinearAcc(refModel, controlModel, footIndex, ModelOffset, CM_ref, CM, Kk, Dk) :
    desLinearAcc = [0,0,0]

    refPos = refModel.getBodyPositionGlobal(footIndex)  
    curPos = controlModel.getBodyPositionGlobal(footIndex)
    refVecL = refPos - CM_ref
    if stage == MOTION_TRACKING:
        refPos = CM + refVecL
        refPos[1] += 0.05
        refPos[0] -= 0.05
    elif stage == POWERFUL_BALANCING:
        refPos = copy.copy(curPos)
        refPos[1] = 0
    else:
        refPos[0] += ModelOffset[0]
                                
    refVel = refModel.getBodyVelocityGlobal(footIndex) 
    curVel = controlModel.getBodyVelocityGlobal(footIndex)
    #refAcc = (0,0,0)
    refAcc = refModel.getBodyAccelerationGlobal(footIndex)
         
    if stage != MOTION_TRACKING:
        refPos[1] = 0
        
    if refPos[1] < 0.0 :
        refPos[1] = 0.0

    desLinearAcc = yct.getDesiredAcceleration(refPos, curPos, refVel, curVel, refAcc, Kk, Dk)         

    return desLinearAcc

def getDesFootAngularAcc(refModel, controlModel, footIndex, Kk, Dk) :
    desAngularAcc = [0,0,0]

    curAng = [controlModel.getBodyOrientationGlobal(footIndex)]
    refAngVel = refModel.getBodyAngVelocityGlobal(footIndex)
    curAngVel = controlModel.getBodyAngVelocityGlobal(footIndex)
    refAngAcc = (0,0,0)
                        
    curAngY = np.dot(curAng, np.array([0,1,0]))
    refAngY = np.array([0,1,0])
    if stage == DYNAMIC_BALANCING:    
        refAng = [refModel.getBodyOrientationGlobal(footIndex)]
        refAngY2 = np.dot(refAng, np.array([0,1,0]))
        refAngY = refAngY2[0]
            
    aL = mm.logSO3(mm.getSO3FromVectors(curAngY[0], refAngY))
    desAngularAcc = Kk*aL + Dk*(refAngVel-curAngVel)

    return desAngularAcc

def main():

    np.set_printoptions(precision=4, linewidth=200)
    
#    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_5()
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped()
        
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    motionModel.recordVelByFiniteDiff()
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    controlModel.initializeHybridDynamics()
    
    ModelOffset = (1.5, -0.01, 0)
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
    w_IK = mot.getTrackingWeight(DOFs, motion[0].skeleton, config['IKweightMap'])
    supL =  motion[0].skeleton.getJointIndex(config['supLink'])
    supR =  motion[0].skeleton.getJointIndex(config['supLink2'])
    rootB = motion[0].skeleton.getJointIndex(config['root'])

    foreSupL = motion[0].skeleton.getJointIndex(config['leftForeFoot'])
    foreSupR = motion[0].skeleton.getJointIndex(config['rightForeFoot'])
    rearSupL = motion[0].skeleton.getJointIndex(config['leftRearFoot'])
    rearSupR = motion[0].skeleton.getJointIndex(config['rightRearFoot'])
    
    selectedBody = motion[0].skeleton.getJointIndex(config['end'])
    #constBody = motion[0].skeleton.getJointIndex('LeftForeArm')
    constBody = motion[0].skeleton.getJointIndex(config['const'])
    
    # jacobian 
    JsupL = yjc.makeEmptyJacobian(DOFs, 1)
    dJsupL = JsupL.copy()
    JsupPreL = JsupL.copy()

    JsupR = yjc.makeEmptyJacobian(DOFs, 1)
    dJsupR = JsupR.copy()
    JsupPreR = JsupR.copy()

    Jsup = yjc.makeEmptyJacobian(DOFs, 1)
    dJsup = Jsup.copy()
    JsupPre = Jsup.copy()

    Jsys = yjc.makeEmptyJacobian(DOFs, controlModel.getBodyNum())
    dJsys = Jsys.copy()
    JsysPre = Jsys.copy()
        
    Jconst = yjc.makeEmptyJacobian(DOFs, 1)
    dJconst = Jconst.copy()
        
    JforeSupL = yjc.makeEmptyJacobian(DOFs, 1)
    dJforeSupL = JforeSupL.copy()
    JforeSupR = yjc.makeEmptyJacobian(DOFs, 1)
    dJforeSupR = JforeSupR.copy()
    JrearSupL = yjc.makeEmptyJacobian(DOFs, 1)
    dJrearSupL = JrearSupL.copy()
    JrearSupR = yjc.makeEmptyJacobian(DOFs, 1)
    dJrearSupR = JrearSupR.copy()

    supLJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, supL)]
    supRJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, supR)]
    constJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, constBody)]
    allLinkJointMasks = yjc.getAllLinkJointMasks(motion[0].skeleton)
    
    foreSupLJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, foreSupL)]
    foreSupRJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, foreSupR)]
    rearSupLJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, rearSupL)]
    rearSupLJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, rearSupR)]
    
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
    rd_CM = [None]
    rd_CM_vec = [None]
    rd_CM_ref_vec = [None]
    rd_CP = [None]
    rd_CP_des = [None]
    rd_dL_des_plane = [None]
    rd_dH_des = [None]
    rd_grf_des = [None]
    
    rd_exf_des = [None]
    rd_root_des = [None]
    rd_soft_const_vec = [None]
    
    rd_root = [None]
    
    rd_footL_vec = [None]
    rd_footR_vec = [None]
        
    rd_CMP = [None]
    rd_DesPosL = [None]
    rd_DesPosR = [None]

    rootPos = [None]
    selectedBodyId = [selectedBody]
    extraForce = [None]
    applyedExtraForce = [None]
    applyedExtraForce[0] = [0,0,0]

    normalVector = [[0,2,0]]
    
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('rd_footCenter', yr.PointsRenderer(rd_footCenter))    
    #viewer.doc.addRenderer('rd_footCenterL', yr.PointsRenderer(rd_footCenterL))  
    #viewer.doc.addRenderer('rd_footCenterR', yr.PointsRenderer(rd_footCenterR))
    #viewer.doc.addRenderer('rd_CM_plane', yr.PointsRenderer(rd_CM_plane, (255,255,0)))
    viewer.doc.addRenderer('rd_CM', yr.PointsRenderer(rd_CM, (255,255,0)))
    viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (0,255,0)))
    #viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (255,0,255)))
#    viewer.doc.addRenderer('rd_dL_des_plane', yr.VectorsRenderer(rd_dL_des_plane, rd_CM, (255,255,0)))
#    viewer.doc.addRenderer('rd_dH_des', yr.VectorsRenderer(rd_dH_des, rd_CM, (0,255,0)))
    viewer.doc.addRenderer('rd_grf_des', yr.ForcesRenderer(rd_grf_des, rd_CP, (0,255,255), .001))

    viewer.doc.addRenderer('rd_exf_des', yr.ForcesRenderer(rd_exf_des, rd_root_des, (0,255,0), .009, 0.05))
        
    #viewer.doc.addRenderer('rd_CMP', yr.PointsRenderer(rd_CMP, (0,0,255)))
    
    viewer.doc.addRenderer('rd_DesPosL', yr.PointsRenderer(rd_DesPosL, (0,0,255)))
    viewer.doc.addRenderer('rd_DesPosR', yr.PointsRenderer(rd_DesPosR, (0,100,255)))

    #viewer.doc.addRenderer('softConstraint', yr.VectorsRenderer(rd_soft_const_vec, rd_CMP, (255,0,0), 3))
        
    viewer.doc.addRenderer('rd_footLVec', yr.VectorsRenderer(rd_footL_vec, rd_footCenterL, (255,0,0), 3))
    viewer.doc.addRenderer('rd_footRVec', yr.VectorsRenderer(rd_footR_vec, rd_footCenterL, (255,255,0), 3))

    #viewer.doc.addRenderer('rd_footCenter_ref', yr.PointsRenderer(rd_footCenter_ref))    
    viewer.doc.addRenderer('rd_CM_plane_ref', yr.PointsRenderer(rd_CM_plane_ref, (255,255,0)))
        
    viewer.doc.addRenderer('rd_refNormalVec', yr.VectorsRenderer(normalVector, rd_footCenter_ref, (255,0,0), 3))
    viewer.doc.addRenderer('rd_refCMVec', yr.VectorsRenderer(rd_CM_ref_vec, rd_footCenter_ref, (255,0,255), 3))
    
    viewer.doc.addRenderer('rd_curNormalVec', yr.VectorsRenderer(normalVector, rd_footCenter, (255,0,0), 3))
    viewer.doc.addRenderer('rd_CMVec', yr.VectorsRenderer(rd_CM_vec, rd_footCenter, (255,0,255), 3))
        
    stage = STATIC_BALANCING

    def simulateCallback(frame):
        global g_initFlag
        global forceShowFrame
        global forceApplyFrame
        global JsysPre
        global JsupPreL
        global JsupPreR
        global JsupPre
        global softConstPoint
        global stage

        motionModel.update(motion[frame])

        Kt, Kk, Kl, Kh, Ksc, Bt, Bl, Bh, Bsc = viewer.GetParam()
        
        Dt = 2*(Kt**.5)
        Dk = 2*(Kk**.5)
        Dl = 2*(Kl**.5)
        Dh = 2*(Kh**.5)
        Dsc = 2*(Ksc**.5)
                
        if Bsc == 0.0 :
            viewer.doc.showRenderer('softConstraint', False)
            viewer.motionViewWnd.update(1, viewer.doc)
        else:
            viewer.doc.showRenderer('softConstraint', True)
            renderer1 = viewer.doc.getRenderer('softConstraint')
            renderer1.rc.setLineWidth(0.1+Bsc*3)
            viewer.motionViewWnd.update(1, viewer.doc)
                        
        # tracking
        th_r = motion.getDOFPositions(frame)
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
        footCenterL = controlModel.getBodyPositionGlobal(supL)        
        footCenterR = controlModel.getBodyPositionGlobal(supR)
                        
        refFootL = motionModel.getBodyPositionGlobal(supL)        
        refFootR = motionModel.getBodyPositionGlobal(supR)
        
        footCenter = footCenterL + (footCenterR - footCenterL)/2.0
        footCenter[1] = 0.        
        
        footCenter_ref = refFootL + (refFootR - refFootL)/2.0
        #footCenter_ref[1] = 0.      
        
        foreFootCenterL = controlModel.getBodyPositionGlobal(foreSupL)
        foreFootCenterR = controlModel.getBodyPositionGlobal(foreSupR)
        rearFootCenterL = controlModel.getBodyPositionGlobal(rearSupL)
        rearFootCenterR = controlModel.getBodyPositionGlobal(rearSupR)
        
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
        CM_ref = yrp.getCM(linkPositions_ref, linkMasses, totalMass)
        CM_plane_ref = copy.copy(CM_ref)
        CM_plane_ref[1] = 0.
        
        P = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions, CM, linkInertias)
        dP = ymt.getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities, dCM, linkAngVelocities, linkInertias)

        yjc.computeJacobian2(Jsys, DOFs, jointPositions, jointAxeses, linkPositions, allLinkJointMasks)
        #dJsys = (Jsys - JsysPre)/(1/30.)
        #JsysPre = Jsys        
        yjc.computeJacobianDerivative2(dJsys, DOFs, jointPositions, jointAxeses, linkAngVelocities, linkPositions, allLinkJointMasks)
        
        if g_initFlag == 0:
            softConstPoint = controlModel.getBodyPositionGlobal(constBody)
            softConstPoint[1] -= .3
            g_initFlag = 1

        yjc.computeJacobian2(JsupL, DOFs, jointPositions, jointAxeses, [footCenterL], supLJointMasks)
        #dJsupL = (JsupL - JsupPreL)/(1/30.)
        #JsupPreL = JsupL
        yjc.computeJacobianDerivative2(dJsupL, DOFs, jointPositions, jointAxeses, linkAngVelocities, [footCenterL], supLJointMasks, False)
        
        yjc.computeJacobian2(JsupR, DOFs, jointPositions, jointAxeses, [footCenterR], supRJointMasks)
        #dJsupR = (JsupR - JsupPreR)/(1/30.)
        #JsupPreR = JsupR
        yjc.computeJacobianDerivative2(dJsupR, DOFs, jointPositions, jointAxeses, linkAngVelocities, [footCenterR], supRJointMasks, False)
        
        bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
        CP = yrp.getCP(contactPositions, contactForces)

        for i in range(len(bodyIDsToCheck)) :
            controlModel.SetBodyColor(bodyIDsToCheck[i], 0, 0, 0)

        flagForeSupLContact = 0
        flagForeSupRContact = 0
        flagRearSupLContact = 0
        flagRearSupLContact = 0

        for i in range(len(bodyIDs)) :
            controlModel.SetBodyColor(bodyIDs[i], 255, 105, 105)
            index = controlModel.id2index(bodyIDs[i])
            if index == foreSupL :
                flagForeSupLContact = 1
                yjc.computeJacobian2(JforeSupL, DOFs, jointPositions, jointAxeses, [foreFootCenterL], foreSupLJointMasks)
                yjc.computeJacobianDerivative2(dJforeSupL, DOFs, jointPositions, jointAxeses, linkAngVelocities, [foreFootCenterL], foreSupLJointMasks, False)
            elif index == foreSupR :
                flagForeSupRContact = 1
                yjc.computeJacobian2(JforeSupR, DOFs, jointPositions, jointAxeses, [foreFootCenterR], foreSupRJointMasks)
                yjc.computeJacobianDerivative2(dJforeSupR, DOFs, jointPositions, jointAxeses, linkAngVelocities, [foreFootCenterR], foreSupRJointMasks, False)
            elif index == rearSupL :
                flagRearSupLContact = 1
                yjc.computeJacobian2(JrearSupL, DOFs, jointPositions, jointAxeses, [rearFootCenterL], rearSupLJointMasks)
                yjc.computeJacobianDerivative2(dJrearSupL, DOFs, jointPositions, jointAxeses, linkAngVelocities, [rearFootCenterL], rearSupLJointMasks, False)
            elif index == rearSupR :
                flagRearSupRContact = 1
                yjc.computeJacobian2(JrearSupR, DOFs, jointPositions, jointAxeses, [rearFootCenterR], rearSupRJointMasks)
                yjc.computeJacobianDerivative2(dJrearSupR, DOFs, jointPositions, jointAxeses, linkAngVelocities, [rearFootCenterR], rearSupRJointMasks, False)
        #

        desForeSupLAcc = [0,0,0]
        desForeSupRAcc = [0,0,0]
                
        totalNormalForce = [0,0,0]
    
        for i in range(len(contactForces)):
            totalNormalForce[0] += contactForces[i][0]
            totalNormalForce[1] += contactForces[i][1]
            totalNormalForce[2] += contactForces[i][2]
                                   
        # linear momentum
        CM_ref_plane = footCenter
        dL_des_plane = Kl*totalMass*(CM_ref_plane - CM_plane) - Dl*totalMass*dCM_plane
    
        # angular momentum
        CP_ref = footCenter

        timeStep = 30.
        if CP_old[0]==None or CP==None:
            dCP = None
        else:
            dCP = (CP - CP_old[0])/(1/timeStep)
        CP_old[0] = CP            
        
        if CP!=None and dCP!=None:
            ddCP_des = Kh*(CP_ref - CP) - Dh*(dCP)
            CP_des = CP + dCP*(1/timeStep) + .5*ddCP_des*((1/timeStep)**2)
            dH_des = np.cross((CP_des - CM), (dL_des_plane + totalMass*mm.s2v(wcfg.gravity)))            
            #dH_des = np.cross((CP_des - CM_plane), (dL_des_plane + totalMass*mm.s2v(wcfg.gravity)))
        else:
            dH_des = None
 
        CMP = yrp.getCMP(contactForces, CM)
        r = [0,0,0]
        if CP!= None and np.any(np.isnan(CMP))!=True :
            r = CP - CMP

        # momentum matrix
        RS = np.dot(P, Jsys)
        R, S = np.vsplit(RS, 2)
        
        rs = np.dot((np.dot(dP, Jsys) + np.dot(P, dJsys)), dth_flat)
        r_bias, s_bias = np.hsplit(rs, 2)


        ##############################
        # soft point constraint
               
        P_des = softConstPoint
        P_cur = controlModel.getBodyPositionGlobal(constBody)
        dP_des = [0, 0, 0]
        dP_cur = controlModel.getBodyVelocityGlobal(constBody)
        ddP_des1 = Ksc*(P_des - P_cur) - Dsc*(dP_cur - dP_des)

        r = P_des - P_cur
        I = np.vstack(([1,0,0],[0,1,0],[0,0,1]))
        Z = np.hstack((I, mm.getCrossMatrixForm(-r)))
          
        yjc.computeJacobian2(Jconst, DOFs, jointPositions, jointAxeses, [softConstPoint], constJointMasks)
        JL, JA = np.vsplit(Jconst, 2)
        Q1 = np.dot(Z, Jconst)
                  
        q1 = np.dot(JA, dth_flat)
        q2 = np.dot(mm.getCrossMatrixForm(q1), np.dot(mm.getCrossMatrixForm(q1), r))
        
        yjc.computeJacobianDerivative2(dJconst, DOFs, jointPositions, jointAxeses, linkAngVelocities, [softConstPoint], constJointMasks, False)
        q_bias1 = np.dot(np.dot(Z, dJconst), dth_flat) + q2
        
        ##############################
        
         
        flagContact = True
        if dH_des==None or np.any(np.isnan(dH_des)) == True:
            flagContact = False 
            viewer.doc.showRenderer('rd_grf_des', False)
            viewer.motionViewWnd.update(1, viewer.doc)
        else:
            viewer.doc.showRenderer('rd_grf_des', True)
            viewer.motionViewWnd.update(1, viewer.doc)
        '''
        0 : initial
        1 : contact
        2 : fly
        3 : landing
        '''

        #MOTION = FORWARD_JUMP
        if mit.MOTION == mit.FORWARD_JUMP :
            frame_index = [136, 100]
        elif mit.MOTION == mit.TAEKWONDO:
            frame_index = [130, 100]
        else :
            frame_index = [1000000, 1000000]
        
        #MOTION = TAEKWONDO 
        #frame_index = [135, 100]

        if frame > frame_index[0] :
            stage = POWERFUL_BALANCING
            Kk = Kk*2
            Dk = 2*(Kk**.5)            
        elif frame > frame_index[1]:
            stage = MOTION_TRACKING

        trackingW = w
        if stage == MOTION_TRACKING:
            trackingW = w2

        # optimization
                
        mot.addTrackingTerms(problem, totalDOF, Bt, trackingW, ddth_des_flat)
                
        mot.addSoftPointConstraintTerms(problem, totalDOF, Bsc, ddP_des1, Q1, q_bias1)

        if flagContact == True:
            if stage != MOTION_TRACKING:
                mot.addLinearTerms(problem, totalDOF, Bl, dL_des_plane, R, r_bias) 
                mot.addAngularTerms(problem, totalDOF, Bh, dH_des, S, s_bias)

        ##############################
        # Hard constraint

        desLinearAccL = getDesFootLinearAcc(motionModel, controlModel, supL, ModelOffset, CM_ref, CM, Kk, Dk) 
        desLinearAccR = getDesFootLinearAcc(motionModel, controlModel, supR, ModelOffset, CM_ref, CM, Kk, Dk) 

        desAngularAccL = getDesFootAngularAcc(motionModel, controlModel, supL, Kk, Dk)
        desAngularAccR = getDesFootAngularAcc(motionModel, controlModel, supR, Kk, Dk)

        '''
        desLinearAccL = [0,0,0]
        desAngularAccL = [0,0,0]
        desLinearAccR = [0,0,0]
        desAngularAccR = [0,0,0]

        refPos = motionModel.getBodyPositionGlobal(supL)  
        curPos = controlModel.getBodyPositionGlobal(supL)
        refRootPos = motionModel.getBodyPositionGlobal(rootB)
        curRootPos = controlModel.getBodyPositionGlobal(rootB)
        refVecL = refPos - CM_ref
        if stage == MOTION_TRACKING:
            refPos = CM + refVecL
            refPos[1] += 0.05
            refPos[0] -= 0.05
        elif stage == POWERFUL_BALANCING:
            refPos = copy.copy(curPos)
            refPos[1] = 0
        else:
            refPos[0] += ModelOffset[0]
        
        rd_root[0] = curRootPos
                        
        refVel = motionModel.getBodyVelocityGlobal(supL) 
        rd_footCenterL[0] = copy.copy(curPos)
        #rd_footCenterL[0][2] += 0.2
        curVel = controlModel.getBodyVelocityGlobal(supL)
        #refAcc = (0,0,0)
        refAcc = motionModel.getBodyAccelerationGlobal(supL)
         
        if stage != MOTION_TRACKING:
            refPos[1] = 0
        
        if refPos[1] < 0.0 :
            refPos[1] = 0.0
        rd_DesPosL[0] = refPos

        desLinearAccL = yct.getDesiredAcceleration(refPos, curPos, refVel, curVel, refAcc, Kk, Dk)
         
        refPos = motionModel.getBodyPositionGlobal(supR)       
        curPos = controlModel.getBodyPositionGlobal(supR)
        refVecR = refPos - CM_ref
        if stage == MOTION_TRACKING:
            refPos = CM + refVecR
            refPos[1] += 0.05
            refPos[0] -= 0.05
        elif stage == POWERFUL_BALANCING:
            refPos = copy.copy(curPos)
            refPos[1] = 0
        else :
            refPos[0] += ModelOffset[0]
        
        refVel = motionModel.getBodyVelocityGlobal(supR) 
        curVel = controlModel.getBodyVelocityGlobal(supR)
        refAcc = motionModel.getBodyAccelerationGlobal(supR)
        
        if stage != MOTION_TRACKING:
            refPos[1] = 0
        
        if refPos[1] < 0.0 :
            refPos[1] = 0.0
        rd_DesPosR[0] = refPos

        desLinearAccR = yct.getDesiredAcceleration(refPos, curPos, refVel, curVel, refAcc, Kk, Dk)

        curAng = [controlModel.getBodyOrientationGlobal(supL)]
        refAngVel = motionModel.getBodyAngVelocityGlobal(supL)
        curAngVel = controlModel.getBodyAngVelocityGlobal(supL)
        refAngAcc = (0,0,0)
                        
        curAngY = np.dot(curAng, np.array([0,1,0]))
        refAngY = np.array([0,1,0])
        if stage == DYNAMIC_BALANCING:    
            refAng = [motionModel.getBodyOrientationGlobal(supL)]
            refAngY2 = np.dot(refAng, np.array([0,1,0]))
            refAngY = refAngY2[0]
            
        rd_footL_vec[0] = refAngY
        rd_footR_vec[0] = curAngY[0]
        aL = mm.logSO3(mm.getSO3FromVectors(curAngY[0], refAngY))
        desAngularAccL = [Kk*aL + Dk*(refAngVel-curAngVel)]      
        
        curAng = [controlModel.getBodyOrientationGlobal(supR)]
        refAngVel = motionModel.getBodyAngVelocityGlobal(supR)
        curAngVel = controlModel.getBodyAngVelocityGlobal(supR)
        refAngAcc = (0,0,0)
        
        curAngY = np.dot(curAng, np.array([0,1,0]))
        refAngY = np.array([0,1,0])
        if stage == DYNAMIC_BALANCING:    
            refAng = [motionModel.getBodyOrientationGlobal(supR)]
            refAngY2 = np.dot(refAng, np.array([0,1,0]))
            refAngY = refAngY2[0]
            
        aL = mm.logSO3(mm.getSO3FromVectors(curAngY[0], refAngY))
        desAngularAccR = [Kk*aL + Dk*(refAngVel-curAngVel)]

        a_sup_2 = [desLinearAccL[0], desLinearAccL[1], desLinearAccL[2], desAngularAccL[0][0], desAngularAccL[0][1], desAngularAccL[0][2], 
                   desLinearAccR[0], desLinearAccR[1], desLinearAccR[2], desAngularAccR[0][0], desAngularAccR[0][1], desAngularAccR[0][2]]
        '''
        a_sup_2 = np.hstack(( np.hstack((desLinearAccL, desAngularAccL)), np.hstack((desLinearAccR, desAngularAccR)) ))
 
        Jsup_2 = np.vstack((JsupL, JsupR))
        dJsup_2 = np.vstack((dJsupL, dJsupR))        

        mot.setConstraint(problem, totalDOF, Jsup_2, dJsup_2, dth_flat, a_sup_2)
        
        ##############################

        ##############################
        # Additional constraint
        desLinearAccL = [0,0,0]
        desAngularAccL = [0,0,0]
        desLinearAccR = [0,0,0]
        desAngularAccR = [0,0,0]

        refPos = motionModel.getBodyPositionGlobal(supL)  
        curPos = controlModel.getBodyPositionGlobal(supL)
        refVecL = refPos - CM_ref
        if stage == MOTION_TRACKING:
            refPos = CM + refVecL
            refPos[1] += 0.05
            refPos[0] -= 0.05
        elif stage == POWERFUL_BALANCING:
            refPos = copy.copy(curPos)
            refPos[1] = 0
        else:
            refPos[0] += ModelOffset[0]
                                
        refVel = motionModel.getBodyVelocityGlobal(supL) 
        curVel = controlModel.getBodyVelocityGlobal(supL)
        #refAcc = (0,0,0)
        refAcc = motionModel.getBodyAccelerationGlobal(supL)
         
        if stage != MOTION_TRACKING:
            refPos[1] = 0
        
        if refPos[1] < 0.0 :
            refPos[1] = 0.0

        desLinearAccL = yct.getDesiredAcceleration(refPos, curPos, refVel, curVel, refAcc, Kk, Dk)

        curAng = [controlModel.getBodyOrientationGlobal(supL)]
        refAngVel = motionModel.getBodyAngVelocityGlobal(supL)
        curAngVel = controlModel.getBodyAngVelocityGlobal(supL)
        refAngAcc = (0,0,0)
                        
        curAngY = np.dot(curAng, np.array([0,1,0]))
        refAngY = np.array([0,1,0])
        if stage == DYNAMIC_BALANCING:    
            refAng = [motionModel.getBodyOrientationGlobal(supL)]
            refAngY2 = np.dot(refAng, np.array([0,1,0]))
            refAngY = refAngY2[0]
            
        rd_footL_vec[0] = refAngY
        rd_footR_vec[0] = curAngY[0]
        aL = mm.logSO3(mm.getSO3FromVectors(curAngY[0], refAngY))
        desAngularAccL = [Kk*aL + Dk*(refAngVel-curAngVel)] 

        ##############################

        r = problem.solve()
        problem.clear()
        ype.nested(r['x'], ddth_sol)
                      
        rootPos[0] = controlModel.getBodyPositionGlobal(selectedBody)
        localPos = [[0, 0, 0]]   
                
        for i in range(stepsPerFrame):
            # apply penalty force
            bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
     
            vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)
                        
            controlModel.setDOFAccelerations(ddth_sol)
            
            if flagForeSupLContact == 1:
                controlModel.setJointAngAccelerationLocal(foreSupL, (2.1, 0, 0))
            if flagForeSupRContact == 1:
                controlModel.setJointAngAccelerationLocal(foreSupR, (2.1, 0, 0))

            controlModel.solveHybridDynamics()
            
            extraForce[0] = viewer.GetForce()
            if (extraForce[0][0] != 0 or extraForce[0][1] != 0 or extraForce[0][2] != 0) :
                forceApplyFrame += 1
                vpWorld.applyPenaltyForce(selectedBodyId, localPos, extraForce)
                applyedExtraForce[0] = extraForce[0]
            
            if forceApplyFrame*wcfg.timeStep > 0.1:
                viewer.ResetForce()
                forceApplyFrame = 0            

            vpWorld.step()                    
            
        # rendering
        rd_footCenter[0] = footCenter
        
        rd_CM[0] = CM.copy()
        
        rd_CM_plane[0] = CM_plane.copy()
        
        rd_footCenter_ref[0] = footCenter_ref
        rd_CM_plane_ref[0] = CM_ref.copy()
        rd_CM_ref[0] = CM_ref.copy()
        rd_CM_ref_vec[0] = (CM_ref - footCenter_ref)*3.
        rd_CM_vec[0] = (CM - footCenter)*3

        #rd_CM_plane[0][1] = 0.
        
        if CP!=None and dCP!=None:
            rd_CP[0] = CP
            rd_CP_des[0] = CP_des
        
        rd_dL_des_plane[0] = dL_des_plane
        rd_dH_des[0] = dH_des
        
        rd_grf_des[0] = totalNormalForce - totalMass*mm.s2v(wcfg.gravity)#dL_des_plane - totalMass*mm.s2v(wcfg.gravity)
                
        rd_exf_des[0] = applyedExtraForce[0]
        rd_root_des[0] = rootPos[0]

        rd_CMP[0] = softConstPoint

        rd_soft_const_vec[0] = controlModel.getBodyPositionGlobal(constBody)-softConstPoint
        
        if (forceApplyFrame == 0) :
            applyedExtraForce[0] = [0, 0, 0]

    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/60.)
    viewer.show()
    
    Fl.run()
    
main()