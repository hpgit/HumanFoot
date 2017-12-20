from fltk import *
import copy
import numpy as np

import sys

import PyCommon.modules.Math.mmMath as mm
import PyCommon.modules.Resource.ysMotionLoader as yf
import PyCommon.modules.Renderer.ysRenderer as yr
import PyCommon.modules.Renderer.csVpRenderer as cvr
import PyCommon.modules.Simulator.csVpWorld as cvw
import PyCommon.modules.Simulator.csVpModel as cvm
import PyCommon.modules.GUI.ysSimpleViewer as ysv
import PyCommon.modules.Optimization.ysAnalyticConstrainedOpt as yac
import PyCommon.modules.ArticulatedBody.ysJacobian as yjc
import PyCommon.modules.Util.ysPythonEx as ype
import PyCommon.modules.ArticulatedBody.ysReferencePoints as yrp
import PyCommon.modules.ArticulatedBody.ysMomentum as ymt
import PyCommon.modules.ArticulatedBody.ysControl as yct
import PyCommon.modules.Motion.ysHierarchyEdit as yme
import PyCommon.modules.Simulator.ysPhysConfig as ypc

import numpy.linalg as npl

import mtOptimize as mot
import mtInitialize as mit

contactState = 0
g_applyForce = False

g_initFlag = 0
preFootCenterL = [0, 0, 0]
preFootCenterR = [0, 0, 0]
preFootOrientationL = [[0,0,0], [0,0,0], [0,0,0]]
preFootOrientationR = [[0,0,0], [0,0,0], [0,0,0]]

softConstPoint = [0, 0, 0]

forceShowFrame = 0
forceApplyFrame = 0

JsysPre = 0
JsupPreL = 0
JsupPreR = 0
JsupPre = 0

stage = 0



def main():

    np.set_printoptions(precision=4, linewidth=200)
    
#    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_5()
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped()
        
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    controlModel.initializeHybridDynamics()
    
    ModelOffset = (1.5, -0.02, 0)
    #ModelOffset = (1.5, 1.02, 0)
    #ModelOffset = (1.5, 0.02, 0)
    controlModel.translateByOffset(ModelOffset)
    #controlModel.translateByOffset((1.5,-0.0328,0))#(1.5,-0.02,0))
    
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
    w_IK = mot.getTrackingWeight(DOFs, motion[0].skeleton, config['IKweightMap'])
    supL =  motion[0].skeleton.getJointIndex(config['supLink'])
    supR =  motion[0].skeleton.getJointIndex(config['supLink2'])

    #controlModel.SetGround(supL, True)
    #controlModel.SetGround(supR, True)

    selectedBody = motion[0].skeleton.getJointIndex(config['end'])
    #constBody = motion[0].skeleton.getJointIndex('LeftForeArm')
    constBody = motion[0].skeleton.getJointIndex('Hips')
    
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

    supLJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, supL)]
    supRJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, supR)]
    constJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, constBody)]
    allLinkJointMasks = yjc.getAllLinkJointMasks(motion[0].skeleton)
    
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
    rd_CM = [None]
    rd_CP = [None]
    rd_CP_des = [None]
    rd_dL_des_plane = [None]
    rd_dH_des = [None]
    rd_grf_des = [None]
    
    rd_exf_des = [None]
    rd_root_des = [None]
    rd_soft_const_vec = [None]
        
    rd_CMP = [None]
    rd_DesPosL = [None]
    rd_DesPosR = [None]

    rootPos = [None]
    selectedBodyId = [selectedBody]
    extraForce = [None]
    applyedExtraForce = [None]
    applyedExtraForce[0] = [0,0,0]
    
    viewer = ysv.SimpleViewer()
#    viewer.record(False)
#    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('rd_footCenter', yr.PointsRenderer(rd_footCenter))    
    #viewer.doc.addRenderer('rd_footCenterL', yr.PointsRenderer(rd_footCenterL))  
    #viewer.doc.addRenderer('rd_footCenterR', yr.PointsRenderer(rd_footCenterR))
    viewer.doc.addRenderer('rd_CM_plane', yr.PointsRenderer(rd_CM_plane, (255,255,0)))
    viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (0,255,0)))
    #viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (255,0,255)))
#    viewer.doc.addRenderer('rd_dL_des_plane', yr.VectorsRenderer(rd_dL_des_plane, rd_CM, (255,255,0)))
#    viewer.doc.addRenderer('rd_dH_des', yr.VectorsRenderer(rd_dH_des, rd_CM, (0,255,0)))
    viewer.doc.addRenderer('rd_grf_des', yr.ForcesRenderer(rd_grf_des, rd_CP_des, (0,255,255), .001))

    viewer.doc.addRenderer('rd_exf_des', yr.ForcesRenderer(rd_exf_des, rd_root_des, (0,255,0), .009, 0.05))
        
    #viewer.doc.addRenderer('rd_CMP', yr.PointsRenderer(rd_CMP, (0,0,255)))
    
    viewer.doc.addRenderer('rd_DesPosL', yr.PointsRenderer(rd_DesPosL, (0,0,255)))
    viewer.doc.addRenderer('rd_DesPosR', yr.PointsRenderer(rd_DesPosR, (0,100,255)))

    #viewer.doc.addRenderer('softConstraint', yr.VectorsRenderer(rd_soft_const_vec, rd_CMP, (255,0,0), 3))

    
    viewer.doc.addRenderer('rd_footCenter_ref', yr.PointsRenderer(rd_footCenter_ref))    
    viewer.doc.addRenderer('rd_CM_plane_ref', yr.PointsRenderer(rd_CM_plane_ref, (255,255,0)))
        
    stage = 0

    def simulateCallback(frame):
        global g_initFlag
        global preFootCenterL, preFootCenterR
        global preFootOrientationL, preFootOrientationR
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
        
        if stage == 3:
            Bsc = 0
            #Kl *= 1.5

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
        footCenter_ref[1] = 0.      
        
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
        CM_plane_ref = yrp.getCM(linkPositions_ref, linkMasses, totalMass)
        CM_plane_ref[1] = 0.
        
        P = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions, CM, linkInertias)
        dP = ymt.getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities, dCM, linkAngVelocities, linkInertias)

        yjc.computeJacobian2(Jsys, DOFs, jointPositions, jointAxeses, linkPositions, allLinkJointMasks)
        dJsys = (Jsys - JsysPre)/(1/30.)
        JsysPre = Jsys
        #yjc.computeJacobianDerivative2(dJsys, DOFs, jointPositions, jointAxeses, linkAngVelocities, linkPositions, allLinkJointMasks)
        
        
        if g_initFlag == 0:
            preFootCenterL = footCenterL
            preFootCenterR = footCenterR
            preFootCenterL[1] -= 0.02
            preFootCenterR[1] -= 0.02
            preFootOrientationL = controlModel.getBodyOrientationGlobal(supL)
            preFootOrientationR = controlModel.getBodyOrientationGlobal(supR)
            softConstPoint = controlModel.getBodyPositionGlobal(constBody)
            #softConstPoint[2] += 0.3
            #softConstPoint[1] -= 1.1
            #softConstPoint[0] += 0.1

            softConstPoint[1] -= .3            
            #softConstPoint[0] -= .1
            #softConstPoint[1] -= 1.
            #softConstPoint[0] -= .5
            g_initFlag = 1

        yjc.computeJacobian2(JsupL, DOFs, jointPositions, jointAxeses, [footCenterL], supLJointMasks)
        dJsupL = (JsupL - JsupPreL)/(1/30.)
        JsupPreL = JsupL
        #yjc.computeJacobianDerivative2(dJsupL, DOFs, jointPositions, jointAxeses, linkAngVelocities, [footCenterL], supLJointMasks, False)
        
        yjc.computeJacobian2(JsupR, DOFs, jointPositions, jointAxeses, [footCenterR], supRJointMasks)
        dJsupR = (JsupR - JsupPreR)/(1/30.)
        JsupPreR = JsupR
        #yjc.computeJacobianDerivative2(dJsupR, DOFs, jointPositions, jointAxeses, linkAngVelocities, [footCenterR], supRJointMasks, False)
        
        preFootCenter = preFootCenterL + (preFootCenterR - preFootCenterL)/2.0
        preFootCenter[1] = 0

        bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
        CP = yrp.getCP(contactPositions, contactForces)
                                   
        # linear momentum
        CM_ref_plane = footCenter
        #CM_ref_plane = preFootCenter
        dL_des_plane = Kl*totalMass*(CM_ref_plane - CM_plane) - Dl*totalMass*dCM_plane
        #print("dL_des_plane ", dL_des_plane )
        #dL_des_plane[1] = 0.
    
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
            #dH_des = [0, 0, 0]
        else:
            dH_des = None
 
        CMP = yrp.getCMP(contactForces, CM)
        r = [0,0,0]
        if CP!= None and np.any(np.isnan(CMP))!=True :
            r = CP - CMP
        #print("r.l", mm.length(r))
        #Bba = Bh*(mm.length(r))
        Bba = Bh

        # momentum matrix
        RS = np.dot(P, Jsys)
        R, S = np.vsplit(RS, 2)
        
        rs = np.dot((np.dot(dP, Jsys) + np.dot(P, dJsys)), dth_flat)
        r_bias, s_bias = np.hsplit(rs, 2)

        ##############################
        # soft point constraint

        '''
        cmDiff = footCenter - CM_plane
        print("cmDiff", cmDiff)
        if stage == 3:
            softConstPoint +=
        '''
        
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
        
        '''
        P_des = preFootCenterR
        P_cur = controlModel.getBodyPositionGlobal(supR)
        P_cur[1] = 0
        dP_des = [0, 0, 0]
        dP_cur = controlModel.getBodyVelocityGlobal(supR)
        ddP_des2 = Kp*(P_des - P_cur) - Dp*(dP_cur - dP_des)
        
        r = P_des - P_cur
        #print("r2", r)
        I = np.vstack(([1,0,0],[0,1,0],[0,0,1]))
        Z = np.hstack((I, mm.getCrossMatrixForm(-r)))
            
        JL, JA = np.vsplit(JsupR, 2)
        Q2 = np.dot(Z, JsupR)
        
        q1 = np.dot(JA, dth_flat)
        q2 = np.dot(mm.getCrossMatrixForm(q1), np.dot(mm.getCrossMatrixForm(q1), r))
        q_bias2 = np.dot(np.dot(Z, dJsupR), dth_flat) + q2
        '''      
        #print("Q1", Q1)
        '''
        print("ddP_des1", ddP_des1)
        q_ddth1 = np.dot(Q1, ddth_c_flat)
        print("q_ddth1", q_ddth1)
        print("q_bias1", q_bias1)
        ddp1 = q_ddth1+q_bias1
        print("ddp1", ddp1)
        print("diff1", ddP_des1-ddp1)
        '''

        '''
        print("ddP_des2", ddP_des2)
        q_ddth2 = np.dot(Q2, ddth_c_flat)
        print("q_ddth2", q_ddth2)
        print("q_bias2", q_bias2)
        ddp2 = q_ddth2+q_bias2
        print("ddp2", ddp2)
        print("diff2", ddP_des2-ddp2)
        '''

        ##############################
        
        
        ############################
        # IK
        '''
        P_des = preFootCenterL
        P_cur = controlModel.getJointPositionGlobal(supL)
        r = P_des - P_cur
        
        Q_des = preFootOrientationL 
        Q_cur = controlModel.getJointOrientationGlobal(supL)
        rv = mm.logSO3(np.dot(Q_cur.transpose(), Q_des))
        #print("rv", rv)

        des_v_sup = (r[0],r[1],r[2], rv[0], rv[1], rv[2])
        A_large = np.dot(JsupL.T, JsupL)
        b_large = np.dot(JsupL.T, des_v_sup)

        des_d_th = npl.lstsq(A_large, b_large)

        ype.nested(des_d_th[0], d_th_IK_L)
                                        
        P_des2 = preFootCenterR
        P_cur2 = controlModel.getJointPositionGlobal(supR)
        r2 = P_des2 - P_cur2
                
        Q_des2 = preFootOrientationR
        Q_cur2 = controlModel.getJointOrientationGlobal(supR)
        rv2 = mm.logSO3(np.dot(Q_cur2.transpose(), Q_des2))
        #print("Q_des2", Q_des2)
        #print("Q_cur2", Q_cur2)
        #print("rv2", rv2)

        des_v_sup2 = (r2[0],r2[1],r2[2], rv2[0], rv2[1], rv[2])
        A_large = np.dot(JsupR.T, JsupR)
        b_large = np.dot(JsupR.T, des_v_sup2)

        des_d_th = npl.lstsq(A_large, b_large)

        ype.nested(des_d_th[0], d_th_IK_R)
        for i in range(len(d_th_IK_L)):
            for j in range(len(d_th_IK_L[i])):
                d_th_IK[i][j] = d_th_IK_L[i][j] + d_th_IK_R[i][j]
                    
        th_IK = yct.getIntegralDOF(th, d_th_IK, 1/timeStep)
        dd_th_IK = yct.getDesiredDOFAccelerations(th_IK, th, d_th_IK, dth, ddth_r, Kk, Dk)
                        
        ype.flatten(d_th_IK, d_th_IK_flat)
        ype.flatten(dd_th_IK, dd_th_IK_flat)
        '''
        ############################
        
        flagContact = True
        if dH_des==None or np.any(np.isnan(dH_des)) == True:
            flagContact = False 
        '''
        0 : initial
        1 : contact
        2 : fly
        3 : landing
        '''
        if flagContact == False :
            if stage == 1:
                stage = 2
                print("fly")
        else:
            if stage == 0:
                stage = 1
                print("contact")
            elif stage == 2:
                stage = 3 
                print("landing")


        if stage == 3:
            Bt = Bt*0.8
            Bl = Bl*1

        # optimization
                
        mot.addTrackingTerms(problem, totalDOF, Bt, w, ddth_des_flat)
        
        #mot.addTrackingTerms(problem, totalDOF, Bk, w_IK, dd_th_IK_flat)       
                
        mot.addSoftPointConstraintTerms(problem, totalDOF, Bsc, ddP_des1, Q1, q_bias1)

        #mot.addSoftPointConstraintTerms(problem, totalDOF, Bp, ddP_des2, Q2, q_bias2)

        #mot.addConstraint(problem, totalDOF, JsupL, dJsupL, dth_flat, a_sup)
        #mot.addConstraint(problem, totalDOF, JsupR, dJsupR, dth_flat, a_sup2)
        
       

        desLinearAccL = [0,0,0]
        desAngularAccL = [0,0,0]
        desLinearAccR = [0,0,0]
        desAngularAccR = [0,0,0]

        refPos = motionModel.getBodyPositionGlobal(supL)
        refPos[0] += ModelOffset[0]
        refPos[1] = 0
                        
        refVel = motionModel.getBodyVelocityGlobal(supL)        
        curPos = controlModel.getBodyPositionGlobal(supL)
        #curPos[1] = 0
        curVel = controlModel.getBodyVelocityGlobal(supL)
        refAcc = (0,0,0)
                
        if stage == 3:
            refPos = curPos
            refPos[1] = 0
            if curPos[1] < 0.0:
                curPos[1] = 0
        else :
            curPos[1] = 0
        rd_DesPosL[0] = refPos

        #(p_r, p, v_r, v, a_r, Kt, Dt)
        desLinearAccL = yct.getDesiredAcceleration(refPos, curPos, refVel, curVel, refAcc, Kk, Dk)
        #desLinearAccL[1] = 0
         
        refPos = motionModel.getBodyPositionGlobal(supR)
        refPos[0] += ModelOffset[0]
        refPos[1] = 0    

        refVel = motionModel.getBodyVelocityGlobal(supR)        
        curPos = controlModel.getBodyPositionGlobal(supR)
        #curPos[1] = 0
        curVel = controlModel.getBodyVelocityGlobal(supR)
        
        if stage == 3:
            refPos = curPos
            refPos[1] = 0
            if curPos[1] < 0.0:
                curPos[1] = 0
        else :
            curPos[1] = 0
        rd_DesPosR[0] = refPos

        desLinearAccR = yct.getDesiredAcceleration(refPos, curPos, refVel, curVel, refAcc, Kk, Dk)
        #desLinearAccR[1] = 0
                
        #(th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        refAng = [preFootOrientationL]
        curAng = [controlModel.getBodyOrientationGlobal(supL)]
        refAngVel = motionModel.getBodyAngVelocityGlobal(supL)
        curAngVel = controlModel.getBodyAngVelocityGlobal(supL)
        refAngAcc = (0,0,0)
                        
        #desAngularAccL = yct.getDesiredAngAccelerations(refAng, curAng, refAngVel, curAngVel, refAngAcc, Kk, Dk)
        curAngY = np.dot(curAng, np.array([0,1,0]))
        aL = mm.logSO3(mm.getSO3FromVectors(curAngY[0], np.array([0,1,0])))
        print("curAngYL=",curAngY, "aL=", aL)
        desAngularAccL = [Kk*aL + Dk*(refAngVel-curAngVel)]

        refAng = [preFootOrientationR]
        curAng = [controlModel.getBodyOrientationGlobal(supR)]
        refAngVel = motionModel.getBodyAngVelocityGlobal(supR)
        curAngVel = controlModel.getBodyAngVelocityGlobal(supR)
        refAngAcc = (0,0,0)
        
        #desAngularAccR = yct.getDesiredAngAccelerations(refAng, curAng, refAngVel, curAngVel, refAngAcc, Kk, Dk)   
        curAngY = np.dot(curAng, np.array([0,1,0]))
        aL = mm.logSO3(mm.getSO3FromVectors(curAngY[0], np.array([0,1,0])))
        desAngularAccR = [Kk*aL + Dk*(refAngVel-curAngVel)]
            
        print("curAngYR=",curAngY, "aL=", aL)

        a_sup_2 = [desLinearAccL[0], desLinearAccL[1], desLinearAccL[2], desAngularAccL[0][0], desAngularAccL[0][1], desAngularAccL[0][2], 
                   desLinearAccR[0], desLinearAccR[1], desLinearAccR[2], desAngularAccR[0][0], desAngularAccR[0][1], desAngularAccR[0][2]]

        if stage == 2 :#or stage == 3:
            refAccL = motionModel.getBodyAccelerationGlobal(supL)
            refAndAccL = motionModel.getBodyAngAccelerationGlobal(supL)
            refAccR = motionModel.getBodyAccelerationGlobal(supR)
            refAndAccR = motionModel.getBodyAngAccelerationGlobal(supR)
            a_sup_2 = [refAccL[0], refAccL[1], refAccL[2], refAndAccL[0], refAndAccL[1], refAndAccL[2],
                       refAccR[0], refAccR[1], refAccR[2], refAndAccR[0], refAndAccR[1], refAndAccR[2]]
            '''
            a_sup_2 = [0,0,0, desAngularAccL[0][0], desAngularAccL[0][1], desAngularAccL[0][2], 
                       0,0,0, desAngularAccR[0][0], desAngularAccR[0][1], desAngularAccR[0][2]]
            '''
                    
        Jsup_2 = np.vstack((JsupL, JsupR))
        dJsup_2 = np.vstack((dJsupL, dJsupR))
        
        if flagContact == True:
            mot.addLinearTerms(problem, totalDOF, Bl, dL_des_plane, R, r_bias) 
            mot.addAngularTerms(problem, totalDOF, Bh, dH_des, S, s_bias)
                   
        mot.setConstraint(problem, totalDOF, Jsup_2, dJsup_2, dth_flat, a_sup_2)
        #mot.setConstraint(problem, totalDOF, JsupR, dJsupR, dth_flat, a_sup2)
        #mot.setConstraint(problem, totalDOF, Jsup_2, dJsup_2, dth_flat, a_sup_2)
        #mot.addConstraint(problem, totalDOF, Jsup_2, dJsup_2, d_th_IK_flat, a_sup_2)

        '''
        jZ = np.dot(dJsup_2.T, dJsup_2)

        lamda = 0.001

        for i in range(len(jZ)):
            for j in range(len(jZ[0])):
                if i == j :
                    jZ[i][j] += lamda

        jZInv = npl.pinv(jZ)
        jA = np.dot(Jsup_2, np.dot(jZInv, np.dot(dJsup_2.T, -Jsup_2)))
        mot.addConstraint2(problem, totalDOF, jA, a_sup_2)
        '''
        
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
        rd_footCenterL[0] = preFootCenterL
        rd_footCenterR[0] = preFootCenterR
        
        rd_CM[0] = CM
        
        rd_CM_plane[0] = CM_plane.copy()
        
        rd_footCenter_ref[0] = footCenter_ref
        rd_CM_plane_ref[0] = CM_plane_ref.copy()

        #rd_CM_plane[0][1] = 0.
        
        if CP!=None and dCP!=None:
            rd_CP[0] = CP
            rd_CP_des[0] = CP_des
        
        rd_dL_des_plane[0] = dL_des_plane
        rd_dH_des[0] = dH_des
        
        rd_grf_des[0] = dL_des_plane - totalMass*mm.s2v(wcfg.gravity)
                
        rd_exf_des[0] = applyedExtraForce[0]
        #print("rd_exf_des", rd_exf_des[0])
        rd_root_des[0] = rootPos[0]

        rd_CMP[0] = softConstPoint

        rd_soft_const_vec[0] = controlModel.getBodyPositionGlobal(constBody)-softConstPoint
        
        #if (applyedExtraForce[0][0] != 0 or applyedExtraForce[0][1] != 0 or applyedExtraForce[0][2] != 0) :
        if (forceApplyFrame == 0) :
            applyedExtraForce[0] = [0, 0, 0]

    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/60.)
    viewer.show()
    
    Fl.run()
    
main()