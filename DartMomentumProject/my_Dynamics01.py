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

import Motion.ysMotion as ym

import mtOptimize as mot
import mtInitialize as mit

stage = 0

def main():
    np.set_printoptions(precision=4, linewidth=200)
    
#    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_5()
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped()
    
    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    
    #controlModel2 = cvm.VpControlModel(vpWorld, motion[0], mcfg)

    vpWorld.initialize()
    controlModel.initializeHybridDynamics()    
    controlModel.translateByOffset((1.5,0,0))
        
    #controlModel2.initializeHybridDynamics()    
    #controlModel2.translateByOffset((2.5,0,0))

    
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
    sup =  motion[0].skeleton.getJointIndex(config['supLink'])

    sup2 =  motion[0].skeleton.getJointIndex(config['supLink2'])
    
    # jacobian 
    Jsup = yjc.makeEmptyJacobian(DOFs, 1)
    dJsup = Jsup.copy()
    
    Jsys = yjc.makeEmptyJacobian(DOFs, controlModel.getBodyNum())
    dJsys = Jsys.copy()

    supJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, sup)]
    allLinkJointMasks = yjc.getAllLinkJointMasks(motion[0].skeleton)
        
    Jsup2 = yjc.makeEmptyJacobian(DOFs, 1)
    dJsup2 = Jsup2.copy()          
    
    supJointMasks2 = [yjc.getLinkJointMask(motion[0].skeleton, sup2)]
    
    CMJointMask = supJointMasks
    for iMask in range(len(CMJointMask[0])):
        if supJointMasks2[0][iMask] == 1:
            CMJointMask[0][iMask] = 1

    Jt = yjc.makeEmptyJacobian(DOFs, 1)
    
    # momentum matrix
    linkMasses = controlModel.getBodyMasses()
    totalMass = controlModel.getTotalMass()
    TO = ymt.make_TO(linkMasses) 
    dTO = ymt.make_dTO(len(linkMasses))
    
    # optimization
    problem = yac.LSE(totalDOF, 6)
    a_sup = (0,0,0, 0,0,0)
    CP_old = [mm.v3(0.,0.,0.)]
        
    # penalty method 
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    mus = [1.]*len(bodyIDsToCheck)


    ##### Parameter    
    '''
    (0, 'Hips')
    (1, 'LeftUpLeg')
    (2, 'LeftLeg')
    (3, 'LeftFoot')
    (4, 'RightUpLeg')
    (5, 'RightLeg')
    (6, 'RightFoot')
    (7, 'Spine')
    (8, 'Spine1')
    (9, 'LeftArm')
    (10, 'LeftForeArm')
    (11, 'RightArm')
    (12, 'RightForeArm')
    '''
    Kp = [None]*len(bodyIDsToCheck)
    Kd = [None]*len(bodyIDsToCheck)
    #Kp = [0,  400, 450, 400,  400, 450, 400,  300, 250,   60, 25, 60, 25]    
    #Kp = [0,  450, 450, 400,  450, 450, 400,  450, 300,   100, 25, 100, 25]

    UpLeg = 310
    Leg = 400
    Foot = 310
    Spine = 350
    Neck = 200
    Arm = 10
    Hand = 10
    Kp = [0,  UpLeg, Leg, Foot,  UpLeg, Leg, Foot,  Spine, Neck,   Arm, Hand, Arm, Hand]
    '''
    for i in range(0, len(Kp)) :
        Kp[i] = Kp[i]*0.8
    '''

    #Kd = [0,  0, 0, 0,    0, 0, 0,    0, 0,  0, 0, 0, 0]
    #Kd = [0,  2.0, 1.5, 1.2,    2.0, 1.5, 1.2,    2.0, 0.1,  0.1, 0.1, 0.1, 0.1]
    #Kd = [0,  1.5, 2.0, 1.2,    1.5, 2.0, 1.2,    1.5, 0.4,  0.5, 0.2, 0.5, 0.2]
    
    '''
    dUpLeg = 1.7
    dLeg = 1.7
    dFoot = 1.2
    dSpine = 1.4
    dNeck = 0.9
    dArm = 0.8
    dHand = 0.2
    '''    
    dUpLeg = 2*(UpLeg**.5)
    dLeg = 2*(Leg**.5)
    dFoot = 2*(Foot**.5)
    dSpine = 2*(Spine**.5)
    dNeck = 2*(Neck**.5)
    dArm = 2*(Arm**.5)
    dHand = 2*(Hand**.5)

    Kd = [0,  dUpLeg, dLeg, dFoot,  dUpLeg, dLeg, dFoot,  dSpine, dNeck,   dArm, dHand, dArm, dHand]
    
    for i in range(0, len(Kd)) :
        Kd[i] = Kd[i]*0.022
   

    '''
    for ii in bodyIDsToCheck :
        print(ii, controlModel.index2name(ii))
    '''

    # flat data structure
    ddth_des_flat = ype.makeFlatList(totalDOF)
    dth_flat = ype.makeFlatList(totalDOF)
    ddth_sol = ype.makeNestedList(DOFs)

    # viewer
    rd_footCenter = [None]
    rd_footCenter1 = [None]
    rd_footCenter2 = [None]
    rd_CM_plane = [None]
    rd_CM = [None]
    rd_CP = [None]
    rd_CP_des = [None]
    rd_dL_des_plane = [None]
    rd_dH_des = [None]
    rd_grf_des = [None]
    rd_vf = [None]

    rd_contactPoint1 = [None]
    rd_contactPoint2 = [None]

    viewer = ysv.SimpleViewer()
#    viewer.record(False)
#    viewer.doc.addRenderer('control', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL))
    #viewer.doc.addRenderer('controlModel2', cvr.VpModelRenderer(controlModel2, (155,100,100), yr.POLYGON_FILL))
    
    #viewer.doc.addRenderer('rd_footCenter', yr.PointsRenderer(rd_footCenter))
    #viewer.doc.addRenderer('rd_footCenter1', yr.PointsRenderer(rd_footCenter1, (255, 0, 255)))
    #viewer.doc.addRenderer('rd_footCenter2', yr.PointsRenderer(rd_footCenter2, (0, 255, 255)))
    #viewer.doc.addRenderer('rd_CM_plane', yr.PointsRenderer(rd_CM_plane, (255,255,0)))
    #viewer.doc.addRenderer('rd_CM', yr.PointsRenderer(rd_CM, (255,255,0)))

    
    #viewer.doc.addRenderer('rd_contactPoint', yr.PointsRenderer(rd_contactPoint1, (0,0,255)))
    #viewer.doc.addRenderer('rd_contactPoint2', yr.PointsRenderer(rd_contactPoint2, (0,255,255)))
    #viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (255,0,255)))
#    viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (255,0,255)))
#    viewer.doc.addRenderer('rd_dL_des_plane', yr.VectorsRenderer(rd_dL_des_plane, rd_CM, (255,255,0)))
#    viewer.doc.addRenderer('rd_dH_des', yr.VectorsRenderer(rd_dH_des, rd_CM, (0,255,0)))
    #viewer.doc.addRenderer('rd_grf_des', yr.ForcesRenderer(rd_grf_des, rd_CP_des, (0,255,0), .001))
    viewer.doc.addRenderer('rd_vf', yr.ForcesRenderer(rd_vf, rd_CM, (0,0,255), .005))

    global stage
    stage = 0
    
    def simulateCallback(frame):
        motionModel.update(motion[frame])
        
        # tracking
        th_r = motion.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        
        '''
        th2 = controlModel2.getDOFPositions()
        dth2 = controlModel2.getDOFVelocities()
        ddth_des2 = yct.getDesiredDOFAccelerations(th_r, th2, dth_r, dth2, ddth_r, Kt, Dt)
        '''

        #ype.flatten(ddth_des, ddth_des_flat)
        #ype.flatten(dth, dth_flat) 
        

        #Control         

        #tracking control
        #print(Kt, Dt)        
        #Tpd = yct.getDesiredDOFTorques(th_r, th, dth_r, dth, 100.0, 0.1)#0.65, 0.031)          

        linkPositions = controlModel.getBodyPositionsGlobal()
        linkVelocities = controlModel.getBodyVelocitiesGlobal()
        linkAngVelocities = controlModel.getBodyAngVelocitiesGlobal()
        linkInertias = controlModel.getBodyInertiasGlobal()

        jointPositions = controlModel.getJointPositionsGlobal()
        jointAxeses = controlModel.getDOFAxeses()        
        
        CM = yrp.getCM(linkPositions, linkMasses, totalMass)
        dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)            
        footCenter1 = controlModel.getBodyPositionGlobal(sup)
        footCenter2 = controlModel.getBodyPositionGlobal(sup2)
        footCenter = (footCenter1+footCenter2)/2
        '''
        yjc.computeJacobian2(Jt, DOFs, jointPositions, jointAxeses, [CM], CMJointMask)

        pHatCom = CM - footCenter
        vCom = dCM
        '''
        
        CM_plane = copy.copy(CM); CM_plane[1]=0.
        dCM_plane = copy.copy(dCM); dCM_plane[1]=0.
        CM_ref_plane = footCenter
        dL_des_plane = Kl*totalMass*(CM_ref_plane - CM_plane) - Dl*totalMass*dCM_plane        
        dL_des_plane[1] = 0.

        CP_ref = footCenter
        bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
        CP = yrp.getCP(contactPositions, contactForces)
   
        if CP_old[0]==None or CP==None:
            dCP = None
        else:
            dCP = (CP - CP_old[0])/(1/30.)
        CP_old[0] = CP            

        if CP!=None and dCP!=None:
            ddCP_des = Kh*(CP_ref - CP) - Dh*(dCP)
            CP_des = CP + dCP*(1/30.) + .5*ddCP_des*((1/30.)**2)
            dH_des = np.cross((CP_des - CM), (dL_des_plane - totalMass*mm.s2v(wcfg.gravity)))
        else:
            dH_des = None
                
        ################
        '''
        linkPositions = motionModel.getBodyPositionsGlobal()
        linkVelocities = motionModel.getBodyVelocitiesGlobal()
        linkAngVelocities = motionModel.getBodyAngVelocitiesGlobal()
        linkInertias = motionModel.getBodyInertiasGlobal()
                    
        CM2 = yrp.getCM(linkPositions, linkMasses, totalMass)
        dCM2 = yrp.getCM(linkVelocities, linkMasses, totalMass)            
        footCenter1 = motionModel.getBodyPositionGlobal(sup)
        footCenter2 = motionModel.getBodyPositionGlobal(sup2)
        footCenter = (footCenter1+footCenter2)/2
        pHatComDes = CM2 - footCenter
        vComDes = dCM2

        Wcp = -750
        Wcv = -10
        fCom = Wcp*(pHatComDes - pHatCom) + Wcv*(vComDes - vCom)
        '''

        #print("VirtualForce", fCom)
        #fCom[0] = 0.
        #fCom[1] = 0.
        #fCom[2] = -20.
        #fCom = [0., 0., 100.]
        #print("VirtualForce", fCom)
        

        for i in range(stepsPerFrame):
            # apply penalty force
            bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)  
           
                        
            controlModel.setDOFAccelerations(ddth_des)            
            
            linkPositions = controlModel.getBodyPositionsGlobal()
            linkVelocities = controlModel.getBodyVelocitiesGlobal()
            linkAngVelocities = controlModel.getBodyAngVelocitiesGlobal()
            linkInertias = controlModel.getBodyInertiasGlobal()

            jointPositions = controlModel.getJointPositionsGlobal()
            jointAxeses = controlModel.getDOFAxeses()        
        
            CM = yrp.getCM(linkPositions, linkMasses, totalMass)
            dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)            
            footCenter1 = controlModel.getBodyPositionGlobal(sup)
            footCenter2 = controlModel.getBodyPositionGlobal(sup2)
            footCenter = (footCenter1+footCenter2)/2
            
            CM_plane = copy.copy(CM); CM_plane[1]=0.
            footCenter_plane = copy.copy(footCenter); footCenter_plane[1] = 0.

            yjc.computeJacobian2(Jt, DOFs, jointPositions, jointAxeses, [CM], CMJointMask)

            pHatCom = CM - footCenter
            vCom = dCM

            linkPositions2 = motionModel.getBodyPositionsGlobal()
            linkVelocities2 = motionModel.getBodyVelocitiesGlobal()
            linkAngVelocities2 = motionModel.getBodyAngVelocitiesGlobal()
            linkInertias2 = motionModel.getBodyInertiasGlobal()
                    
            CM2 = yrp.getCM(linkPositions2, linkMasses, totalMass)
            dCM2 = yrp.getCM(linkVelocities2, linkMasses, totalMass)            
            footCenter1 = motionModel.getBodyPositionGlobal(sup)
            footCenter2 = motionModel.getBodyPositionGlobal(sup2)
            footCenter = (footCenter1+footCenter2)/2
            pHatComDes = CM2 - footCenter
            vComDes = dCM2

            Wcp = 0
            Wcv = 0
            Wcm = 0
            '''
            0 : initial
            1 : contact
            2 : fly
            3 : landing
            '''

            global stage
            if len(contactForces) == 0 :
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
            

                   
                    
            Wcp = -550
            Wcv = -100
            Wcm = 0 
            
            if stage == 1:
                Wcp = -550
                Wcv = -100
                Wcm = 0
                #Wcp = -550
                #Wcv = -100
            elif stage == 3:
                #Wcp = -950
                #Wcv = -300
                Wcp = -950
                Wcv = -300
                Wcm = 0
            

            # COM Position control
            fCom = Wcp*(pHatComDes - pHatCom) + Wcv*(vComDes - vCom) + Wcm*(footCenter_plane - CM_plane)
            
            if len(contactForces) == 0 :                
                fCom[0] = 0.
                fCom[1] = -10.  #-10
                fCom[2] = 0.

            '''
            fCom[0] = 10.
            fCom[1] = 0.
            fCom[2] = 250.
            '''

            # Angular Momentum control
            L_ref = 0
            L_con = 0
            #R,i
            for i in range(1, controlModel.getBodyNum()):
                L_ref += (linkMasses[i]*np.cross(linkPositions2[i]-[CM], linkVelocities2[i])+linkInertias2[i]*linkAngVelocities2[i])
                L_con += (linkMasses[i]*np.cross(linkPositions[i]-[CM], linkVelocities[i])+linkInertias[i]*linkAngVelocities[i])
            
            for iJoint in range(1, 7): # from 'LeftUpLeg'(1) to 'RightFoot'(6)
                JpT1 = ( Jt[0][6+3*(iJoint-1)], Jt[1][6+3*(iJoint-1)], Jt[2][6+3*(iJoint-1)])
                JpT2 = ( Jt[0][6+3*(iJoint-1)+1], Jt[1][6+3*(iJoint-1)+1], Jt[2][6+3*(iJoint-1)+1])
                JpT3 = ( Jt[0][6+3*(iJoint-1)+2], Jt[1][6+3*(iJoint-1)+2], Jt[2][6+3*(iJoint-1)+2])
                Tfi = (np.dot(JpT1,fCom), np.dot(JpT2,fCom), np.dot(JpT3,fCom))
                currentT = controlModel.getJointAngAccelerationLocal(iJoint)
                controlModel.setJointAngAccelerationLocal(iJoint, currentT+Tfi)
            
            currentT = controlModel.getJointAngAccelerationLocal(0)            
            #print(currentT)
            #JpT = ( Jt[0][0], Jt[1][0], Jt[2][0])
            #Tfi = JpT*fCom 
            #controlModel.setJointAngAccelerationLocal(0, currentT+Tfi)

            '''
            if (len(contactForces) != 0) :
                for iJoint in range(1, 7): # from 'LeftUpLeg'(1) to 'RightFoot'(6)
                    JpT = ( Jt[0][6+3*(iJoint-1)], Jt[1][6+3*(iJoint-1)], Jt[2][6+3*(iJoint-1)])
                    Tfi = JpT*fCom
                    currentT = controlModel.getJointAngAccelerationLocal(iJoint)
                    controlModel.setJointAngAccelerationLocal(iJoint, currentT+Tfi)
            else:
                print("No Contact force!!")
            '''

            controlModel.solveHybridDynamics()
                 

            
            vpWorld.step()
            
                        
        
        # rendering

        rd_CM[0] = CM
        
        rd_CM_plane[0] = CM.copy()
        rd_CM_plane[0][1] = 0.

        if CP!=None and dCP!=None:
            rd_CP[0] = CP
            rd_CP_des[0] = CP_des
        
        rd_dL_des_plane[0] = dL_des_plane
        rd_dH_des[0] = dH_des
        
        rd_grf_des[0] = dL_des_plane - totalMass*mm.s2v(wcfg.gravity)      
        rd_vf[0] = fCom       

    viewer.setSimulateCallback(simulateCallback)
    
    viewer.startTimer(1/30.)
    viewer.show()
    
    Fl.run()
    
main()