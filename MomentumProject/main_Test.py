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
import Simulator.hpLCPSimulator as hls
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
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    
    vpWorld.initialize()
    controlModel.initializeHybridDynamics()
    
    totalDOF = controlModel.getTotalDOF()
    DOFs = controlModel.getDOFs()

    # flat data structure
    ddth_des_flat = ype.makeFlatList(totalDOF)
    dth_flat = ype.makeFlatList(totalDOF)
    ddth_sol = ype.makeNestedList(DOFs)

    #print controlModel.getBodyGravityForceLocal(0)
    #print controlModel.getDOFVelocities()

    def simulateCallback(frame):
        print "main:frame : ", frame
        # motionModel.update(motion[0])

        Kt, Kk, Kl, Kh, Ksc, Bt, Bl, Bh, B_CM, B_CMSd, B_Toe = 1.,1.,1.,1.,1.,1.,1.,1.,1.,1.,1.
        
        Dt = 2*(Kt**.5)
        Dk = 2*(Kk**.5)
        Dl = 2*(Kl**.5)
        Dh = 2*(Kh**.5)
        Dsc = 2*(Ksc**.5)
                
                       
        # tracking
        th_r_ori = motion.getDOFPositions(frame)
        th_r = copy.copy(th_r_ori)


        #th = controlModel.getDOFPositions()
        #dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        #ddth_r = motion.getDOFAccelerations(frame)
        #ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        #ddth_c = controlModel.getDOFAccelerations()


      
        for i in range(stepsPerFrame):
            # apply penalty force
            #bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            #print frame, bodyIDs, contactPositions, contactPositionLocals, contactForces
            #vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)                      
            
            #print ddth_sol
            #controlModel.setDOFAccelerations(ddth_des)
            
            #controlModel.solveHybridDynamics()
            #vpWorld.step()                    
            pass
            
        # print timeReport
        #del th
    #for i in range(4):
    #    simulateCallback(i)

    # viewer.setSimulateCallback(simulateCallback)
    # 
    # viewer.startTimer(1/30.)
    # viewer.show()
    # 
    # Fl.run()

main()

