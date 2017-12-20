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
import mtInitialize_Simple as mit

motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped()
mcfg_motion = mit.normal_mcfg()
    
vpWorld = cvw.VpWorld(wcfg)
motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
motionModel.recordVelByFiniteDiff()
controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
vpWorld.initialize()
controlModel.initializeHybridDynamics()