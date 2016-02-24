from fltk import *
import copy
import numpy as np
import numpy.linalg as npl
import time

import sys
# if '../PyCommon/modules' not in sys.path:
#     sys.path.append('../PyCommon/modules')
# if './modules' not in sys.path:
#    sys.path.append('./modules')

import math
import Math.mmMath as mm
import Resource.ysMotionLoader as yf
import Renderer.ysRenderer as yr
import Renderer.csVpRenderer as cvr
import Simulator.csVpWorld as cvw
import Simulator.csVpModel as cvm
import Simulator.csVpWorld_tmp as cvwt

import Simulator.hpLCPSimulator as hls
import GUI.hpSimpleViewer as hsv
import Optimization.ysAnalyticConstrainedOpt as yac
import Util.ysPythonEx as ype
import ArticulatedBody.ysJacobian as yjc
import ArticulatedBody.ysReferencePoints as yrp
import ArticulatedBody.ysMomentum as ymt
import ArticulatedBody.ysControl as yct
import GUI.ysMultiViewer as ymv
import Motion.ysHierarchyEdit as yme
import Simulator.ysPhysConfig as ypc

import VirtualPhysics.vpWorld as VPW
import VirtualPhysics.vpBody as VPB
import VirtualPhysics.vpJoint as VPJ
import VirtualPhysics.vpGeom as VPG
import VirtualPhysics.LieGroup as VPL


import Motion.import_trc as trc

import mtOptimize as mot
import mtInitialize_Simple as mit

MOTION_COLOR = (213, 111, 162)
CHARACTER_COLOR = (20, 166, 188)
FEATURE_COLOR = (255, 102, 0)
CHARACTER_COLOR2 = (200, 200, 200)

motion = None
mcfg = None
wcfg = None
stepsPerFrame = None
config = None
mcfg_motion = None

viewer = None


def init():
    global motion
    global mcfg
    global wcfg
    global stepsPerFrame
    global config
    global mcfg_motion
    global viewer


    np.set_printoptions(precision=4, linewidth=200)
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped_basic('../2_Test01_Dynamic.bvh')
    mcfg_motion = mit.normal_mcfg()

    viewer = hsv.hpSimpleViewer([100, 100, 1280, 960])
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion))


init()


class Callback:
    def __init__(self):
        self.frame = 0

    def simulateCallback(self, frame):
        self.frame = frame
        # print "main:frame : ", frame

callback = Callback()

viewer.setSimulateCallback(callback.simulateCallback)

viewer.startTimer(1 / 120.)
viewer.show()

Fl.run()
