from fltk import *
import copy
import os.path
from cPickle import load
# import time
import numpy as np

import sys
if ".." not in sys.path:
    sys.path.append("..")

from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.Math import ysFunctionGraph as yfg
from PyCommon.modules.Renderer import ysRenderer as yr
# from PyCommon.modules.Simulator import ysVpUtil as yvu
from PyCommon.modules.GUI import ysSimpleViewer_ori as ysv
from PyCommon.modules.GUI import ysMultiViewer as ymv
# from PyCommon.modules.ArticulatedBody import ysControl as yct
# from PyCommon.modules.ArticulatedBody import ysReferencePoints as yrp
from PyCommon.modules.Motion import ysMotionAnalysis as yma
from PyCommon.modules.Motion import ysBipedAnalysis as yba
from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Motion import ysMotionBlend as ymb
from PyCommon.modules.Motion import ysMotionExtend as ymt
# from PyCommon.modules.Motion import ysSkeletonEdit as yhe
from PyCommon.modules.Motion import mmAnalyticIK as aik
# from PyCommon.modules.Util import ysMatplotEx as ymp
from PyCommon.modules.Resource import ysMotionLoader as yf
from PyCommon.modules.Simulator import ysPhysConfig as ypc

from PyCommon.modules.Simulator import hpDartLCPSimulator as hdls
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Util import ysPythonEx as ype

from PyCommon.modules import pydart2 as pydart
from PyCommon.modules.Simulator import csDartModel as cpm
from pdcontroller import PDController

import hpFootIK as hfi

import math
# from matplotlib import collections

import multiprocessing as mp
import cma

current_path = os.path.dirname(os.path.abspath(__file__))

# MOTION_COLOR = (128,128,128)
# CHARACTER_COLOR = (102,102,153)
MOTION_COLOR = (213, 111, 162)
CHARACTER_COLOR = (20, 166, 188)

MAX_FRAME = 1500

SEGMENT_FOOT = True
SEGMENT_FOOT_MAG = .03
SEGMENT_FOOT_RAD = SEGMENT_FOOT_MAG * .5

def buildMassMap():
    massMap = {}
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips',
                                'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector',
                                'LeftLeg', 'LeftShoulder1', 'LeftUpLeg',
                                'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector',
                                'RightLeg', 'RightShoulder', 'RightUpLeg',
                                'Spine', 'Spine1',
                                'RightFoot_foot_0_0', 'RightFoot_foot_0_1', 'RightFoot_foot_0_1_Effector',
                                'RightFoot_foot_1_0', 'RightFoot_foot_1_1', 'RightFoot_foot_1_1_Effector',
                                'RightFoot_foot_2_0', 'RightFoot_foot_2_1', 'RightFoot_foot_2_1_Effector',
                                'LeftFoot_foot_0_0', 'LeftFoot_foot_0_1', 'LeftFoot_foot_0_1_Effector',
                                'LeftFoot_foot_1_0', 'LeftFoot_foot_1_1', 'LeftFoot_foot_1_1_Effector',
                                'LeftFoot_foot_2_0', 'LeftFoot_foot_2_1', 'LeftFoot_foot_2_1_Effector',
                                ], 0.)

    # torso : 10
    massMap['Hips'] += 2.
    massMap['Spine'] += 8.

    # head : 3
    massMap['Spine1'] += 3.

    # right upper arm : 2
    massMap['RightArm'] += 2.

    # left upper arm : 2
    massMap['LeftArm'] += 2.

    # right lower arm : 1
    massMap['RightForeArm'] = 1.
    #    massMap['RightForeArm'] = 2.

    # left lower arm : 1
    massMap['LeftForeArm'] = 1.
    #    massMap['LeftForeArm'] = 2.

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
    # massMap['RightFoot'] += .4

    # left foot : 4
    massMap['LeftFoot'] += 2.
    # massMap['LeftFoot'] += .4
    '''
    massMap['RightFoot_foot_0_0'] = .3
    massMap['RightFoot_foot_0_1'] = .3
    massMap['RightFoot_foot_1_0'] = .3
    massMap['RightFoot_foot_1_1'] = .3
    massMap['RightFoot_foot_2_0'] = .3
    massMap['RightFoot_foot_2_1'] = .3
    massMap['LeftFoot_foot_0_0'] = .3
    massMap['LeftFoot_foot_0_1'] = .3
    massMap['LeftFoot_foot_1_0'] = .3
    massMap['LeftFoot_foot_1_1'] = .3
    massMap['LeftFoot_foot_2_0'] = .3
    massMap['LeftFoot_foot_2_1'] = .3
    #'''

    massMap['RightFoot_foot_0_0'] = .1
    massMap['RightFoot_foot_0_1'] = .1
    massMap['RightFoot_foot_0_0_0'] = .1
    massMap['RightFoot_foot_0_1_0'] = .1
    massMap['RightFoot_foot_1_0'] = .1
    massMap['RightFoot_foot_1_1'] = .1
    massMap['RightFoot_foot_1_2'] = .1
    massMap['LeftFoot_foot_0_0'] = .1
    massMap['LeftFoot_foot_0_1'] = .1
    massMap['LeftFoot_foot_0_0_0'] = .1
    massMap['LeftFoot_foot_0_1_0'] = .1
    massMap['LeftFoot_foot_1_0'] = .1
    massMap['LeftFoot_foot_1_1'] = .1
    massMap['LeftFoot_foot_1_2'] = .1

    return massMap


def buildMcfg():
    massMap = buildMassMap()
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .9

    totalMass = 0.
    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]
        # totalMass += node.mass

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
    #    node.length = .27
    #    node.offset = (0,0,0.01)
    node.width = .1
    node.geom = 'MyFoot1'

    node = mcfg.getNode('LeftFoot')
    node.length = .25
    #    node.length = .27
    #    node.offset = (0,0,0.01)
    node.width = .1
    node.geom = 'MyFoot1'

    def capsulize(node_name):
        node = mcfg.getNode(node_name)
        node.geom = 'MyFoot4'
        node.width = 0.01
        node.density = 200.
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., math.pi/4., 0.])], ypc.CapsuleMaterial(1000., .02, .2))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., math.pi/4., 0.])], ypc.CapsuleMaterial(1000., .02, .1))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., 0., 0.])], ypc.CapsuleMaterial(1000., .01, -1))
        # node.addGeom('MyFoot4', None, ypc.CapsuleMaterial(1000., .02, .1))

    # capsulize('RightFoot')
    # capsulize('LeftFoot')

    if SEGMENT_FOOT:
        node = mcfg.getNode('RightFoot')
        node.density = 200.
        node.geom = 'MyFoot5'
        node.width = 0.01
        node.jointType = 'B'

        node = mcfg.getNode('LeftFoot')
        node.density = 200.
        node.geom = 'MyFoot5'
        node.width = 0.01
        node.jointType = 'B'

    # bird foot
    # capsulize('RightFoot_foot_0_0')
    # capsulize('RightFoot_foot_0_1')
    # capsulize('RightFoot_foot_1_0')
    # capsulize('RightFoot_foot_1_1')
    # capsulize('RightFoot_foot_2_0')
    # capsulize('RightFoot_foot_2_1')
    # capsulize('LeftFoot_foot_0_0')
    # capsulize('LeftFoot_foot_0_1')
    # capsulize('LeftFoot_foot_1_0')
    # capsulize('LeftFoot_foot_1_1')
    # capsulize('LeftFoot_foot_2_0')
    # capsulize('LeftFoot_foot_2_1')


    # human foot
    if SEGMENT_FOOT:
        footJointType = 'B'
        capsulDensity = 400.

        # RightFoot_foot_0_0 : outside metatarsals
        capsulize('RightFoot_foot_0_0')
        node = mcfg.getNode('RightFoot_foot_0_0')
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-0.3, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.5 + 2.*SEGMENT_FOOT_RAD))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-0.3-1.2, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.5 + 2.*SEGMENT_FOOT_RAD))
        # node.addGeom('MyFoot4', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0., 0., 0.])], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        # RightFoot_foot_0_0_0 : outside phalanges
        capsulize('RightFoot_foot_0_0_0')
        node = mcfg.getNode('RightFoot_foot_0_0_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType

        # RightFoot_foot_0_1 : inside metatarsals
        capsulize('RightFoot_foot_0_1')
        node = mcfg.getNode('RightFoot_foot_0_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity,SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType

        # RightFoot_foot_0_1_0 : inside phalanges
        capsulize('RightFoot_foot_0_1_0')
        node = mcfg.getNode('RightFoot_foot_0_1_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType

        # RightFoot_foot_1_0 : center heel
        capsulize('RightFoot_foot_1_0')
        node = mcfg.getNode('RightFoot_foot_1_0')
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0., 0., .7]), mm.exp([0.]*3)],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2. + SEGMENT_FOOT_RAD * 2.))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        # RightFoot_foot_1_1 : inside heel
        capsulize('RightFoot_foot_1_1')
        node = mcfg.getNode('RightFoot_foot_1_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType

        # RightFoot_foot_1_2 : outside heel
        capsulize('RightFoot_foot_1_2')
        node = mcfg.getNode('RightFoot_foot_1_2')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType


        capsulize('LeftFoot_foot_0_0')
        node = mcfg.getNode('LeftFoot_foot_0_0')
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0.3, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.5+2.*SEGMENT_FOOT_RAD))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0.3+1.2, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.5+2.*SEGMENT_FOOT_RAD))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_0_0_0')
        node = mcfg.getNode('LeftFoot_foot_0_0_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_0_1')
        node = mcfg.getNode('LeftFoot_foot_0_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_0_1_0')
        node = mcfg.getNode('LeftFoot_foot_0_1_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_1_0')
        node = mcfg.getNode('LeftFoot_foot_1_0')
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0., 0., .7]), mm.exp([0.]*3)],
                     ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, SEGMENT_FOOT_MAG*2.0+2.*SEGMENT_FOOT_RAD))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_1_1')
        node = mcfg.getNode('LeftFoot_foot_1_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_1_2')
        node = mcfg.getNode('LeftFoot_foot_1_2')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, SEGMENT_FOOT_RAD, -1))
        node.jointType = footJointType


    return mcfg


def walkings(params, isCma=True):
    """

    :type params: list[float]
    :return:
    """
    class ForceInfo:
        def __init__(self, startFrame, duration, force):
            self.startFrame = startFrame    # frame
            self.duration = duration        # sec
            self.force = force              # Newton
            self.targetBody = None

    #===============================================================================
    # load motion
    #===============================================================================
    MULTI_VIEWER = False
    CAMERA_TRACKING = False
    TORQUE_PLOT = False
    NO_FOOT_SLIDING = True

    # global parameters
    Kt = 20.
    Dt = 2.*(Kt**.5)
    # Dt = Kt/900.
    Ks = 1000.
    Ds = 2.*(Ks**.5)
    mu = 1.
    # Dt = 0.

    # constants
    c_min_contact_vel = 100.
    #    c_min_contact_vel = 2.
    c_min_contact_time = .7
    c_landing_duration = .2
    c_taking_duration = .3
    # c_swf_mid_offset = .02
    c_swf_mid_offset = .0
    c_locking_vel = .05

    c_swf_offset = .0
    # c_swf_offset = .01
    #    c_swf_offset = .005
    K_stp_pos = 0.

    #    c5 = .5;    c6 = .01
    c5 = .5;    c6 = .02
    #    c5 = .5;    c6 = .05
    #    c5 = 1.;    c6 = .05
    #    c5 = .0;    c6 = .0

    K_stb_vel = .1
    K_stb_pos = .1

    OLD_SWING_HEIGHT = False
    # OLD_SWING_HEIGHT = True
    # HIGHER_OFFSET = True
    HIGHER_OFFSET = False

    motionDir = current_path+'/ppmotion/'
    # motionDir = './ppmotion/'
    #
    ##    K_swp_vel_sag = .1; K_swp_vel_cor = .4; K_swp_pos_sag = .3; K_swp_pos_cor = 0.
    #    K_swp_vel_sag = .05; K_swp_vel_cor = .2; K_swp_pos_sag = .2; K_swp_pos_cor = .2
    #    K_swp_pos_sag_faster = .05
    #    filename = 'wd2_WalkSameSame01.bvh'
    ##    filename = 'wd2_WalkSameSame01_REPEATED.bvh'

    ##    K_swp_vel_sag = .1; K_swp_vel_cor = .4; K_swp_pos_sag = .3; K_swp_pos_cor = 0.
    #    K_swp_vel_sag = .05; K_swp_vel_cor = .25; K_swp_pos_sag = .5; K_swp_pos_cor = .2
    #    K_swp_pos_sag_faster = .05
    #    filename = 'wd2_WalkForwardSlow01.bvh'
    ##    filename = 'wd2_WalkForwardSlow01_REPEATED.bvh' # 3 frame diff

    # K_swp_vel_sag = .1; K_swp_vel_cor = .4; K_swp_pos_sag = 1.; K_swp_pos_cor = 0.
    # K_stp_pos = .6
    K_swp_vel_sag = .0; K_swp_vel_cor = .3; K_swp_pos_sag = 1.2; K_swp_pos_cor = .2
    # K_swp_vel_sag = .0; K_swp_vel_cor = 1.3; K_swp_pos_sag = 1.2; K_swp_pos_cor = 1.
    K_swp_pos_sag_faster = .05
    # filename = 'segfoot_wd2_WalkForwardNormal00.bvh'
    filename = 'segfoot_wd2_WalkForwardNormal00_REPEATED.bvh'

    ##    K_swp_vel_sag = .1; K_swp_vel_cor = .4; K_swp_pos_sag = .3; K_swp_pos_cor = 0.
    ##    K_stp_pos = 0.
    #    K_swp_vel_sag = .05; K_swp_vel_cor = .2; K_swp_pos_sag = .3; K_swp_pos_cor = .2
    #    K_swp_pos_sag_faster = .05
    ##    filename = 'wd2_WalkHandWav00.bvh'
    #    filename = 'wd2_WalkHandWav00_REPEATED.bvh'

    #    mu = 2.
    ##    K_swp_vel_sag = .1; K_swp_vel_cor = .4; K_swp_pos_sag = .3; K_swp_pos_cor = 0.
    ##    K_stp_pos = 0.
    #    K_swp_vel_sag = .0; K_swp_vel_cor = .3; K_swp_pos_sag = .2; K_swp_pos_cor = .2
    #    K_swp_pos_sag_faster = .0
    ##    filename = 'wd2_WalkAzuma01.bvh'
    #    filename = 'wd2_WalkAzuma01_REPEATED.bvh'   # 2 frame diff

    ##    K_swp_vel_sag = .1; K_swp_vel_cor = .4; K_swp_pos_sag = 1.; K_swp_pos_cor = 0.
    ##    K_stp_pos = 0.
    #    K_swp_vel_sag = .0; K_swp_vel_cor = .3; K_swp_pos_sag = .2; K_swp_pos_cor = .2
    #    K_swp_pos_sag_faster = .05
    ##    filename = 'wd2_WalkSoldier00.bvh'    # K_swp_pos_sag = .0
    #    filename = 'wd2_WalkSoldier00_REPEATED.bvh'

    # mu = 2.
    # #    K_swp_vel_sag = .2; K_swp_vel_cor = .4; K_swp_pos_sag = .5;K_swp_pos_cor = 0.
    # #    K_stp_pos = 0.
    # K_swp_vel_sag = .05; K_swp_vel_cor = .3; K_swp_pos_sag = .5; K_swp_pos_cor = .2
    # K_swp_pos_sag_faster = .05
    # #    filename = 'wd2_WalkForwardVFast00.bvh'
    # filename = 'wd2_WalkForwardVFast00_REPEATED.bvh'

    ##    K_swp_vel_sag = .0; K_swp_vel_cor = .4; K_swp_pos_sag = .04; K_swp_pos_cor = .1
    ##    K_swp_pos_sag_faster = .02
    ##    K_stb_vel = .2
    #    K_swp_vel_sag = .1; K_swp_vel_cor = .3; K_swp_pos_sag = 1.; K_swp_pos_cor = .3
    #    K_swp_pos_sag_faster = .0
    #    K_stb_vel = .3
    ##    filename = 'wd2_WalkBackward00.bvh'
    #    filename = 'wd2_WalkBackward00_REPEATED.bvh'


    # parameters
    if params is not None:
        _params = np.around(params, decimals=3)
        Ks = 1000.
        Ds                    = 2.*(Ks**.5)
        c_min_contact_vel = 100.
        #    c_min_contact_vel = 2.
        c_min_contact_time = .7
        c_landing_duration = .2
        c_taking_duration = .3
        c_swf_mid_offset = .02
        # c_swf_mid_offset = .0
        c_locking_vel = .05

        c_swf_offset = .0
        # c_swf_offset = .01
        #    c_swf_offset = .005
        K_stp_pos             = _params[0]*_params[0]
        c5                    = _params[1]*_params[1]
        c6                    = _params[2]*_params[2]
        K_stb_vel             = _params[3]*_params[3]
        K_stb_pos             = _params[4]*_params[4]
        K_swp_vel_sag         = _params[5]*_params[5]
        K_swp_vel_cor         = _params[6]*_params[6]
        K_swp_pos_sag         = _params[7]*_params[7]
        K_swp_pos_cor         = _params[8]*_params[8]
        K_swp_pos_sag_faster  = _params[9]*_params[9]

    # motion
    bvh = yf.readBvhFileAsBvh(motionDir+filename)

    if SEGMENT_FOOT:
        # partBvhFilePath = '../PyCommon/modules/samples/simpleJump_long_test2.bvh'
        partBvhFilePath = current_path+'/../PyCommon/modules/samples/simpleJump_long_test2.bvh'
        partBvh = yf.readBvhFileAsBvh(partBvhFilePath)
        bvh.replaceJointFromBvh('RightFoot', partBvh, SEGMENT_FOOT_MAG)
        partBvh = yf.readBvhFileAsBvh(partBvhFilePath)
        partBvh.mirror('YZ')
        bvh.replaceJointFromBvh('LeftFoot', partBvh, SEGMENT_FOOT_MAG)

    motion_ori = bvh.toJointMotion(1., False)

    # motion_ori = yf.readBvhFile(motionDir+filename)
    frameTime = 1/motion_ori.fps

    if 'REPEATED' in filename:
        REPEATED = True
        CAMERA_TRACKING = True
    else:
        REPEATED = False

    #===============================================================================
    # options
    #===============================================================================
    SEGMENT_EDITING =           True
    STANCE_FOOT_STABILIZE =     True
    MATCH_STANCE_LEG =          True
    SWING_FOOT_PLACEMENT =      True
    SWING_FOOT_HEIGHT =         True

    SWING_FOOT_ORIENTATION =    False

    STANCE_FOOT_PUSH =          True
    STANCE_FOOT_BALANCING =     True

    stitch_func = lambda x : 1. - yfg.hermite2nd(x)
    stf_stabilize_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
    match_stl_func = yfg.hermite2nd
    swf_placement_func = yfg.hermite2nd
    swf_height_func = yfg.hermite2nd
    swf_height_sine_func = yfg.sine
    #    stf_balancing_func = yfg.concatenate([yfg.hermite2nd, yfg.one], [c_landing_duration])
    stf_balancing_func = yfg.hermite2nd

    #    forceInfos = [ForceInfo(70, .4, (100,0,0))]
    forceInfos = []

    #===============================================================================
    # initialize character
    #===============================================================================
    # mcfgfile = open(dir + 'mcfg', 'r')
    # mcfg = cPickle.load(mcfgfile)
    # mcfgfile.close()

    mcfg = buildMcfg()

    wcfg = ypc.WorldConfig()
    wcfg.planeHeight = 0.
    wcfg.useDefaultContactModel = False
    wcfg.lockingVel = c_locking_vel
    stepsPerFrame = 50
    wcfg.timeStep = frameTime/stepsPerFrame

    pydart.init()
    dartModel = cpm.DartModel(wcfg, motion_ori[0], mcfg)
    dartMotionModel = None # type: cpm.DartModel
    if not isCma:
        dartMotionModel = cpm.DartModel(wcfg, motion_ori[0], mcfg)
    # q = dartModel.skeleton.q
    # q[0:3] = mm.logSO3(motion_ori.getJointOrientationGlobal(0, 0))
    # q[3:6] = motion_ori.getJointPositionGlobal(0, 0)
    # dartModel.skeleton.set_positions(q)
    # q[3:6] = motion_ori.getJointPositionGlobal(0, 0)
    # pdController = PDController(dartModel.skeleton, wcfg.timeStep, Kt=1000., Dt=50.)
    pdController = PDController(dartModel.skeleton, wcfg.timeStep)
    # dartModel.skeleton.set_controller(pdController)
    # dartModel.world.set_gravity(np.array((0., 0., 0.)))
    dartModel.initializeHybridDynamics()
    dartModel.initializeForwardDynamics()

    # dartModel.getJoint('LeftFoot').set_actuator_type(pydart.Joint.FORCE)
    # dartModel.getJoint('RightFoot').set_actuator_type(pydart.Joint.FORCE)

    #===============================================================================
    # load segment info
    #===============================================================================
    skeleton = motion_ori[0].skeleton

    segname = os.path.splitext(filename)[0]+'.seg'
    segfile = open(motionDir+segname, 'r')
    seginfo = load(segfile)
    segfile.close()

    if not isCma:
        for seg in seginfo:
            print(seg)

    intervals = [info['interval'] for info in seginfo]
    states = [info['state'] for info in seginfo]
    temp_motion = copy.deepcopy(motion_ori)
    segments = yma.splitMotionIntoSegments(temp_motion, intervals)
    if not isCma:
        print(len(intervals), 'segments')
        for i in range(len(intervals)):
            print('%dth'%i, yba.GaitState.text[states[i]], intervals[i], ',',)
        print("")

    motion_seg_orig = ym.JointMotion()
    motion_seg_orig += segments[0]
    motion_seg = ym.JointMotion()
    motion_seg += segments[0]
    motion_stitch = ym.JointMotion()
    motion_stitch += segments[0]

    motion_stf_stabilize = ym.JointMotion()
    motion_match_stl = ym.JointMotion()
    motion_swf_placement = ym.JointMotion()
    motion_swf_height = ym.JointMotion()
    motion_swf_orientation = ym.JointMotion()
    motion_stf_balancing = ym.JointMotion()
    motion_stf_push = ym.JointMotion()
    motion_control = ym.JointMotion()

    motion_debug1 = ym.JointMotion()
    motion_debug2 = ym.JointMotion()
    motion_debug3 = ym.JointMotion()

    P = ym.JointMotion()
    P_hat = ym.JointMotion()
    M_tc = ym.JointMotion()
    M_hat_tc_1 = ym.JointMotion()

    #===============================================================================
    # loop variable
    #===============================================================================
    seg_index = [0]
    acc_offset = [0]
    extended = [False]
    prev_R_swp = [None]
    stl_y_limit_num = [0]
    stl_xz_limit_num = [0]
    avg_dCM = [mm.O_Vec3()]
    #    avg_stf_v = [mm.O_Vec3()]
    #    avg_stf_av = [mm.O_Vec3()]

    #    stf_push_func = [yfg.zero]
    step_length_cur = [0.]

    step_length_tar = [0.]
    step_axis = [mm.O_Vec3()]
    #===============================================================================
    # information
    #===============================================================================
    bodyIDsToCheck = range(dartModel.getBodyNum())
    # bodyIDsToCheck = [dartModel.getBody("LeftFoot").index_in_skeleton(), dartModel.getBody("RightFoot").index_in_skeleton()]
    mus = [mu]*len(bodyIDsToCheck)

    totalMass = dartModel.getTotalMass()
    # bodyMasses = controlModel.getBodyMasses()
    # totalMass = controlModel.getTotalMass()

    # hwangpil
    #extendedFootName = ['Foot_foot_0_0', 'Foot_foot_0_1', 'Foot_foot_1_0',
    #                    'Foot_foot_1_1', 'Foot_foot_2_0', 'Foot_foot_2_1']

    extendedFootName = ['Foot_foot_0_0', 'Foot_foot_0_1', 'Foot_foot_0_0_0', 'Foot_foot_0_1_0', 'Foot_foot_1_0',
                        'Foot_foot_1_1', 'Foot_foot_1_2']

    # extendedFootName = ['Foot_foot_0_1', 'Foot_foot_1_1', 'Foot_foot_2_1']


    ToeName = ['Foot_foot_0_0_0', 'Foot_foot_0_1_0']
    HeelName = ['Foot_foot_1_0', 'Foot_foot_1_1', 'Foot_foot_1_2']

    lIDs = [skeleton.getJointIndex('Left'+name) for name in extendedFootName]
    rIDs = [skeleton.getJointIndex('Right'+name) for name in extendedFootName]

    lIDdic = {'Left'+name:skeleton.getJointIndex('Left'+name) for name in extendedFootName}
    rIDdic = {'Right'+name:skeleton.getJointIndex('Right'+name) for name in extendedFootName}
    footIdDic = lIDdic.copy()
    footIdDic.update(rIDdic)

    lToes = [skeleton.getJointIndex('Left'+name) for name in ToeName]
    rToes = [skeleton.getJointIndex('Right'+name) for name in ToeName]

    lHeels = [skeleton.getJointIndex('Left'+name) for name in HeelName]
    rHeels = [skeleton.getJointIndex('Right'+name) for name in HeelName]

    footDofNames = [] # type: list[str]
    footDofNames += sum(list(['j_Left'+name+'_x', 'j_Left'+name+'_y', 'j_Left'+name+'_z'] for name in extendedFootName), [])
    footDofNames += sum(list(['j_Right'+name+'_x', 'j_Right'+name+'_y', 'j_Right'+name+'_z'] for name in extendedFootName), [])

    footDofs = dartModel.skeleton.dof_indices(footDofNames)
    LeftFootDofs = dartModel.skeleton.dof_indices(['j_LeftFoot_x','j_LeftFoot_y','j_LeftFoot_z'])
    RightFootDofs = dartModel.skeleton.dof_indices(['j_RightFoot_x','j_RightFoot_y','j_RightFoot_z'])

    # controlled foot joint dofs
    if SEGMENT_FOOT:
        variableDofIdx = dartModel.skeleton.dof_indices(footDofNames)
        # joint dofs except foot joint
        specifiedDofIdx = list(range(dartModel.getTotalDOF()))
        for dofidx in variableDofIdx:
            specifiedDofIdx.remove(dofidx)

    # for i in lIDs+rIDs:
    #     controlModel.setHybridDynamics(i, "DYNAMIC")

    # each dof is whether KINEMATIC or not
    hdAccMask = [True]*dartModel.getTotalDOF()
    hdAccMask[:6] = [False]*6
    # for i in lIDs+rIDs:
    #     hdAccMask[3+3*i : 6+3*i] = [False]*3

    # for i in range(1, len(dartModel.skeleton.joints)):
    #     dartModel.skeleton.joints[i].set_actuator_type(pydart.Joint.ACCELERATION)


    lID = dartModel.skeleton.bodynode_index('LeftFoot')
    rID = dartModel.skeleton.bodynode_index('RightFoot')

    lUpLeg = skeleton.getJointIndex('LeftUpLeg');rUpLeg = skeleton.getJointIndex('RightUpLeg')
    lKnee = skeleton.getJointIndex('LeftLeg');   rKnee = skeleton.getJointIndex('RightLeg')
    lFoot = skeleton.getJointIndex('LeftFoot');  rFoot = skeleton.getJointIndex('RightFoot')
    spine = skeleton.getJointIndex('Spine')

    uppers = [skeleton.getJointIndex(name) for name in ['Hips', 'Spine', 'Spine1', 'LeftArm', 'LeftForeArm', 'RightArm', 'RightForeArm']]
    # upperMass = sum([bodyMasses[i] for i in uppers])
    lLegs = [skeleton.getJointIndex(name) for name in ['LeftUpLeg', 'LeftLeg', 'LeftFoot']]
    rLegs = [skeleton.getJointIndex(name) for name in ['RightUpLeg', 'RightLeg', 'RightFoot']]
    allJoints = set(range(skeleton.getJointNum()))


    '''
    footMass = sum([bodyMasses[i] for i in lIDs]) + bodyMasses[lID]
    HeelMass = sum([bodyMasses[i] for i in lHeels])
    ToeMass = sum([bodyMasses[i] for i in lToes])
    print('totalMass: ', totalMass)
    print('footMass: ', footMass)
    print('heelmass: ', HeelMass)
    print('ToeMass: ', ToeMass)
    #'''

    halfFootHeight = 0.05
    if not SEGMENT_FOOT:
        halfFootHeight = dartModel.getBody(lFoot).shapenodes[0].shape.size()[1]/2.

    for fi in forceInfos:
        fi.targetBody = spine

    #===========================================================================
    # data collection
    #===========================================================================
    rhip_torques = []
    rknee_torques = []
    rankle_torques = []
    rankle_torques = []

    #===============================================================================
    # rendering
    #===============================================================================
    rd_CM = [None]; rd_CP = [None]; rd_CMP = [None]
    rd_forces = [None]; rd_force_points = [None]
    rd_torques = []; rd_joint_positions = []

    rd_point1 = [None]
    rd_point2 = [None]
    rd_vec1 = [None];   rd_vecori1 = [None]
    rd_vec2 = [None];   rd_vecori2 = [None]
    rd_frame1 = [None]
    rd_frame2 = [None]

    rd_cForces = [None]
    rd_cPositions = [None]
    rd_cForcesControl = [None]
    rd_cPositionsControl = [None]

    viewer = None
    plot = None

    def getParamVal(paramname):
        return viewer.objectInfoWnd.getVal(paramname)

    # renderer settings
    if not isCma:
        if MULTI_VIEWER:
            viewer = ymv.MultiViewer(800, 655)
            #        viewer = ymv.MultiViewer(800, 655, True)
            viewer.setRenderers1([yr.DartModelRenderer(dartMotionModel, MOTION_COLOR)])
            viewer.setRenderers2([yr.DartModelRenderer(dartModel, (200, 200, 0))])
        else:
            # viewer = ysv.SimpleViewer()
            viewer = hsv.hpSimpleViewer(viewForceWnd=True)
            #    viewer.record(False)
            if not isCma:
                viewer.doc.addRenderer('motionModel', yr.DartModelRenderer(dartMotionModel, (0,150,255), yr.POLYGON_LINE))
            viewer.doc.addRenderer('controlModel', yr.DartModelRenderer(dartModel, (50, 200, 200)))

            viewer.doc.addObject('motion_ori', motion_ori)
            viewer.doc.addRenderer('motion_ori', yr.JointMotionRenderer(motion_ori, (0,100,255), yr.LINK_BONE))
            # viewer.doc.addRenderer('motion_seg_orig', yr.JointMotionRenderer(motion_seg_orig, (0,100,255), yr.LINK_BONE))
            # viewer.doc.addRenderer('motion_seg', yr.JointMotionRenderer(motion_seg, (0,150,255), yr.LINK_BONE))
            # viewer.doc.addRenderer('motion_stitch', yr.JointMotionRenderer(motion_stitch, (0,255,200), yr.LINK_BONE))

            # viewer.doc.addRenderer('motion_stf_stabilize', yr.JointMotionRenderer(motion_stf_stabilize, (255,0,0), yr.LINK_BONE))
            # viewer.doc.addRenderer('motion_match_stl', yr.JointMotionRenderer(motion_match_stl, (255,200,0), yr.LINK_BONE))
            # viewer.doc.addRenderer('motion_swf_placement', yr.JointMotionRenderer(motion_swf_placement, (255,100,255), yr.LINK_BONE))
            # viewer.doc.addRenderer('motion_swf_height', yr.JointMotionRenderer(motion_swf_height, (50,255,255), yr.LINK_BONE))
            # viewer.doc.addRenderer('motion_swf_orientation', yr.JointMotionRenderer(motion_swf_orientation, (255,100,0), yr.LINK_BONE))
            # viewer.doc.addRenderer('motion_stf_push', yr.JointMotionRenderer(motion_stf_push, (50,255,200), yr.LINK_BONE))
            # viewer.doc.addRenderer('motion_stf_balancing', yr.JointMotionRenderer(motion_stf_balancing, (255,100,255), yr.LINK_BONE))
            viewer.doc.addRenderer('motion_control', yr.JointMotionRenderer(motion_control, (255,0,0), yr.LINK_BONE))

            #        viewer.doc.addRenderer('motion_debug1', yr.JointMotionRenderer(motion_debug1, (0,255,0), yr.LINK_BONE))
            #        viewer.doc.addRenderer('motion_debug2', yr.JointMotionRenderer(motion_debug2, (255,0,255), yr.LINK_BONE))
            #        viewer.doc.addRenderer('motion_debug3', yr.JointMotionRenderer(motion_debug3, (255,255,0), yr.LINK_BONE))

            #        viewer.doc.addRenderer('M_tc', yr.JointMotionRenderer(M_tc, (255,255,0), yr.LINK_BONE))
            #        viewer.doc.addRenderer('P_hat', yr.JointMotionRenderer(P_hat, (255,255,0), yr.LINK_BONE))
            #        viewer.doc.addRenderer('P', yr.JointMotionRenderer(P, (255,255,0), yr.LINK_BONE))
            #        viewer.doc.addRenderer('M_hat_tc_1', yr.JointMotionRenderer(M_hat_tc_1, (255,255,0), yr.LINK_BONE))

            #    viewer.doc.addRenderer('rd_CM', yr.PointsRenderer(rd_CM, (255,255,0)))
            #    viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (255,0,0)))
            #    viewer.doc.addRenderer('rd_CMP', yr.PointsRenderer(rd_CMP, (0,255,0)))
            #    viewer.doc.addRenderer('forces', yr.ForcesRenderer(rd_forces, rd_force_points, (255,0,0), ratio=.01, fromPoint=False))
            #        viewer.doc.addRenderer('torques', yr.VectorsRenderer(rd_torques, rd_joint_positions, (255,0,0)))

            viewer.doc.addRenderer('rd_contactForcesControl', yr.VectorsRenderer(rd_cForcesControl, rd_cPositionsControl, (255, 0, 0), .1, 'rd_c1'))
            viewer.doc.addRenderer('rd_contactForces', yr.VectorsRenderer(rd_cForces, rd_cPositions, (0, 255, 0), .1, 'rd_c2'))

            viewer.doc.addRenderer('rd_point1', yr.PointsRenderer(rd_point1, (0,255,0)))
            viewer.doc.addRenderer('rd_point2', yr.PointsRenderer(rd_point2, (255,0,0)))
        #        viewer.doc.addRenderer('rd_vec1', yr.VectorsRenderer(rd_vec1, rd_vecori1, (255,0,0)))
        #    viewer.doc.addRenderer('rd_vec2', yr.VectorsRenderer(rd_vec2, rd_vecori2, (0,255,0)))
        #    viewer.doc.addRenderer('rd_frame1', yr.FramesRenderer(rd_frame1, (0,200,200)))
        #    viewer.doc.addRenderer('rd_frame2', yr.FramesRenderer(rd_frame2, (200,200,0)))
        #    viewer.setMaxFrame(len(motion_ori)-1)


        viewer.objectInfoWnd.add1DSlider("penalty_grf_gain",    0., 5000., 10., Ks)
        viewer.objectInfoWnd.add1DSlider("c_min_contact_vel",   0., 200., .2, 100.)
        viewer.objectInfoWnd.add1DSlider("c_min_contact_time",  0., 5., .01, .7)
        viewer.objectInfoWnd.add1DSlider("c_landing_duration",  0., 5., .01, .2)
        viewer.objectInfoWnd.add1DSlider("c_taking_duration",   0., 5., .01, .3)
        viewer.objectInfoWnd.add1DSlider("c_swf_mid_offset",    -1., 1., .001, c_swf_mid_offset)
        viewer.objectInfoWnd.add1DSlider("c_locking_vel",       0., 1., .001, .05)

        viewer.objectInfoWnd.add1DSlider("c_swf_offset",        -1., 1., .001, .01)
        viewer.objectInfoWnd.add1DSlider("K_stp_pos",           0., 1., .01, 0.)

        viewer.objectInfoWnd.add1DSlider("c5",                  0., 5., .01, c5)
        viewer.objectInfoWnd.add1DSlider("c6",                  0., 1., .01, c6)
        viewer.objectInfoWnd.add1DSlider("K_stb_vel",           0., 1., .01, K_stb_vel)
        viewer.objectInfoWnd.add1DSlider("K_stb_pos",           0., 1., .01, K_stb_pos)
        viewer.objectInfoWnd.add1DSlider("K_swp_vel_sag",       0., 5., .01, K_swp_vel_sag)
        viewer.objectInfoWnd.add1DSlider("K_swp_vel_cor",       0., 5., .01, K_swp_vel_cor)
        viewer.objectInfoWnd.add1DSlider("K_swp_pos_sag",       0., 5., .01, K_swp_pos_sag)
        viewer.objectInfoWnd.add1DSlider("K_swp_pos_cor",       0., 5., .01, K_swp_pos_cor)
        viewer.objectInfoWnd.add1DSlider("K_swp_pos_sag_faster",0., 1., .01, K_swp_pos_sag_faster)


        viewer.objectInfoWnd.add1DSlider("LeftFootKp",          0., 500., 10., 300.)
        viewer.objectInfoWnd.add1DSlider("LeftFootKd",          0., 100., 1., 30.)
        viewer.objectInfoWnd.add1DSlider("RightFootKp",          0., 500., 10., 300.)
        viewer.objectInfoWnd.add1DSlider("RightFootKd",          0., 100., 1., 30.)

        viewer.cForceWnd.addDataSet('expForce', FL_BLACK)
        viewer.cForceWnd.addDataSet('desForceMin', FL_RED)
        viewer.cForceWnd.addDataSet('desForceMax', FL_RED)
        viewer.cForceWnd.addDataSet('realForce', FL_GREEN)


        if not REPEATED:
            viewer.setMaxFrame(len(motion_ori)-1)
        else:
            viewer.setMaxFrame(MAX_FRAME)

        if CAMERA_TRACKING:
            if MULTI_VIEWER:
                cameraTargets1 = [None] * (viewer.getMaxFrame()+1)
                cameraTargets2 = [None] * (viewer.getMaxFrame()+1)
            else:
                cameraTargets = [None] * (viewer.getMaxFrame()+1)

        if TORQUE_PLOT:
            rhip_torques = [0.]*viewer.getMaxFrame()
            rknee_torques = [0.]*viewer.getMaxFrame()
            rankle_torques = [0.]*viewer.getMaxFrame()


        # ===============================================================================
        # viewer setting for parameter setting
        # ===============================================================================

        #    pt = [0.]
        def postFrameCallback_Always(frame):
            #        if frame==1: pt[0] = time.time()
            #        if frame==31: print 'elapsed time for 30 frames:', time.time()-pt[0]
            if CAMERA_TRACKING:
                if MULTI_VIEWER:
                    if cameraTargets1[frame] is None:
                        # cameraTargets1[frame] = motionModel.getBodyPositionGlobal(0)
                        cameraTargets1[frame] = dartMotionModel.getBodyPositionGlobal(0)
                    #                    cameraTargets1[frame] = motion_ori[frame].getJointPositionGlobal(0)
                    viewer.setCameraTarget1(cameraTargets1[frame])

                    if cameraTargets2[frame] is None:
                        # cameraTargets2[frame] = controlModel.getJointPositionGlobal(0)
                        cameraTargets2[frame] = dartModel.getJointPositionGlobal(0)
                    viewer.setCameraTarget2(cameraTargets2[frame])

                else:
                    if cameraTargets[frame] is None:
                        cameraTargets[frame] = dartModel.getJointPositionGlobal(0)
                        # cameraTargets[frame] = controlModel.getJointPositionGlobal(0)
                    viewer.setCameraTarget(cameraTargets[frame])
            if plot is not None:
                plot.updateVline(frame)


        if not isCma:
            viewer.setPostFrameCallback_Always(postFrameCallback_Always)

        # plot = ymp.InteractivePlot()
        if plot is not None:
            plot.setXlimit(0, len(motion_ori))
            plot.setYlimit(-0.05, .05)
            plot.addDataSet('zero')
            plot.addDataSet('diff')
            plot.addDataSet('debug1')
            plot.addDataSet('debug2')


        def viewer_onClose(data):
            if plot is not None:
                plot.close()
            viewer.onClose(data)
        viewer.callback(viewer_onClose)

    if not isCma:
        for bodynode in dartModel.skeleton.bodynodes:
            print(bodynode.name, bodynode.mass())


    def simulateCallback(frame):
        # c_min_contact_vel, c_min_contact_time, c_landing_duration, \
        # c_taking_duration, c_swf_mid_offset, c_locking_vel, c_swf_offset, \
        # K_stp_pos, c5, c6, K_stb_vel, K_stb_pos, K_swp_vel_sag, K_swp_vel_cor, \
        # K_swp_pos_sag, K_swp_pos_cor, K_swp_pos_sag_faster = viewer.objectInfoWnd.getVals()
        if not isCma:
        # if not isCma and params is None:
            Ks                    = getParamVal("penalty_grf_gain")
            Ds                    = 2.*(Ks**.5)
            c_min_contact_vel     = getParamVal("c_min_contact_vel")
            c_min_contact_time    = getParamVal("c_min_contact_time")
            c_landing_duration    = getParamVal("c_landing_duration")
            c_taking_duration     = getParamVal("c_taking_duration")
            c_swf_mid_offset      = getParamVal("c_swf_mid_offset")
            c_locking_vel         = getParamVal("c_locking_vel")
            c_swf_offset          = getParamVal("c_swf_offset")
            K_stp_pos             = getParamVal("K_stp_pos")
            c5                    = getParamVal("c5")
            c6                    = getParamVal("c6")
            K_stb_vel             = getParamVal("K_stb_vel")
            K_stb_pos             = getParamVal("K_stb_pos")
            K_swp_vel_sag         = getParamVal("K_swp_vel_sag")
            K_swp_vel_cor         = getParamVal("K_swp_vel_cor")
            K_swp_pos_sag         = getParamVal("K_swp_pos_sag")
            K_swp_pos_cor         = getParamVal("K_swp_pos_cor")
            K_swp_pos_sag_faster  = getParamVal("K_swp_pos_sag_faster")
        elif params is not None:
            _params = np.around(params, decimals=3)
            Ks = 1000.
            Ds = 2. * (Ks ** .5)
            c_min_contact_vel = 100.
            #    c_min_contact_vel = 2.
            c_min_contact_time = .7
            c_landing_duration = .2
            c_taking_duration = .3
            c_swf_mid_offset = .02
            c_locking_vel = .05

            #    c_swf_offset = .0
            c_swf_offset = .01
            #    c_swf_offset = .005
            K_stp_pos = _params[0] * _params[0]
            c5 = _params[1] * _params[1]
            c6 = _params[2] * _params[2]
            K_stb_vel = _params[3] * _params[3]
            K_stb_pos = _params[4] * _params[4]
            K_swp_vel_sag = _params[5] * _params[5]
            K_swp_vel_cor = _params[6] * _params[6]
            K_swp_pos_sag = _params[7] * _params[7]
            K_swp_pos_cor = _params[8] * _params[8]
            K_swp_pos_sag_faster = _params[9] * _params[9]

        # print c_swf_mid_offset

        # seginfo
        segIndex = seg_index[0]
        curState = seginfo[segIndex]['state']
        curInterval = yma.offsetInterval(acc_offset[0], seginfo[segIndex]['interval'])
        stanceLegs = seginfo[segIndex]['stanceHips']
        swingLegs = seginfo[segIndex]['swingHips']
        stanceFoots = seginfo[segIndex]['stanceFoots']
        swingFoots = seginfo[segIndex]['swingFoots']
        swingKnees = seginfo[segIndex]['swingKnees']
        groundHeight = seginfo[segIndex]['ground_height']
        maxStfPushFrame = seginfo[segIndex]['max_stf_push_frame']

        # hwangpil
        # temporary change
        for legList in (stanceLegs, swingLegs):
            for legIdx in range(len(legList)):
                if legList[legIdx] == 10:
                    legList[legIdx] = skeleton.getJointIndex('RightUpLeg')

        for footList in (stanceFoots, swingFoots):
            for footIdx in range(len(footList)):
                if footList[footIdx] == 12:
                    footList[footIdx] = skeleton.getJointIndex('RightFoot')

        stanceToes = []
        if skeleton.getJointIndex('LeftFoot') in stanceFoots:
            stanceToes.extend(lToes)
        if skeleton.getJointIndex('RightFoot') in stanceFoots:
            stanceToes.extend(rToes)

        swingHeels = []
        if skeleton.getJointIndex('LeftFoot') in swingFoots:
            swingHeels.extend(lHeels)
        if skeleton.getJointIndex('RightFoot') in swingFoots:
            swingHeels.extend(rHeels)


        prev_frame = frame-1 if frame>0 else 0
        #        prev_frame = frame

        # information
        #        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(frame), bodyMasses, upperMass, uppers)
        #        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(frame), bodyMasses, upperMass, uppers)
        ##        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(frame), bodyMasses, totalMass)
        ##        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(frame), bodyMasses, totalMass)
        #        stf_tar = motion_seg.getJointPositionGlobal(stanceFoots[0], frame)
        #        CMr_tar = CM_tar - stf_tar

        dCM_tar = motion_seg.getJointVelocityGlobal(0, prev_frame)
        CM_tar = motion_seg.getJointPositionGlobal(0, prev_frame)
        #        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(prev_frame), bodyMasses, upperMass, uppers)
        #        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(prev_frame), bodyMasses, upperMass, uppers)
        #        dCM_tar = yrp.getCM(motion_seg.getJointVelocitiesGlobal(prev_frame), bodyMasses, totalMass)
        #        CM_tar = yrp.getCM(motion_seg.getJointPositionsGlobal(prev_frame), bodyMasses, totalMass)
        stf_tar = motion_seg.getJointPositionGlobal(stanceFoots[0], prev_frame)
        CMr_tar = CM_tar - stf_tar

        # dCM : average velocity of root of controlModel over 1 frame
        dCM = avg_dCM[0]
        # CM = controlModel.getJointPositionGlobal(0)
        CM = dartModel.getBody("Hips").com()
        #        CM = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, upperMass, uppers)
        #        CM = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, totalMass)
        # CMreal = yrp.getCM(controlModel.getJointPositionsGlobal(), bodyMasses, totalMass)
        CMreal = dartModel.getCOM()
        stf = dartModel.getJointPositionGlobal(stanceFoots[0])
        # stf = controlModel.getJointPositionGlobal(stanceFoots[0])
        CMr = CM - stf

        diff_dCM = mm.projectionOnPlane(dCM-dCM_tar, (1,0,0), (0,0,1))
        diff_dCM_axis = np.cross((0,1,0), diff_dCM)
        rd_vec1[0] = diff_dCM; rd_vecori1[0] = CM_tar

        diff_CMr = mm.projectionOnPlane(CMr-CMr_tar, (1,0,0), (0,0,1))
        #        rd_vec1[0] = diff_CMr; rd_vecori1[0] = stf_tar
        diff_CMr_axis = np.cross((0,1,0), diff_CMr)

        direction = mm.normalize2(mm.projectionOnPlane(dCM_tar, (1,0,0), (0,0,1)))
        #        direction = mm.normalize2(mm.projectionOnPlane(dCM, (1,0,0), (0,0,1)))
        directionAxis = np.cross((0,1,0), direction)

        diff_dCM_sag, diff_dCM_cor = mm.projectionOnVector2(diff_dCM, direction)
        #        rd_vec1[0] = diff_dCM_sag; rd_vecori1[0] = CM_tar
        diff_dCM_sag_axis = np.cross((0,1,0), diff_dCM_sag)
        diff_dCM_cor_axis = np.cross((0,1,0), diff_dCM_cor)

        diff_CMr_sag, diff_CMr_cor = mm.projectionOnVector2(diff_CMr, direction)
        diff_CMr_sag_axis = np.cross((0,1,0), diff_CMr_sag)
        diff_CMr_cor_axis = np.cross((0,1,0), diff_CMr_cor)

        t = (frame-curInterval[0])/float(curInterval[1]-curInterval[0])
        t_raw = t
        if t>1.: t=1.


        p_root = motion_stitch[frame].getJointPositionGlobal(0)
        R_root = motion_stitch[frame].getJointOrientationGlobal(0)

        motion_seg_orig.goToFrame(frame)
        motion_seg.goToFrame(frame)
        motion_stitch.goToFrame(frame)

        motion_debug1.append(motion_stitch[frame].copy())
        motion_debug1.goToFrame(frame)
        motion_debug2.append(motion_stitch[frame].copy())
        motion_debug2.goToFrame(frame)
        motion_debug3.append(motion_stitch[frame].copy())
        motion_debug3.goToFrame(frame)

        # paper implementation
        M_tc.append(motion_stitch[prev_frame])
        M_tc.goToFrame(frame)
        P_hat.append(M_tc[frame].copy())
        P_hat.goToFrame(frame)

        # p_temp = ym.JointPosture(skeleton)
        # p_temp.rootPos = controlModel.getJointPositionGlobal(0)
        # p_temp.setJointOrientationsLocal(controlModel.getJointOrientationsLocal())
        # P.append(p_temp)
        # P.goToFrame(frame)

        '''
        # Jacobian Transpose Balance Control
        balanceKp = 100.
        balanceKd = 100.
        balanceDiff = dartMotionModel.getCOM() - dartModel.getCOM()
        balanceDiff[1] = 0.
        balanceVelDiff = -dartModel.skeleton.com_velocity()
        balanceVelDiff[1] = 0.
        balanceTorque = np.dot(dartModel.getBody('RightFoot').world_jacobian()[3:6].T,
                               balanceKp*balanceDiff + balanceKd*balanceVelDiff)
        balanceTorque[:6] = np.array([0.]*6)
        '''

        '''
        # stance foot stabilize
        motion_stf_stabilize.append(motion_stitch[frame].copy())
        motion_stf_stabilize.goToFrame(frame)
        if STANCE_FOOT_STABILIZE:
            for stanceFoot in stanceFoots:
                R_target_foot = motion_seg[frame].getJointOrientationGlobal(stanceFoot)
                R_current_foot = motion_stf_stabilize[frame].getJointOrientationGlobal(stanceFoot)
                motion_stf_stabilize[frame].setJointOrientationGlobal(stanceFoot, mm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))
                # motion_stf_stabilize[frame].setJointOrientationGlobal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))
                #                R_target_foot = motion_seg[frame].getJointOrientationLocal(stanceFoot)
                #                R_current_foot = motion_stf_stabilize[frame].getJointOrientationLocal(stanceFoot)
                #                motion_stf_stabilize[frame].setJointOrientationLocal(stanceFoot, cm.slerp(R_current_foot, R_target_foot , stf_stabilize_func(t)))
        #'''

        # match stance leg
        # motion_match_stl.append(motion_stf_stabilize[frame].copy())
        motion_match_stl.append(motion_stitch[frame].copy())
        motion_match_stl.goToFrame(frame)
        if MATCH_STANCE_LEG:
            if curState!=yba.GaitState.STOP:
                for stanceLegIdx in range(len(stanceLegs)):
                    stanceLeg = stanceLegs[stanceLegIdx]
                    # stanceFoot = stanceFoots[stanceLegIdx]

                    #                    # motion stance leg -> character stance leg as time goes
                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
                    # R_character = controlModel.getJointOrientationGlobal(stanceLeg)
                    R_character = dartModel.getJointOrientationGlobal(stanceLeg)
                    # motion_ori[0].skeleton.getJointName(stanceLeg)
                    # motion_match_stl[frame].setJointOrientationGlobal(stanceLeg, cm.slerp(R_motion, R_character, match_stl_func(t)))
                    motion_match_stl[frame].setJointOrientationGlobal(stanceLeg, mm.slerp(R_motion, R_character, match_stl_func(t)))

                    #                    t_y = match_stl_func_y(t)
                    #                    t_xz = match_stl_func(t)
                    #
                    #                    R_motion = motion_match_stl[frame].getJointOrientationGlobal(stanceLeg)
                    #                    R_character = controlModel.getJointOrientationGlobal(stanceLeg)
                    #                    R = np.dot(R_character, R_motion.T)
                    #                    R_y, R_xz = mm.projectRotation((0,1,0), R)
                    #                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_xz, t_xz))
                    #                    motion_match_stl[frame].mulJointOrientationGlobal(stanceLeg, mm.scaleSO3(R_y, t_y))

        # swing foot placement
        motion_swf_placement.append(motion_match_stl[frame].copy())
        motion_swf_placement.goToFrame(frame)
        if SWING_FOOT_PLACEMENT:
            t_swing_foot_placement = swf_placement_func(t)

            if extended[0]:
                R_swp_sag = prev_R_swp[0][0]
                R_swp_cor = prev_R_swp[0][1]
            else:
                R_swp_sag = mm.I_SO3(); R_swp_cor = mm.I_SO3()
                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * K_swp_vel_sag * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_dCM_cor_axis * K_swp_vel_cor * -t_swing_foot_placement))
                if np.dot(direction, diff_CMr_sag) < 0:
                    R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * K_swp_pos_sag * -t_swing_foot_placement))
                else:
                    R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_CMr_sag_axis * K_swp_pos_sag_faster * -t_swing_foot_placement))
                R_swp_cor = np.dot(R_swp_cor, mm.exp(diff_CMr_cor_axis * K_swp_pos_cor * -t_swing_foot_placement))

            for i in range(len(swingLegs)):
                swingLeg = swingLegs[i]
                swingFoot = swingFoots[i]

                # save swing foot global orientation
                # R_swf = motion_swf_placement[frame].getJointOrientationGlobal(swingFoot)

                # rotate swing leg
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_sag)
                motion_swf_placement[frame].mulJointOrientationGlobal(swingLeg, R_swp_cor)

                # restore swing foot global orientation
                # motion_swf_placement[frame].setJointOrientationGlobal(swingFoot, R_swf)

                # hwangpil
                # temporal code.... for heel strike and ankle pushup
                # motion_swf_placement[frame].mulJointOrientationGlobal(swingFoot, mm.exp([0., 0., -0.17*t_swing_foot_placement]))
                # motion_swf_placement[frame].mulJointOrientationGlobal(swingFoot, mm.exp([0.2*t_swing_foot_placement, 0., 0.]))

                prev_R_swp[0] = (R_swp_sag, R_swp_cor)

        # swing foot height
        motion_swf_height.append(motion_swf_placement[frame].copy())
        motion_swf_height.goToFrame(frame)
        if SWING_FOOT_HEIGHT:
            for swingFoot in swingFoots:
                stanceFoot = stanceFoots[0]

                # save foot global orientation
                R_foot = motion_swf_height[frame].getJointOrientationGlobal(swingFoot)
                R_stance_foot = motion_swf_height[frame].getJointOrientationGlobal(stanceFoot)

                d_height_tar = 0
                if OLD_SWING_HEIGHT:
                    height_tar = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] \
                                 - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]
                else:
                    height_tar = motion_swf_height[prev_frame].getJointPositionGlobal(swingFoot)[1] - groundHeight
                    d_height_tar = motion_swf_height.getJointVelocityGlobal(swingFoot, prev_frame)[1]
                #                motion_debug1[frame] = motion_swf_height[frame].copy()

                # rotate
                # motion_swf_height[frame].rotateByTarget(controlModel.getJointOrientationGlobal(0))
                motion_swf_height[frame].rotateByTarget(dartModel.getJointOrientationGlobal(0))
                #                motion_debug2[frame] = motion_swf_height[frame].copy()
                #                motion_debug2[frame].translateByTarget(controlModel.getJointPositionGlobal(0))

                d_height_cur = 0
                if OLD_SWING_HEIGHT:
                    height_cur = motion_swf_height[frame].getJointPositionGlobal(swingFoot)[1] \
                                 - motion_swf_height[frame].getJointPositionGlobal(stanceFoot)[1]
                else:
                    height_cur = dartModel.getJointPositionGlobal(swingFoot)[1] - halfFootHeight - c_swf_offset
                    d_height_cur = dartModel.getJointVelocityGlobal(swingFoot)[1]

                if OLD_SWING_HEIGHT:
                    offset_height = (height_tar - height_cur) * swf_height_func(t) * c5
                else:
                    offset_height = ((height_tar - height_cur) * c5
                                     + (d_height_tar - d_height_cur) * c6) * swf_height_func(t)

                offset_sine = c_swf_mid_offset * swf_height_sine_func(t)

                offset = 0.
                offset += offset_height
                offset += offset_sine

                if offset > 0.:
                    newPosition =  motion_swf_height[frame].getJointPositionGlobal(swingFoot)
                    newPosition[1] += offset
                    aik.ik_analytic(motion_swf_height[frame], swingFoot, newPosition)
                else:
                    if HIGHER_OFFSET:
                        newPosition =  motion_swf_height[frame].getJointPositionGlobal(stanceFoot)
                        newPosition[1] -= offset
                        aik.ik_analytic(motion_swf_height[frame], stanceFoot, newPosition)

                        # return
                        #                motion_debug3[frame] = motion_swf_height[frame].copy()
                        #                motion_debug3[frame].translateByTarget(controlModel.getJointPositionGlobal(0))
                motion_swf_height[frame].rotateByTarget(R_root)

                # restore foot global orientation
                motion_swf_height[frame].setJointOrientationGlobal(swingFoot, R_foot)
                motion_swf_height[frame].setJointOrientationGlobal(stanceFoot, R_stance_foot)

                if plot is not None:
                    plot.addDataPoint('debug1', frame, offset_height)
                    #                    plot.addDataPoint('debug2', frame, height_cur)
                    #                    plot.addDataPoint('diff', frame, diff)

        # stance foot push
        motion_stf_push.append(motion_swf_height[frame].copy())
        motion_stf_push.goToFrame(frame)
        if STANCE_FOOT_PUSH:
            # TODO:
            # swingFoots?????????????????????????
            # for swingFoot in swingFoots:
            for swingFoot in stanceFoots:
                #                max_t = (maxStfPushFrame)/float(curInterval[1]-curInterval[0])
                #                stf_push_func = yfg.concatenate([yfg.sine, yfg.zero], [max_t*2])
                stf_push_func = yfg.concatenate([yfg.sine, yfg.zero], [c_taking_duration*2])

                R_swp_sag = mm.I_SO3()
                #                R_swp_sag = np.dot(R_swp_sag, mm.exp(diff_dCM_sag_axis * K_stp_vel * -stf_push_func(t)))

                #                if step_length_cur[0] < step_length_tar[0]:
                #                    ratio = step_length_cur[0] / step_length_tar[0]
                #                    R_max = maxmaxStfPushFrame
                #                    R_zero =
                R_swp_sag = np.dot(R_swp_sag, mm.exp((step_length_tar[0] - step_length_cur[0])*step_axis[0] * K_stp_pos * -stf_push_func(t)))

                motion_stf_push[frame].mulJointOrientationGlobal(swingFoot, R_swp_sag)

        # '''
        # stance foot stabilize
        motion_stf_stabilize.append(motion_stf_push[frame].copy())
        motion_stf_stabilize.goToFrame(frame)
        if STANCE_FOOT_STABILIZE:
            for stanceFoot in stanceFoots:
                R_target_foot = motion_stf_push[frame].getJointOrientationGlobal(stanceFoot)
                R_current_foot = motion_stf_stabilize[frame].getJointOrientationGlobal(stanceFoot)
                motion_stf_stabilize[frame].setJointOrientationGlobal(stanceFoot,
                                                      mm.slerp(R_current_foot, R_target_foot, stf_stabilize_func(t)))
        #'''

        # stance foot balancing
        # motion_stf_balancing.append(motion_stf_push[frame].copy())
        motion_stf_balancing.append(motion_stf_stabilize[frame].copy())
        motion_stf_balancing.goToFrame(frame)
        if STANCE_FOOT_BALANCING:
            R_stb = mm.exp(diff_dCM_axis * K_stb_vel * stf_balancing_func(t))
            R_stb = np.dot(R_stb, mm.exp(diff_CMr_axis * K_stb_pos * stf_balancing_func(t)))
            for stanceFoot in stanceFoots:
                if frame < 5: break
                motion_stf_balancing[frame].mulJointOrientationGlobal(stanceFoot, R_stb)


        # hwangpil
        # swing foot parallelizing with ground
        def swf_par_func(x):
            if x<.5:
                return -.5*math.pow(1.-2.*x, 1./3.) + .5
            else:
                return .5*math.pow(2.*x-1., 1./3.) + .5

        if False:
            for swingFoot in swingFoots:
                swingBody = dartModel.getBody(swingFoot)
                for shapeNode in swingBody.shapenodes:
                    if shapeNode.has_collision_aspect():
                        geomType = shapeNode.shape.shape_type_name()
                        geomT = np.dot(swingBody.world_transform(), shapeNode.relative_transform())
                        if geomType == "BOX":
                            shape = shapeNode.shape # type: pydart.BoxShape
                            data = shape.size() * .5
                            footVec = np.dot(geomT[:3, :3], np.array((0., 1., 0.)))
                            R_swf_current = mm._I_SO3
                            R_swf_par = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                            motion_stf_balancing[frame].mulJointOrientationGlobal(swingFoot,
                                                                  mm.slerp(R_swf_current, R_swf_par, swf_par_func(t)))

        '''
        # swing foot heel strike adjustment
        # make heel as flat as possible to ground
        swf_heel_func = yfg.hermite2nd
        for swingHeel in swingHeels:
            joint_vec_cur = np.dot(dartModel.getJointOrientationGlobal(swingHeel), np.array((0., 0., 1.)))
            joint_vec_tar = copy.deepcopy(joint_vec_cur)
            joint_vec_tar[1] = 0.
            R_target_heel = mm.exp(swf_heel_func(t)*mm.logSO3(mm.getSO3FromVectors(joint_vec_cur, joint_vec_tar)))
            motion_stf_balancing[frame].mulJointOrientationGlobal(swingHeel, R_target_heel)
        # stance foot ankle pushup adjustment
        # stf_ankle_func = yfg.hermite2nd
        stf_ankle_func = lambda x: -2*(x**2)+3*(x**3)
        if len(stanceFoots) == 1:
            for stanceFoot in stanceFoots:
                R_target_ankle = mm.exp(stf_ankle_func(t)*mm.deg2Rad(30.)*np.array([1., 0., 0.]))
                motion_stf_balancing[frame].mulJointOrientationLocal(stanceFoot, R_target_ankle)
        # stance foot toe adjustment
        # stf_toe_func = yfg.hermite2nd
        stf_toe_func = lambda x: -2*(x**8)+3*(x**9)
        if len(stanceFoots) == 1:
            for stanceToe in stanceToes:
                # joint_vec_cur = np.dot(controlModel.getJointOrientationGlobal(stanceToe), np.array((0., 0., 1.)))
                ## joint_vec_cur = np.dot(motion_stf_balancing[frame].getJointOrientationGlobal(stanceToe), np.array((0., 0., 1.)))
                # joint_vec_tar = copy.deepcopy(joint_vec_cur)
                # joint_vec_tar[1] = 0.
                ## R_target_toe = mm.exp(stf_toe_func(t)*mm.logSO3(mm.getSO3FromVectors(joint_vec_cur, joint_vec_tar)))
                # R_target_toe = mm.getSO3FromVectors(joint_vec_cur, joint_vec_tar)
                # motion_stf_balancing[frame].mulJointOrientationGlobal(stanceToe, R_target_toe)
                R_target_toe = mm.exp(stf_toe_func(t)*mm.deg2Rad(-30.)*np.array([1., 0., 0.]))
                motion_stf_balancing[frame].mulJointOrientationLocal(stanceToe, R_target_toe)
        #'''

        # foot adjustment
        hfi.footAdjust(motion_stf_balancing[frame], footIdDic, SEGMENT_FOOT_MAG, SEGMENT_FOOT_RAD, .03)


        # control trajectory
        # motion_control.append(motion_stitch[frame].copy())
        # motion_control.append(motion_swf_height[frame].copy())
        # motion_control.append(motion_match_stl[frame].copy())
        motion_control.append(motion_stf_balancing[frame].copy())
        motion_control.goToFrame(frame)

        #=======================================================================
        # tracking with inverse dynamics
        #=======================================================================

        weightMap = [1.] * (skeleton.getJointNum())

        if False:
            toeWeights = 0.001

            for jointIdx in lIDs:
                weightMap[jointIdx] = toeWeights

            for jointIdx in rIDs:
                weightMap[jointIdx] = toeWeights

        th_r = motion_control.getDOFPositions(frame)
        # th_r = motion_stitch.getDOFPositions(frame)
        th = dartModel.skeleton.q
        # th = controlModel.getDOFPositions()
        dth_r = motion_control.getDOFVelocities(frame)
        dth = dartModel.skeleton.dq
        # dth = controlModel.getDOFVelocities()
        ddth_r = motion_control.getDOFAccelerations(frame)
        # ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt, weightMap)

        totalDOF = dartModel.getTotalDOF()
        ddth_des_flat = ype.makeFlatList(totalDOF)
        dth_r_flat = ype.makeFlatList(totalDOF)
        # ype.flatten(ddth_des, ddth_des_flat)
        # ype.flatten(dth_r, dth_r_flat)

        # print dartModel.skeleton.q[:6]
        # print dartModel.getBody(0).com(), dartModel.skeleton.joint(0).position_in_world_frame(), dartModel.skeleton.q[:6]

        #=======================================================================
        # simulation
        #=======================================================================
        CP = mm.v3(0.,0.,0.)
        F = mm.v3(0.,0.,0.)
        avg_dCM[0] = mm.v3(0.,0.,0.)

        # external force rendering info
        if not isCma:
            del rd_forces[:]; del rd_force_points[:]
            for fi in forceInfos:
                if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/frameTime):
                    rd_forces.append(fi.force)
                    # rd_force_points.append(controlModel.getBodyPositionGlobal(fi.targetBody))
                    rd_force_points.append(dartModel.getBodyPositionGlobal(fi.targetBody))

        contactPositions = None
        # dartModel.update(motion_ori[frame])
        pdController.setTartgetPose(th_r)

        # bodyIDs = [body.index_in_skeleton for body in dartModel.world.collision_result.contacted_bodies]

        if not isCma:
            # change foot Kd and Kp
            for dofs in LeftFootDofs:
                pdController.setKpKd(dofs, getParamVal('LeftFootKp'), getParamVal('LeftFootKd'))

            for dofs in RightFootDofs:
                pdController.setKpKd(dofs, getParamVal('RightFootKp'), getParamVal('RightFootKd'))

            for dofs in footDofs:
                pdController.setKpKd(dofs, 500., 20.)
        elif True:
            # change foot Kd and Kp
            for dofs in LeftFootDofs:
                pdController.setKpKd(dofs, 300., 30.)

            for dofs in RightFootDofs:
                pdController.setKpKd(dofs, 300., 30.)

            for dofs in footDofs:
                pdController.setKpKd(dofs, 500., 20.)

        else:
            # change foot Kd and Kp
            for dofs in LeftFootDofs:
                pdController.setKpKd(dofs, 80., 10.)

            for dofs in RightFootDofs:
                pdController.setKpKd(dofs, 80., 10.)

        simulContactForces = np.zeros(3)
        cForcesControl = []
        cPointsControl = []

        if frame > 40:
            for i in range(stepsPerFrame):
                # bodyIDs, contactPositions, contactPositionLocals, contactForces = dartModel.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
                bodyIDs = dartModel.skeleton.self_collision_check()

                _tau = np.zeros(dartModel.skeleton.q.shape)
                # bodyIDs, contactPositions, contactPositionLocals, contactForces, timeStamp = \
                #     hdls.calcLCPForces(motion_ori, dartModel.world, dartModel, bodyIDsToCheck, 1., _tau)
                # dartModel.applyPenaltyForce(bodyIDs, contactPositions, contactForces, localForce=False)
                # print('penalty force sum: ', sum(contactForce for contactForce in contactForces))

                _ddq = pdController.compute()
                controlTau = None
                if False and SEGMENT_FOOT:
                    _ddq = pdController.compute()
                    _ddq0 = _ddq[specifiedDofIdx]
                    temp1, cPointsControl, temp3, cForcesControl, controlTau = hdls.calcLCPbasicControl(
                        motion_ori, dartModel.world, dartModel, bodyIDsToCheck, mu, np.array([0., 300., 0.]), [1., 1., 1.],
                        tau0=_ddq, variableDofIdx=footDofs)
                    print('controlTau: ', controlTau)
                # dartModel.skeleton.set_accelerations(_ddq)

                dartModel.skeleton.set_forces(pdController.compute())
                # dartModel.skeleton.set_forces(pdController.compute()+balanceTorque)
                dartModel.step()
                sumForce = sum([(-contact.force if contact.bodynode1.name == 'ground' else contact.force)
                                for contact in dartModel.world.collision_result.contacts])
                simulContactForces += sumForce
                '''
                if False and i % 5 == 0:
                    # bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
                    bodyIDs, contactPositions, contactPositionLocals, contactForces, timeStamp \
                        = hls.calcLCPForcesHD(motion_ori, vpWorld, dartModel, bodyIDsToCheck, 1., ddth_des_flat, ddth_des_flat, solver='qp', hdAccMask=hdAccMask)

                    if contactForces is not None:
                        lContactNum = sum([sum([j==i for j in bodyIDs]) for i in lIDs])
                        rContactNum = sum([sum([j==i for j in bodyIDs]) for i in rIDs])
                        if 1 <= lContactNum <= 2:
                            lbodyIDbs = [any([j==i for i in lIDs])for j in bodyIDs]
                            lbodyIDs = [i for i, x in enumerate(lbodyIDbs) if x]
                            for i in reversed(lbodyIDs):
                                bodyIDs.pop(i)
                                contactPositions.pop(i)
                                contactPositionLocals.pop(i)
                                contactForces.pop(i)

                        if 1 <= rContactNum <= 2:
                            rbodyIDbs = [any([j==i for i in rIDs])for j in bodyIDs]
                            rbodyIDs = [i for i, x in enumerate(rbodyIDbs) if x]
                            for i in reversed(rbodyIDs):
                                bodyIDs.pop(i)
                                contactPositions.pop(i)
                                contactPositionLocals.pop(i)
                                contactForces.pop(i)

                    if contactForces is not None:
                        vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)

                # print contactForces

                # apply external force
                for fi in forceInfos:
                    if fi.startFrame <= frame and frame < fi.startFrame + fi.duration*(1/frameTime):
                        # controlModel.applyBodyForceGlobal(fi.targetBody, fi.force)
                        dartModel.getBody(fi.targetBody).add_ext_force(fi.force)

                # for i in rIDs+lIDs:
                #     controlModel.setJointTorqueLocal(i, ddth_des[i])
                # controlModel.setDOFAccelerations(ddth_des)
                # controlModel.solveHybridDynamics()

                # if TORQUE_PLOT:
                #     rhip_torques[frame] += mm.length(controlModel.getJointTorqueLocal(rUpLeg))
                #     rknee_torques[frame] += mm.length(controlModel.getJointTorqueLocal(rKnee))
                #     rankle_torques[frame] += mm.length(controlModel.getJointTorqueLocal(rFoot))

                # rd_torques[:] = [controlModel.getJointTorqueLocal(j)/100. for j in range(1, skeleton.getJointNum())]
                # rd_joint_positions[:] = controlModel.getJointPositionsGlobal()

                # vpWorld.step()
                #            yvu.align2D(controlModel)
                '''


                '''
                if contactForces is not None and len(contactForces) > 0:
                    CP += yrp.getCP(contactPositions, contactForces)
                    F += sum(contactForces)
                avg_dCM[0] += dartModel.getJointVelocityGlobal(0)
                '''
        #            avg_dCM[0] += yrp.getCM(controlModel.getJointVelocitiesGlobal(), bodyMasses, upperMass, uppers)
        #            avg_dCM[0] += yrp.getCM(controlModel.getJointVelocitiesGlobal(), bodyMasses, totalMass)

        #            if len(stanceFoots)>0:
        #                avg_stf_v[0] += controlModel.getJointVelocityGlobal(stanceFoots[0])
        #                avg_stf_av[0] += controlModel.getJointAngVelocityGlobal(stanceFoots[0])


        bodyIDs, contactPositions, contactPositionLocals, velocities = dartModel.getContactPoints(bodyIDsToCheck)

        contactPoints = [contact.point for contact in dartModel.world.collision_result.contacts]
        contactForces = [(-contact.force if contact.bodynode1.name == 'ground' else contact.force)
                         for contact in dartModel.world.collision_result.contacts]

        sumForce = sum(contactForces)

        if not isCma:
            # graph calculated force
            viewer.cForceWnd.insertData('realForce', frame, simulContactForces[1]/stepsPerFrame)

        if not isCma:
            del rd_cForces[:]
            del rd_cPositions[:]
            for i in range(len(contactPoints)):
                rd_cForces.append(contactForces[i] / 50.)
                rd_cPositions.append(contactPoints[i])

            del rd_cForcesControl[:]
            del rd_cPositionsControl[:]
            for i in range(len(cForcesControl)):
                rd_cForces.append(cForcesControl[i] / 50.)
                rd_cPositions.append(cPointsControl[i])

        # bodyIDs = [body.index_in_skeleton() for body in contacted_bodies]
        # contacted_bodies = dartModel.world.collision_result.contacted_bodies # type: list[pydart.BodyNode]
        # bodyIDs = []
        # for body in contacted_bodies:
        #     ground_skeleton = body.skeleton # type: pydart.Skeleton
        #     if ground_skeleton.name == "grount skeleton":
        #         print("hehe")

        # print "COM: ", dartModel.getCOM()
        # print "dq: ", dartModel.skeleton.dq

        if not isCma:
            del rd_point2[:]
            if contactPositions is not None:
                rd_point2.extend(contactPositions)

        if not isCma:
            del rd_point1[:]
            rd_point1.append(dartModel.getCOM())


        CP /= stepsPerFrame
        F /= stepsPerFrame
        avg_dCM[0] /= stepsPerFrame

        #        if len(stanceFoots)>0:
        #            avg_stf_v[0] /= stepsPerFrame
        #            avg_stf_av[0] /= stepsPerFrame
        #            rd_vec1[0] = avg_stf_av[0]; rd_vec1[0][0] = 0.; rd_vec1[0][2] = 0.
        #            rd_vecori1[0]= controlModel.getJointPositionGlobal(stanceFoots[0])

        #=======================================================================
        # segment editing
        #=======================================================================
        lastFrame = False

        # print curState
        # print bodyIDs

        if SEGMENT_EDITING:
            if curState==yba.GaitState.STOP:
                if frame == len(motion_seg)-1:
                    lastFrame = True

            elif (curState==yba.GaitState.LSWING or curState==yba.GaitState.RSWING) and t>c_min_contact_time:
                contact = False

                if not SEGMENT_FOOT:
                    # original box foot
                    swingID = lID if curState==yba.GaitState.LSWING else rID

                    if swingID in bodyIDs:
                        minContactVel = 1000.
                        for i in range(len(bodyIDs)):
                            if bodyIDs[i]==swingID:
                                vel = dartModel.getBodyVelocityGlobal(swingID, contactPositionLocals[i])
                                vel[1] = 0
                                contactVel = mm.length(vel)
                                if contactVel < minContactVel: minContactVel = contactVel
                        if minContactVel < c_min_contact_vel: contact = True

                else:
                    # segmented foot
                    swingIDs = copy.deepcopy(lIDs) if curState==yba.GaitState.LSWING else copy.deepcopy(rIDs)

                    contact = False

                    for swingID in swingIDs:
                        if swingID in bodyIDs:
                            minContactVel = 1000.
                            for i in range(len(bodyIDs)):
                                if bodyIDs[i]==swingID:
                                    vel = dartModel.getBodyVelocityGlobal(swingID, contactPositionLocals[i])
                                    vel[1] = 0
                                    contactVel = mm.length(vel)
                                    if contactVel < minContactVel: minContactVel = contactVel
                            if minContactVel < c_min_contact_vel: contact = True

                extended[0] = False

                if contact:
                    #                    print frame, 'foot touch'
                    lastFrame = True
                    acc_offset[0] += frame - curInterval[1]

                elif frame == len(motion_seg)-1:
                    if not isCma:
                        print(frame, 'extend frame', frame+1)

                    preserveJoints = []
                    #                    preserveJoints = [lFoot, rFoot]
                    #                    preserveJoints = [lFoot, rFoot, lKnee, rKnee]
                    #                    preserveJoints = [lFoot, rFoot, lKnee, rKnee, lUpLeg, rUpLeg]
                    stanceKnees = [rKnee] if curState==yba.GaitState.LSWING else [lKnee]
                    preserveJoints = [stanceFoots[0], stanceKnees[0], stanceLegs[0]]

                    diff = 3
                    motion_seg_orig.extend([motion_seg_orig[-1]])
                    motion_seg.extend(ymt.extendByIntegration_root(motion_seg, 1, diff))

                    motion_stitch.extend(ymt.extendByIntegration_constant(motion_stitch, 1, preserveJoints, diff))

                    #                    # extend for swing foot ground speed matching & swing foot height lower
                    ##                    extendedPostures = ymt.extendByIntegration(motion_stitch, 1, preserveJoints, diff)
                    ##                    extendedPostures = [motion_stitch[-1]]
                    ##
                    #                    extendFrameNum = frame - curInterval[1] + 1
                    #                    k = 1.-extendFrameNum/5.
                    #                    if k<0.: k=0.
                    #                    extendedPostures = ymt.extendByIntegrationAttenuation(motion_stitch, 1, preserveJoints, diff, k)
                    #
                    ##                    if len(swingFoots)>0 and np.inner(dCM_tar, dCM)>0.:
                    ##                        print frame, 'speed matching'
                    ##                        R_swf = motion_stitch[-1].getJointOrientationGlobal(swingFoots[0])
                    ##
                    ##                        p_swf = motion_stitch[-1].getJointPositionGlobal(swingFoots[0])
                    ##                        v_swf = motion_stitch.getJointVelocityGlobal(swingFoots[0], frame-diff, frame)
                    ##                        a_swf = motion_stitch.getJointAccelerationGlobal(swingFoots[0], frame-diff, frame)
                    ##                        p_swf += v_swf * (frameTime) + a_swf * (frameTime)*(frameTime)
                    ##                        aik.ik_analytic(extendedPostures[0], swingFoots[0], p_swf)
                    ##
                    ##                        extendedPostures[0].setJointOrientationGlobal(swingFoots[0], R_swf)
                    #
                    #                    motion_stitch.extend(extendedPostures)

                    extended[0] = True
        else:
            if frame == len(motion_seg)-1: lastFrame = True

        if lastFrame:
            if segIndex < len(segments)-1:
                if not isCma:
                    print('%d (%d): end of %dth seg (%s, %s)'%(frame, frame-curInterval[1],segIndex, yba.GaitState.text[curState], curInterval))
                if plot is not None:
                    plot.addDataPoint('diff', frame, (frame-curInterval[1])*.01)

                if len(stanceFoots)>0 and len(swingFoots)>0:
                    #                    step_cur = controlModel.getJointPositionGlobal(swingFoots[0]) - controlModel.getJointPositionGlobal(stanceFoots[0])
                    #                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(swingFoots[0]) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                    step_cur = dartModel.getJointPositionGlobal(0) - dartModel.getJointPositionGlobal(stanceFoots[0])
                    step_tar = motion_seg[curInterval[1]].getJointPositionGlobal(0) - motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])

                    step_cur = mm.projectionOnPlane(step_cur, (1,0,0), (0,0,1))
                    step_tar = mm.projectionOnPlane(step_tar, (1,0,0), (0,0,1))

                    step_cur_sag, step_cur_cor = mm.projectionOnVector2(step_cur, direction)
                    step_tar_sag, step_tar_cor = mm.projectionOnVector2(step_tar, direction)

                    step_length_tar[0] = mm.length(step_tar_sag)
                    if np.inner(step_tar_sag, step_cur_sag) > 0:
                        step_length_cur[0] = mm.length(step_cur_sag)
                    else:
                        step_length_cur[0] = -mm.length(step_cur_sag)

                    step_axis[0] = directionAxis

                #                    rd_vec1[0] = step_tar_sag
                #                    rd_vecori1[0] = motion_seg[curInterval[1]].getJointPositionGlobal(stanceFoots[0])
                #                    rd_vec2[0] = step_cur_sag
                #                    rd_vecori2[0] = controlModel.getJointPositionGlobal(stanceFoots[0])

                seg_index[0] += 1
                curSeg = segments[seg_index[0]]
                stl_y_limit_num[0] = 0
                stl_xz_limit_num[0] = 0

                del motion_seg_orig[frame+1:]
                motion_seg_orig.extend(ymb.getAttachedNextMotion(curSeg, motion_seg_orig[-1], False, False))

                del motion_seg[frame+1:]
                del motion_stitch[frame+1:]
                transitionLength = len(curSeg)-1

                #                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, motion_seg[-1], False, False))
                #                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, motion_control[-1], transitionLength, stitch_func, True, False))

                d = motion_seg[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, d, True, False))

                if NO_FOOT_SLIDING:
                    if segIndex == len(segments)-2:
                        Rl = motion_control[-1].getJointOrientationLocal(lUpLeg)
                        Rr = motion_control[-1].getJointOrientationLocal(rUpLeg)
                        Rlk = motion_control[-1].getJointOrientationLocal(lKnee)
                        Rrk = motion_control[-1].getJointOrientationLocal(rKnee)
                        Rlf = motion_control[-1].getJointOrientationLocal(lFoot)
                        Rrf = motion_control[-1].getJointOrientationLocal(rFoot)
                        for p in curSeg:
                            p.setJointOrientationLocal(lUpLeg, Rl, False)
                            p.setJointOrientationLocal(rUpLeg, Rr, False)
                            p.setJointOrientationLocal(lKnee, Rlk, False)
                            p.setJointOrientationLocal(rKnee, Rrk, False)
                            p.setJointOrientationLocal(lFoot, Rlf, False)
                            p.setJointOrientationLocal(rFoot, Rrf, False)
                            p.updateGlobalT()

                d = motion_control[-1] - curSeg[0]
                d.rootPos[1] = 0.
                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, d, transitionLength, stitch_func, True, False))

            #                motion_seg.extend(ymb.getAttachedNextMotion(curSeg, motion_seg[-1], False, True))
            #                motion_stitch.extend(ymb.getStitchedNextMotion(curSeg, motion_control[-1], transitionLength, stitch_func, True, True))
            else:
                motion_seg_orig.append(motion_seg_orig[-1])
                motion_seg.append(motion_seg[-1])
                motion_stitch.append(motion_control[-1])


        # rendering
        # motionModel.update(motion_ori[frame])
        if not isCma:
            # dartMotionModel.update(motion_stitch[frame])
            dartMotionModel.update(motion_stf_balancing[frame])
        #        motionModel.update(motion_seg[frame])

        rd_CP[0] = CP
        # rd_CMP[0] = (CMreal[0] - (F[0]/F[1])*CMreal[1], 0, CMreal[2] - (F[2]/F[1])*CMreal[1])

        if plot is not None:
            plot.addDataPoint('zero', frame, 0)
            plot.updatePoints()

    if not isCma:
        viewer.setSimulateCallback(simulateCallback)

        if MULTI_VIEWER:
            viewer.startTimer(frameTime / 1.4)
        else:
            viewer.startTimer(frameTime * .1)
        viewer.show()

        Fl.run()
    else:
        objectiveSum = 0
        successSum = 0
        comSum = 0
        velSum = 0
        dirSum = 0

        for i in range(MAX_FRAME):
            simulateCallback(i)

            _com = dartModel.getCOM()

            if i > 50:
                successSum -= 1

                comSum += _com[2] * _com[2]

                _com_vel = dartModel.skeleton.com_velocity()
                _com_vel[1] = 0.
                velSum += (np.linalg.norm(_com_vel) - 0.7)*(np.linalg.norm(_com_vel)-0.7)

                dirDiff = mm.normalize(_com_vel) - np.array((-1., 0., 0.))
                dirSum += np.dot(dirDiff, dirDiff)

            if _com[1] < 0.65 or _com[1] > 1.0:
                break
            if i % 50 == 0 and (np.isnan(velSum) or np.isnan(dirSum)):
                break

        # objectiveSum = successSum + .3*comSum + velSum
        objectiveSum = successSum + velSum + .3*dirSum
        # print(objectiveSum, successSum, velSum, .3*dirSum, params)
        del motion_stitch[:]
        del motion_debug1[:]
        del motion_debug2[:]
        del motion_debug3[:]
        del motion_control[:]
        del motion_stf_balancing[:]
        del motion_match_stl[:]
        del motion_ori[:]
        del motion_seg[:]
        del motion_seg_orig[:]
        del motion_stf_push[:]
        del motion_stf_stabilize[:]
        del motion_swf_height[:]
        del motion_swf_placement[:]
        del motion_swf_orientation[:]
        return float(objectiveSum), float(successSum), float(velSum), float(.3*dirSum)
        # return float(objectiveSum)


if __name__ == '__main__':
    # c_min_contact_vel = 100.
    # c_min_contact_time = .7
    # c_landing_duration = .2
    # c_taking_duration = .3
    # c_swf_mid_offset = .0
    # c_locking_vel = .05
    # c_swf_offset = .01

    # K_stp_pos = 0.
    # c5 = .7
    # c6 = .02
    # K_stb_vel = .1
    # K_stb_pos = .1
    # K_swp_vel_sag = .0
    # K_swp_vel_cor = 1.3
    # K_swp_pos_sag = 1.2
    # K_swp_pos_cor = 1.
    # K_swp_pos_sag_faster = .05

    # viewer.objectInfoWnd.add1DSlider("c_min_contact_vel",   0., 200., .2, 100.)
    # viewer.objectInfoWnd.add1DSlider("c_min_contact_time",  0., 5., .01, .7)
    # viewer.objectInfoWnd.add1DSlider("c_landing_duration",  0., 5., .01, .2)
    # viewer.objectInfoWnd.add1DSlider("c_taking_duration",   0., 5., .01, .3)
    # viewer.objectInfoWnd.add1DSlider("c_swf_mid_offset",    -1., 1., .001, 0.)
    # viewer.objectInfoWnd.add1DSlider("c_locking_vel",       0., 1., .001, .05)
    # viewer.objectInfoWnd.add1DSlider("c_swf_offset",        -1., 1., .001, .01)

    # viewer.objectInfoWnd.add1DSlider("K_stp_pos",           0., 1., .01, 0.)
    # viewer.objectInfoWnd.add1DSlider("c5",                  0., 5., .01, .7)
    # viewer.objectInfoWnd.add1DSlider("c6",                  0., 1., .01, .02)
    # viewer.objectInfoWnd.add1DSlider("K_stb_vel",           0., 1., .01, .1)
    # viewer.objectInfoWnd.add1DSlider("K_stb_pos",           0., 1., .01, .1)
    # viewer.objectInfoWnd.add1DSlider("K_swp_vel_sag",       0., 5., .01, 0.)
    # viewer.objectInfoWnd.add1DSlider("K_swp_vel_cor",       0., 5., .01, 1.3)
    # viewer.objectInfoWnd.add1DSlider("K_swp_pos_sag",       0., 5., .01, 1.2)
    # viewer.objectInfoWnd.add1DSlider("K_swp_pos_cor",       0., 5., .01, 1.)
    # viewer.objectInfoWnd.add1DSlider("K_swp_pos_sag_faster",0., 1., .01, .05)


    # walkings(None, False)


    # hand tuning
    # params = [0., .7, .02, .1, .1, .0, 1.3, 1.2, 1., .05]
    # 325 frames success, Ks = 600.
    params = [ 0.01918975,  0.86622863,  0.15111008,  0.50972221,  0.09746768, -0.09129272,  1.12736657,  1.2873114 ,  0.84409227,  0.38928674]

    # 347 frames success, Ks = 600. ????????
    # params = [-0.0096717475861028673, 0.51455174209881782, 0.1414213562373095, 0.31622776601683794, 0.19555994814530026, 0.0, 1.1401754250991381, 1.457290633087426, 0.78654212710618387, 0.61027611069961429]

    # 287 frames success, Ks = 1000.
    # params = [-0.15744347,  0.67592998,  0.14142136,  0.31622777,  0.35696289, 0.,  1.14017543,  1.27637941,  0.95735647,  0.23835687]



    # 400 frames success, box foot, LCP, Kp = 200, Kd = 20
    # params = [-0.11523854,  0.56103475,  0.14142136,  0.31622777,  0.13175649,        0.        ,  1.14017543,  1.18703622,  0.77193057,  0.20490717]

    # infinite frames success, box foot, LCP, Kp = 200, Kd = 20, foot Kp = 80, foot Kd = 10
    params = [-0.13880733, 0.3439617, 0.14142136, 0.31622777, -0.18792631, 0., 1.14017543, 1.53473264, 1.07681499, 0.22992996]


    # 1220 frames success, parameter rounding, box foot, LCP,  Kp = 200, Kd = 20, foot Kp = 80, foot Kd = 10,
    params = [-0.11608721,  0.42672724,  0.14142136,  0.31622777, -0.12770363, 0.,  1.14017543,  1.63989139,  1.01964141,  0.18439344]

    # 1850 frames success, parameter rounding, box foot, LCP,  Kp = 200, Kd = 20, foot Kp = 80, foot Kd = 10,
    params = [-0.10540525,  0.40167391,  0.14142136,  0.31622777, -0.06906434, 0.,  1.14017543,  1.57445634,  1.01106981,  0.23834485]

    # infinite frames success, parameter rounding, box foot, LCP,  Kp = 200, Kd = 20, foot Kp = 80, foot Kd = 10,
    # params = [-0.03424024,  0.32955692,  0.0850351 ,  0.28576747, -0.10735104, 0.00185764,  1.36932697,  1.27616424,  0.97477866,  0.29608671]

    params = [ 0.23265769,  1.04283873, -0.29465862,  0.3544647, 0.2997252, -0.17338881, 2.08012922, 1.09571025, 0.6792339, -0.35920458]

    # DartTrackingFoot0 result, c_swf_mid_offset = 0.02
    params = [ 0.00745384, -0.56053261,  0.00921962,  0.42575388,  1.03165526, 0.69931117,  1.42782163,  1.65119398,  1.1237301 ,  0.5327249 ]

    params = [0., .7, .02, .1, .1, .0, 1.3, 1.2, 1., .05]
    params = [ 0.52572998,  0.15153905, -0.59859175,  0.93952107,  0.49886098, -0.1271257,  0.7328913,  0.87975694, 1.73943837, -0.97777014]

    # 120 frames success
    params = [-0.03373822, 0.21621505, -0.46121163, 0.97844009,  1.26921316,  0.07107696,  1.43362972,  0.10045292, 1.40123327, -0.67596869]

    # 195 frames success
    params = [-0.156885745146, 0.224351871531, -0.651388957459, 0.803834992348, 1.05714177435, 0.00542880291931, 1.56462249867, -0.111631227361, 1.37037255808, -1.00517210154]
    isCma = False

    if len(sys.argv) == 1 and not isCma:
        walkings(params, False)
    elif len(sys.argv) == 2 and sys.argv[1] == '-view' and not isCma:
        walkings(params, False)
    elif (len(sys.argv) == 2 and sys.argv[1] == '-cma') or isCma:
        # from PyCommon.modules.Math.Nomalizer import Normalizer
        # normalizer = Normalizer([0.]*10., [1., 5., .2, 1., 1., 3., 3., 3., 3., .5], [1.]*10, [-1.]*10)
        # c6, K_stb_vel, K_swp_vel_sag, K_swp_vel_cor is velocity gain
        # cmaOption = cma.CMAOptions('fixed_variables')
        # cmaOption.set('fixed_variables', {2:math.sqrt(.02), 3:math.sqrt(.1), 5:math.sqrt(0.), 6:math.sqrt(1.3)})
        # cma.fmin(walkings, np.sqrt([0., .5, .02, .1, .1, .0, 0.3, 1.2, .5, .05]).tolist(), .1, args=(True,), options=cmaOption)
        # cma.fmin(walkings, params, .1, args=(True,), options=cmaOption)
        # cma.fmin(walkings, params, .1, args=(True,))

        from datetime import datetime
        filename = datetime.now().strftime('%Y%m%d%H%M')+".opt"
        fout = open(filename, "w")
        fout.write(os.path.basename(__file__)+'\n')
        es = cma.CMAEvolutionStrategy(params, .1,
                                      {'maxiter':100})
        fout.close()
        # {'maxiter':2, 'fixed_variables':{2:math.sqrt(.02), 3:math.sqrt(.1), 5:math.sqrt(0.), 6:math.sqrt(1.3)}})
        pool = mp.Pool(es.popsize)
        cmaCount = 0
        while not es.stop():
            fout = open(filename, "a")
            X = es.ask()
            f_values = pool.map_async(walkings, X).get()
            obj_values = [f_value[0] for f_value in f_values]
            es.tell(X, obj_values)
            es.disp()
            es.logger.add()

            print(cmaCount, min(f_values), X[np.argmin(obj_values)])
            fout.write(str(cmaCount)+' '+str(min(f_values)))
            for x in X[np.argmin(obj_values)]:
                fout.write(' '+str(x)+',')
            fout.write('\n')
            cmaCount += 1
            fout.close()

        print("------------best-----------")
        print("eval: ", es.best.evals)
        print("f: ", es.best.f)
        print("x: ", es.best.x)
