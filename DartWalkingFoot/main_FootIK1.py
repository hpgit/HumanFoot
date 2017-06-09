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
        capsulRadius = SEGMENT_FOOT_MAG/2.

        # RightFoot_foot_0_0 : outside metatarsals
        capsulize('RightFoot_foot_0_0')
        node = mcfg.getNode('RightFoot_foot_0_0')
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-0.3, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])],
                     ypc.CapsuleMaterial(capsulDensity, capsulRadius, SEGMENT_FOOT_MAG*2.5 + 2.*capsulRadius))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-0.3-1.2, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])],
                     ypc.CapsuleMaterial(capsulDensity, capsulRadius, SEGMENT_FOOT_MAG*2.5 + 2.*capsulRadius))
        # node.addGeom('MyFoot4', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0., 0., 0.])], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        # RightFoot_foot_0_0_0 : outside phalanges
        capsulize('RightFoot_foot_0_0_0')
        node = mcfg.getNode('RightFoot_foot_0_0_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.jointType = footJointType

        # RightFoot_foot_0_1 : inside metatarsals
        capsulize('RightFoot_foot_0_1')
        node = mcfg.getNode('RightFoot_foot_0_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity,capsulRadius, -1))
        node.jointType = footJointType

        # RightFoot_foot_0_1_0 : inside phalanges
        capsulize('RightFoot_foot_0_1_0')
        node = mcfg.getNode('RightFoot_foot_0_1_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.jointType = footJointType

        # RightFoot_foot_1_0 : center heel
        capsulize('RightFoot_foot_1_0')
        node = mcfg.getNode('RightFoot_foot_1_0')
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0., 0., .7]), mm.exp([0.]*3)],
                     ypc.CapsuleMaterial(capsulDensity, capsulRadius, SEGMENT_FOOT_MAG*2. + capsulRadius * 2.))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        # RightFoot_foot_1_1 : inside heel
        capsulize('RightFoot_foot_1_1')
        node = mcfg.getNode('RightFoot_foot_1_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.jointType = footJointType

        # RightFoot_foot_1_2 : outside heel
        capsulize('RightFoot_foot_1_2')
        node = mcfg.getNode('RightFoot_foot_1_2')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.jointType = footJointType


        capsulize('LeftFoot_foot_0_0')
        node = mcfg.getNode('LeftFoot_foot_0_0')
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0.3, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])],
                     ypc.CapsuleMaterial(capsulDensity, capsulRadius, SEGMENT_FOOT_MAG*2.5+2.*capsulRadius))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0.3+1.2, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])],
                     ypc.CapsuleMaterial(capsulDensity, capsulRadius, SEGMENT_FOOT_MAG*2.5+2.*capsulRadius))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_0_0_0')
        node = mcfg.getNode('LeftFoot_foot_0_0_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_0_1')
        node = mcfg.getNode('LeftFoot_foot_0_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_0_1_0')
        node = mcfg.getNode('LeftFoot_foot_0_1_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.addGeom('MyFoot4', [SEGMENT_FOOT_MAG*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_1_0')
        node = mcfg.getNode('LeftFoot_foot_1_0')
        node.addGeom('MyFoot3', [SEGMENT_FOOT_MAG*np.array([0., 0., .7]), mm.exp([0.]*3)],
                     ypc.CapsuleMaterial(capsulDensity, capsulRadius, SEGMENT_FOOT_MAG*2.0+2.*capsulRadius))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_1_1')
        node = mcfg.getNode('LeftFoot_foot_1_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_1_2')
        node = mcfg.getNode('LeftFoot_foot_1_2')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, capsulRadius, -1))
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
    filename = 'segfoot_wd2_WalkForwardNormal00.bvh'
    # filename = 'segfoot_wd2_WalkForwardNormal00_REPEATED.bvh'

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
    print motion_ori[0].skeleton
    LeftAnkleIdx = motion_ori[0].skeleton.getJointIndex('LeftFoot')
    oriAnkle = motion_ori[0].getJointOrientationGlobal(LeftAnkleIdx)
    # motion_ori[0].setJointOrientationGlobal(LeftAnkleIdx, mm.slerp(R_motion, R_character, match_stl_func(t)))
    motion_ori[0].setJointOrientationGlobal(LeftAnkleIdx, np.dot(mm.rotZ(math.pi/4.), oriAnkle))

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
    rd_point3 = [None]
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
            viewer.doc.addRenderer('controlModel', yr.DartModelRenderer(dartModel, (50, 200, 200)))
            viewer.doc.addRenderer('motionModel', yr.DartModelRenderer(dartMotionModel, (50, 50, 200)))

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
            # viewer.doc.addRenderer('motion_control', yr.JointMotionRenderer(motion_control, (255,0,0), yr.LINK_BONE))

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
            viewer.doc.addRenderer('rd_point3', yr.PointsRenderer(rd_point3, (255,255,0)))
        #        viewer.doc.addRenderer('rd_vec1', yr.VectorsRenderer(rd_vec1, rd_vecori1, (255,0,0)))
        #    viewer.doc.addRenderer('rd_vec2', yr.VectorsRenderer(rd_vec2, rd_vecori2, (0,255,0)))
        #    viewer.doc.addRenderer('rd_frame1', yr.FramesRenderer(rd_frame1, (0,200,200)))
        #    viewer.doc.addRenderer('rd_frame2', yr.FramesRenderer(rd_frame2, (200,200,0)))
        #    viewer.setMaxFrame(len(motion_ori)-1)



        leftFootOri = dartModel.getBody('LeftFoot').world_transform() # type: np.ndarray

        viewer.objectInfoWnd.add1DSlider("offset_tx", 0., 2., .001, dartModel.getBodyPositionGlobal(0)[0])
        viewer.objectInfoWnd.add1DSlider("offset_ty", 0., 2., .001, dartModel.getBodyPositionGlobal(0)[1])
        viewer.objectInfoWnd.add1DSlider("offset_tz", 0., 2., .001, dartModel.getBodyPositionGlobal(0)[2])
        # viewer.objectInfoWnd.add1DSlider("offset_rx", 0., 2., .001, dartModel.getBody('LeftFoot').world_transform())
        # viewer.objectInfoWnd.add1DSlider("offset_ry", 0., 2., .001, dartModel.getBodyPositionGlobal(0)[0])
        # viewer.objectInfoWnd.add1DSlider("offset_rz", 0., 2., .001, dartModel.getBodyPositionGlobal(0)[0])

        viewer.objectInfoWnd.add1DRoller("rotate_x")
        viewer.objectInfoWnd.add1DRoller("rotate_y")
        viewer.objectInfoWnd.add1DRoller("rotate_z")

        viewer.objectInfoWnd.add1DSlider("debug", 0., 2., .001, 1.)

        currentFootOri = [motion_ori[0].getJointOrientationGlobal(LeftAnkleIdx)]

        def offsetSliderHandler(slider, event):
            '''
            :type slider: Fl_Hor_Value_Slider | Fl_Roller
            :return: 
            '''
            if event == FL_RELEASE:
                if slider.label() == 'rotate_x':
                    currentFootOri[0] = np.dot(mm.rotX(slider.value() * 4.), currentFootOri[0])
                elif slider.label() == 'rotate_y':
                    currentFootOri[0] = np.dot(mm.rotY(slider.value() * 4.), currentFootOri[0])
                elif slider.label() == 'rotate_z':
                    currentFootOri[0] = np.dot(mm.rotZ(slider.value() * 4.), currentFootOri[0])
                slider.value(0.)

        viewer.objectInfoWnd.getValobject('rotate_x').set_handler(offsetSliderHandler)
        viewer.objectInfoWnd.getValobject('rotate_y').set_handler(offsetSliderHandler)
        viewer.objectInfoWnd.getValobject('rotate_z').set_handler(offsetSliderHandler)

        def offsetSliderCallback(slider):
            '''
            :type slider: Fl_Hor_Value_Slider | Fl_Roller
            :return: 
            '''

            # trigger part
            if slider.label() == 'offset_tx':
                _rootpos = dartModel.getBodyPositionGlobal(0)
                dartModel.translateByOffset(np.array((slider.value() - _rootpos[0], 0., 0.)))
                dartMotionModel.translateByOffset(np.array((slider.value() - _rootpos[0], 0., 0.)))
                motion_ori[0].translateByOffset(np.array((slider.value()- _rootpos[0], 0., 0.)))
            elif slider.label() == 'offset_ty':
                _rootpos = dartModel.getBodyPositionGlobal(0)
                dartModel.translateByOffset(np.array((0., slider.value() - _rootpos[1], 0.)))
                dartMotionModel.translateByOffset(np.array((0., slider.value() - _rootpos[1], 0.)))
                motion_ori[0].translateByOffset(np.array((0., slider.value()- _rootpos[1], 0.)))
            elif slider.label() == 'offset_tz':
                _rootpos = dartModel.getBodyPositionGlobal(0)
                dartModel.translateByOffset(np.array((0., 0., slider.value() - _rootpos[2])))
                dartMotionModel.translateByOffset(np.array((0., 0., slider.value() - _rootpos[2])))
                motion_ori[0].translateByOffset(np.array((0., 0., slider.value()- _rootpos[2])))

            elif slider.label() == 'rotate_x':
                # footOri = motion_ori[0].getJointOrientationGlobal(LeftAnkleIdx)
                motion_ori[0].setJointOrientationGlobal(LeftAnkleIdx, np.dot(mm.rotX(slider.value() * 4.), currentFootOri[0]))
            elif slider.label() == 'rotate_y':
                # footOri = motion_ori[0].getJointOrientationGlobal(LeftAnkleIdx)
                motion_ori[0].setJointOrientationGlobal(LeftAnkleIdx, np.dot(mm.rotY(slider.value() * 4.), currentFootOri[0]))
            elif slider.label() == 'rotate_z':
                # footOri = motion_ori[0].getJointOrientationGlobal(LeftAnkleIdx)
                motion_ori[0].setJointOrientationGlobal(LeftAnkleIdx, np.dot(mm.rotZ(slider.value() * 4.), currentFootOri[0]))

            for idx in lIDs:
                motion_ori[0].setJointOrientationLocal(idx, np.eye(3))

            def getJointChildPositionGlobal(posture, jointNameOrIdx):
                """

                :type posture: ym.JointPosture
                :type jointNameOrIdx: str | int
                :return: np.array
                """
                idx = jointNameOrIdx
                if type(jointNameOrIdx) == str:
                    idx = posture.skeleton.getJointIndex(jointNameOrIdx)
                effectorOffset = posture.skeleton.getJoint(idx).children[0].offset
                return posture.getJointPositionGlobal(idx) + np.dot(posture.getJointOrientationGlobal(idx), effectorOffset)

            def makeTwoContactPos(posture, jointNameOrIdx, isLeftFoot=True, isOutside=True, baseHeight=None):
                """

                :type posture: ym.JointPosture
                :type jointNameOrIdx: str | int
                :return: np.array, np.array
                """
                idx = jointNameOrIdx
                if type(jointNameOrIdx) == str:
                    idx = posture.skeleton.getJointIndex(jointNameOrIdx)

                insideOffset = np.array((0., 0., SEGMENT_FOOT_MAG * 2.5))
                outsideOffset = np.array((1.2, 0., SEGMENT_FOOT_MAG * 2.5))
                if not isOutside:
                    # if it is not outside phalange,
                    outsideOffset[0] = -1.2

                origin = posture.getJointPositionGlobal(idx)
                inside = posture.getJointPositionGlobal(idx, insideOffset)
                outside = posture.getJointPositionGlobal(idx, outsideOffset)

                length = SEGMENT_FOOT_MAG * 2.5
                radius = SEGMENT_FOOT_MAG * .5

                RotVec1_tmp1 = inside - origin
                RotVec1_tmp2 = inside - origin
                RotVec1_tmp2[1] = 0.
                RotVec1 = np.cross(RotVec1_tmp1, RotVec1_tmp2)
                inner = (origin[1] - radius)/length

                angle1_1 = math.acos(inner if inner < 1.0 else 1.0)
                if baseHeight is not None:
                    angle1_1 = math.acos((origin[1] - (baseHeight + radius))/length)
                angle1_2 = math.acos((origin[1] - inside[1])/length)
                footRot1 = mm.exp(RotVec1, angle1_1-angle1_2)
                footOri1 = posture.getJointOrientationGlobal(idx)
                posture.setJointOrientationGlobal(idx, np.dot(footRot1, footOri1))

                inside_new = posture.getJointPositionGlobal(idx, insideOffset)
                outside_new_tmp = posture.getJointPositionGlobal(idx, outsideOffset)

                # RotVec2 = inside_new - origin
                width = np.linalg.norm(outside - inside)
                widthVec_tmp = np.cross(RotVec1_tmp1, np.array((0., 1., 0.))) if isLeftFoot ^ isOutside \
                    else np.cross(np.array((0., 1., 0.)), RotVec1_tmp1)

                widthVec = width * widthVec_tmp / np.linalg.norm(widthVec_tmp)
                outside_new = inside_new + widthVec

                footRot2 = mm.getSO3FromVectors(outside_new_tmp - inside_new, widthVec)
                footOri2 = posture.getJointOrientationGlobal(idx)
                # print footRot2, footOri2
                posture.setJointOrientationGlobal(idx, np.dot(footRot2, footOri2))

                return inside_new, outside_new

            def makeFourContactPos(posture, jointNameOrIdx, isLeftFoot=True, isOutside=True):
                """

                :type posture: ym.JointPosture
                :type jointNameOrIdx: str | int
                :return: np.array, np.array
                """
                idx = jointNameOrIdx
                if type(jointNameOrIdx) == str:
                    idx = posture.skeleton.getJointIndex(jointNameOrIdx)

                insideOffset = np.array((0., 0., SEGMENT_FOOT_MAG * 2.5))
                outsideOffset = np.array((1.2, 0., SEGMENT_FOOT_MAG * 2.5))
                if isLeftFoot ^ isOutside:
                    # if it is not outside phalange,
                    outsideOffset[0] = -1.2

                origin = posture.getJointPositionGlobal(idx)
                inside = posture.getJointPositionGlobal(idx, insideOffset)
                outside = posture.getJointPositionGlobal(idx, outsideOffset)

                length = SEGMENT_FOOT_MAG * 2.5
                radius = SEGMENT_FOOT_MAG * .5

                RotVec1_tmp1 = inside - origin
                RotVec1_tmp2 = inside - origin
                RotVec1_tmp2[1] = 0.
                RotVec1 = np.cross(RotVec1_tmp1, RotVec1_tmp2)
                angle1_1 = math.acos((origin[1] - radius)/length)
                angle1_2 = math.acos((origin[1] - inside[1])/length)
                footRot1 = mm.exp(RotVec1, angle1_1-angle1_2)
                footOri1 = posture.getJointOrientationGlobal(idx)
                posture.setJointOrientationGlobal(idx, np.dot(footRot1, footOri1))

                inside_new = posture.getJointPositionGlobal(idx, insideOffset)
                outside_new_tmp = posture.getJointPositionGlobal(idx, outsideOffset)

                # RotVec2 = inside_new - origin
                width = np.linalg.norm(outside - inside)
                widthVec_tmp = np.cross(RotVec1_tmp1, np.array((0., 1., 0.))) if isLeftFoot ^ isOutside \
                    else np.cross(np.array((0., 1., 0.)), RotVec1_tmp1)

                widthVec = width * widthVec_tmp / np.linalg.norm(widthVec_tmp)
                outside_new = inside_new + widthVec

                footRot2 = mm.getSO3FromVectors(outside_new_tmp - inside_new, widthVec)
                footOri2 = posture.getJointOrientationGlobal(idx)
                # print footRot2, footOri2
                posture.setJointOrientationGlobal(idx, np.dot(footRot2, footOri2))
                return

            collide={'LeftFoot_foot_0_0_0_Effector':False}
            collide['LeftFoot_foot_0_0_0'] = False
            collide['LeftFoot_foot_0_0'] = False

            if getJointChildPositionGlobal(motion_ori[0], 'LeftFoot_foot_0_0_0')[1] < SEGMENT_FOOT_MAG/2.:
                collide['LeftFoot_foot_0_0_0_Effector'] = True
            if getJointChildPositionGlobal(motion_ori[0], 'LeftFoot_foot_0_1_0')[1] < SEGMENT_FOOT_MAG/2.:
                collide['LeftFoot_foot_0_1_0_Effector'] = True
            if motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'])[1] < SEGMENT_FOOT_MAG/2.:
                collide['LeftFoot_foot_0_0_0'] = True
            if motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_1_0'])[1] < SEGMENT_FOOT_MAG/2.:
                collide['LeftFoot_foot_0_1_0'] = True
            if motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0'])[1] < SEGMENT_FOOT_MAG/2.:
                collide['LeftFoot_foot_0_0'] = True
            if motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_1'])[1] < SEGMENT_FOOT_MAG/2.:
                collide['LeftFoot_foot_0_1'] = True


            if collide['LeftFoot_foot_0_0_0_Effector'] and collide['LeftFoot_foot_0_0_0'] and collide['LeftFoot_foot_0_0']:

                footPoint0 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0'], np.array((0., 0., -1.)))
                footPoint1 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0'], np.array((0., 0., 1.)))
                footPoint2 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0'], np.array((1., 0., 1.)))

                footVec = np.cross(footPoint1 - footPoint0, footPoint2 - footPoint0)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = motion_ori[0].skeleton.getJointIndex('LeftFoot_foot_0_0')
                footOri = motion_ori[0].getJointOrientationGlobal(footIdx)
                # motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footOri, footRot))
                motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

            elif collide['LeftFoot_foot_0_0_0_Effector'] and collide['LeftFoot_foot_0_0_0']:
                _inside, _outside = makeTwoContactPos(motion_ori[0], 'LeftFoot_foot_0_0')
                # makeFourContactPos(motion_ori[0], 'LeftFoot_foot_0_0_0')
                footPoint0 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((0., 0., -1.)))
                footPoint1 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((0., 0., 1.)))
                footPoint2 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((1., 0., 1.)))

                footVec = np.cross(footPoint1 - footPoint0, footPoint2 - footPoint0)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = motion_ori[0].skeleton.getJointIndex('LeftFoot_foot_0_0_0')
                footOri = motion_ori[0].getJointOrientationGlobal(footIdx)
                # motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footOri, footRot))
                motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

                inside_tmp = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'])
                outside_tmp = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((1., 0., 0.)))
                footRot2 = mm.getSO3FromVectors(outside_tmp - inside_tmp, _outside - _inside)
                motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footRot2, np.dot(footRot, footOri)))

            elif collide['LeftFoot_foot_0_0_0_Effector']:
                footPoint = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'])

                _inside, _outside = makeTwoContactPos(motion_ori[0], 'LeftFoot_foot_0_0', baseHeight=footPoint[1]-SEGMENT_FOOT_MAG * .5)
                footPoint0 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((0., 0., -1.)))
                footPoint1 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((0., 0., 1.)))
                footPoint2 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((1., 0., 1.)))

                footVec = np.cross(footPoint1 - footPoint0, footPoint2 - footPoint0)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = motion_ori[0].skeleton.getJointIndex('LeftFoot_foot_0_0_0')
                footOri = motion_ori[0].getJointOrientationGlobal(footIdx)
                # motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footOri, footRot))
                motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

                inside_tmp = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'])
                outside_tmp = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((1., 0., 0.)))
                footRot2 = mm.getSO3FromVectors(outside_tmp - inside_tmp, _outside - _inside)
                motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footRot2, np.dot(footRot, footOri)))





            '''
            # contact detecting
            # if dartMotionModel.getBody('LeftFoot_foot_0_0_0').to_world()[1] < 0.:
            # if motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'])[1] < SEGMENT_FOOT_MAG/2.:
            if getJointChildPositionGlobal(motion_ori[0], 'LeftFoot_foot_0_0')[1] < SEGMENT_FOOT_MAG/2.:
                footPoint0 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0'], np.array((0., 0., -1.)))
                footPoint1 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0'], np.array((0., 0., 1.)))
                footPoint2 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0'], np.array((1., 0., 1.)))

                footVec = np.cross(footPoint1 - footPoint0, footPoint2 - footPoint0)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = motion_ori[0].skeleton.getJointIndex('LeftFoot_foot_0_0')
                footOri = motion_ori[0].getJointOrientationGlobal(footIdx)
                # motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footOri, footRot))
                motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

            if getJointChildPositionGlobal(motion_ori[0], 'LeftFoot_foot_0_0_0')[1] < SEGMENT_FOOT_MAG/2.:
                footPoint0 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((0., 0., -1.)))
                footPoint1 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((0., 0., 1.)))
                footPoint2 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_0_0'], np.array((1., 0., 1.)))

                footVec = np.cross(footPoint1 - footPoint0, footPoint2 - footPoint0)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = motion_ori[0].skeleton.getJointIndex('LeftFoot_foot_0_0_0')
                footOri = motion_ori[0].getJointOrientationGlobal(footIdx)
                # motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footOri, footRot))
                motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

            # if dartMotionModel.getBody('LeftFoot_foot_0_1_0').to_world()[1] < 0.:
            # if motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_1_0'])[1] < SEGMENT_FOOT_MAG/2.:
            if getJointChildPositionGlobal(motion_ori[0], 'LeftFoot_foot_0_1')[1] < SEGMENT_FOOT_MAG/2.:
                footPoint0 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_1'], np.array((0., 0., -1.)))
                footPoint1 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_1'], np.array((0., 0., 1.)))
                footPoint2 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_1'], np.array((1., 0., 1.)))

                footVec = np.cross(footPoint1 - footPoint0, footPoint2 - footPoint0)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = motion_ori[0].skeleton.getJointIndex('LeftFoot_foot_0_1')
                footOri = motion_ori[0].getJointOrientationGlobal(footIdx)
                motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))

            if getJointChildPositionGlobal(motion_ori[0], 'LeftFoot_foot_0_1_0')[1] < SEGMENT_FOOT_MAG/2.:
                footPoint0 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_1_0'], np.array((0., 0., -1.)))
                footPoint1 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_1_0'], np.array((0., 0., 1.)))
                footPoint2 = motion_ori[0].getJointPositionGlobal(lIDdic['LeftFoot_foot_0_1_0'], np.array((1., 0., 1.)))

                footVec = np.cross(footPoint1 - footPoint0, footPoint2 - footPoint0)
                footRot = mm.getSO3FromVectors(footVec, np.array((0., 1., 0.)))
                footIdx = motion_ori[0].skeleton.getJointIndex('LeftFoot_foot_0_1_0')
                footOri = motion_ori[0].getJointOrientationGlobal(footIdx)
                motion_ori[0].setJointOrientationGlobal(footIdx, np.dot(footRot, footOri))
            '''

            dartMotionModel.update(motion_ori[0])

            # solver.clear()
            # solver.addConstraints(dartModel.getBody('LeftFoot_foot_0_0_0').index_in_skeleton(),np.array((0., 0., 0.)),
            #                       np.array((1.79, 0.02, 0.17)), np.array(((1., 0., 0.),(0., 1., 0.),(0., 0., 1.))),
            #                       (False, True, False, False))
            # solver.solve(np.array((0., 0., 0.)))

            viewer.motionViewWnd.glWindow.redraw()

        viewer.objectInfoWnd.getValobject("offset_tx").callback(offsetSliderCallback)
        viewer.objectInfoWnd.getValobject("offset_ty").callback(offsetSliderCallback)
        viewer.objectInfoWnd.getValobject("offset_tz").callback(offsetSliderCallback)

        viewer.objectInfoWnd.getValobject("rotate_x").callback(offsetSliderCallback)
        viewer.objectInfoWnd.getValobject("rotate_y").callback(offsetSliderCallback)
        viewer.objectInfoWnd.getValobject("rotate_z").callback(offsetSliderCallback)

        viewer.objectInfoWnd.getValobject("debug").callback(offsetSliderCallback)


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

        def viewer_onClose(data):
            if plot is not None:
                plot.close()
            viewer.onClose(data)
        viewer.callback(viewer_onClose)


    if not isCma:
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
