from fltk import *
import copy
import numpy as np
import numpy.linalg as npl
import time

import sys
# if '../PyCommon/modules' not in sys.path:
    # sys.path.append('../PyCommon/modules')
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

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import Motion.import_trc as trc

import mtOptimize as mot
import mtInitialize_Simple as mit

MOTION_COLOR = (213, 111, 162)
CHARACTER_COLOR = (20, 166, 188)
FEATURE_COLOR = (255, 102, 0)
CHARACTER_COLOR2 = (200, 200, 200)


def getPartJacobian(_Jsys, _jIdx):
    # warning : only Jsys works.
    return _Jsys[6 * _jIdx:6 * _jIdx + 6].copy()


def getBodyGlobalPos(_model, _motion, _name):
    return _model.getBodyPositionGlobal(_motion[0].skeleton.getJointIndex(_name))


def getBodyGlobalOri(_model, _motion, _name):
    return _model.getBodyOrientationGlobal(_motion[0].skeleton.getJointIndex(_name))

motion = None
mcfg = None
wcfg = None
stepsPerFrame = None
config = None
mcfg_motion = None

rd_jointPos = None

rd_tibia = None
rd_backfoot = None
rd_midfoot = None
rd_forefoot = None

rd_tibia_tri = None
rd_backfoot_tri = None
rd_midfoot_tri = None
rd_forefoot_tri = None

viewer = None

infoNames = None
infoData = None
markerName = None
markerPosi = None

tibia_marker_idx = None
backfoot_marker_idx = None
midfoot_marker_idx = None
forefoot_marker_idx = None

rd_tibia_cog = None
rd_backfoot_cog = None
rd_midfoot_cog = None

def init():
    global motion
    global mcfg
    global wcfg
    global stepsPerFrame
    global config
    global mcfg_motion
    global viewer

    global rd_jointPos
    global rd_tibia
    global rd_forefoot
    global rd_backfoot
    global rd_midfoot

    global infoNames
    global infoData
    global markerName
    global markerPosi

    global tibia_marker_idx
    global backfoot_marker_idx
    global forefoot_marker_idx
    global midfoot_marker_idx

    global rd_tibia_cog
    global rd_backfoot_cog
    global rd_midfoot_cog

    global rd_tibia_tri
    global rd_backfoot_tri
    global rd_midfoot_tri
    global rd_forefoot_tri

    np.set_printoptions(precision=4, linewidth=200)
    motion, mcfg, wcfg, stepsPerFrame, config = mit.create_biped_basic('../2_Test01_Dynamic.bvh')
    mcfg_motion = mit.normal_mcfg()

    infoNames, infoData, markerName, markerPosi = trc.import_trc('../trc/2_test01_dynamic.trc')
    # infoNames, infoData, markerName, markerPosi = trc.import_trc('../trc/2_test02_dynamic.trc')
    # infoNames, infoData, markerName, markerPosi = trc.import_trc('../trc/P_1_test01_dynamic.trc')
    # infoNames, infoData, markerName, markerPosi = trc.import_trc('../trc/P_2_test01_dynamic.trc')
    # infoNames, infoData, markerName, markerPosi = trc.import_trc('../trc/P_3_test01_dynamic.trc')

    info = dict(zip(infoNames, infoData))
    numFrame = int(info['NumFrames'])-1

    motion[:] = [motion[0]]*numFrame

    rd_jointPos = [None]

    rd_tibia = [None]
    rd_backfoot = [None]
    rd_midfoot = [None]
    rd_forefoot = [None]

    rd_tibia_tri = [None]
    rd_backfoot_tri = [None]
    rd_midfoot_tri = [None]
    rd_forefoot_tri = [None]

    rd_tibia_cog = [None]
    rd_backfoot_cog = [None]
    rd_midfoot_cog = [None]

    viewer = hsv.hpSimpleViewer([100, 100, 1280, 960])
    viewer.doc.addObject('motion', motion)
    # viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion))
    viewer.doc.addRenderer('rd_jointPos', yr.PointsRenderer(rd_jointPos, color=(255,0,0), pointStyle=1))

    viewer.doc.addRenderer('rd_tibia', yr.PointsRenderer(rd_tibia, color=(255,0,255), pointStyle=1))
    viewer.doc.addRenderer('rd_tibia_tri', yr.LinesRenderer(rd_tibia_tri, color=(255,0,255)))

    viewer.doc.addRenderer('rd_backfoot', yr.PointsRenderer(rd_backfoot, color=(0,0,255), pointStyle=1))
    viewer.doc.addRenderer('rd_backfoot_tri', yr.LinesRenderer(rd_backfoot_tri, color=(0,0,255)))

    viewer.doc.addRenderer('rd_midfoot', yr.PointsRenderer(rd_midfoot, color=(0,255,0), pointStyle=1))
    viewer.doc.addRenderer('rd_midfoot_tri', yr.LinesRenderer(rd_midfoot_tri, color=(0,255,0)))

    viewer.doc.addRenderer('rd_forefoot', yr.PointsRenderer(rd_forefoot, color=(0,0,0), pointStyle=1))
    viewer.doc.addRenderer('rd_forefoot_tri', yr.LinesRenderer(rd_forefoot_tri, color=(0,0,0)))

    # viewer.doc.addRenderer('rd_tibia_cog', yr.PointsRenderer(rd_tibia_cog, color=(255,0,255), pointStyle=0))
    # viewer.doc.addRenderer('rd_backfoot_cog', yr.PointsRenderer(rd_backfoot_cog, color=(0,0,255), pointStyle=0))
    # viewer.doc.addRenderer('rd_midfoot_cog', yr.PointsRenderer(rd_midfoot_cog, color=(0,255,0), pointStyle=0))
    # viewer.doc.addRenderer('rd_tibia_cog', yr.LinesRenderer(rd_tibia_cog, color=(255,0,255)))
    # viewer.doc.addRenderer('rd_backfoot_cog', yr.LinesRenderer(rd_backfoot_cog, color=(0,0,255)))
    # viewer.doc.addRenderer('rd_midfoot_cog', yr.LinesRenderer(rd_midfoot_cog, color=(0,255,0)))


    viewer.objectInfoWnd.add1DSlider(
        'PD gain', minVal=0., maxVal=1000., initVal=180., valStep=.1)
    viewer.objectInfoWnd.addBtn('image', viewer.motionViewWnd.dump)
    viewer.objectInfoWnd.addBtn('movie', viewer.motionViewWnd.dumpMov)

    # viewer.cForceWnd.addDataSet('midfoot_height', FL_BLACK)
    # viewer.cForceWnd.addDataSet('midfoot_top_angle', FL_GREEN)
    # viewer.cForceWnd.addDataSet('midfoot_front_angle', FL_RED)

    print info
    print markerName

    '''
    ['Top.Head', 'Front.Head', 'Rear.Head',
     'R.Shoulder', 'R.Offset', 'R.Elbow', 'R.Wrist',
     'L.Shoulder', 'L.Elbow', 'L.Wrist', 'R.ASIS', 'L.ASIS',
     'V.Sacral',
     'R.Thigh', 'R.Knee', 'R.Shank', 'R.Ankle', 'R.Heel', 'R.Toe',
     'L.Thigh', 'L.Knee', 'L.Shank', 'L.Ankle', 'L.Heel', 'L.Toe',
     'R1', 'R2', 'R3', 'R4', 'R5', 'R6', 'R7',
     'L1', 'L2', 'L3', 'L4', 'L5', 'L6', 'L7',
     'V_Mid_ASIS', 'V_Pelvis_Origin', 'V_R.Hip_JC', 'V_L.Hip_JC', 'V_R.Knee_JC', 'V_L.Knee_JC', 'V_R.Ankle_JC', 'V_L.Ankle_JC', 'V_Mid_Hip', 'V_Mid_Shoulder', 'V_R.Hand', 'V_L.Hand', 'V_R.Toe_Offset_Static', 'V_L.Toe_Offset_Static', 'V_R.Toe_Offset', 'V_L.Toe_Offset']
    '''
    '''
    ['LFWT', 'RFWT', 'VBWT',
    'LKNE', 'LANK', 'LMT5', 'LMT1', 'LHEL', 'LTOE',
    'RKNE', 'RANK', 'RMT5', 'RMT1', 'RHEL', 'RTOE',
    'T10', 'FHEAD', 'THEAD', 'RHEAD',
    'LBSH', 'LELB', 'LOWR', 'RBSH', 'RELB', 'ROWR',
    'LTHI', 'RTHI', 'LSHAN', 'RSHAN',
    'LIHEL', 'LOHEL', 'RIHEL', 'ROHEL', 'LMT3', 'LNAVI', 'LCUNE', 'RMT3', 'RNAVI', 'RCUNE']
    '''
    tibia_marker = ['RKNE', 'RSHAN', 'RANK', 'LKNE', 'LSHAN', 'LANK']
    forefoot_marker = ['RTOE', 'RMT1', 'RMT3', 'RMT5', 'LTOE', 'LMT1', 'LMT3', 'LMT5']
    midfoot_marker = ['RCUNE', 'RNAVI', 'LCUNE', 'LNAVI']
    backfoot_marker = ['RHEL', 'RIHEL', 'ROHEL', 'LHEL', 'LIHEL', 'LOHEL']
    tibia_marker_idx = [markerName.index(i) for i in tibia_marker]
    midfoot_marker_idx = [markerName.index(i) for i in midfoot_marker]
    forefoot_marker_idx = [markerName.index(i) for i in forefoot_marker]
    backfoot_marker_idx = [markerName.index(i) for i in backfoot_marker]

    print tibia_marker_idx
    print forefoot_marker_idx
    print midfoot_marker_idx
    print backfoot_marker_idx

    midfoot_height = []
    midfoot_top_angle = []
    midfoot_front_angle = []
    midfoot_length = []

    for frame in range(numFrame):
        tibia_points = [np.array(markerPosi[frame][i]) for i in tibia_marker_idx[:3]]
        backfoot_points = [np.array(markerPosi[frame][i]) for i in backfoot_marker_idx[:3]]
        midfoot_points = [np.array(markerPosi[frame][i]) for i in midfoot_marker_idx[:2]]
        forefoot_points = [np.array(markerPosi[frame][i]) for i in forefoot_marker_idx[:4]]
        center, basis = getCenterAndBasisFromPoints(backfoot_points)
        basis = [basis[2], basis[0], basis[1]]
        rotation = np.zeros((3, 3))
        for i in range(3):
            rotation[i, :] = basis[i]

        offset = np.array((0., 0., 0.))

        tibia_trans_points = [np.dot(rotation, point-center) + offset for point in tibia_points]
        backfoot_trans_points = [np.dot(rotation, point-center) + offset for point in backfoot_points]
        midfoot_trans_points = [np.dot(rotation, point-center) + offset for point in midfoot_points]
        forefoot_trans_points = [np.dot(rotation, point-center) + offset for point in forefoot_points]

        # viewer.cForceWnd.insertData('midfoot_height', frame, sum(midfoot_trans_points)[1]*1000/2.)
        midfoot_height.append(sum(midfoot_trans_points)[1]*1000./2.)

        mf_ang_vec = midfoot_trans_points[1] - midfoot_trans_points[0]
        mf_front_ang = math.asin(abs(mf_ang_vec[1])/npl.norm(mf_ang_vec)) * 180./math.pi
        mf_top_ang = math.atan2(mf_ang_vec[2], -mf_ang_vec[0]) * 180./math.pi

        midfoot_top_angle.append(mf_top_ang)
        midfoot_front_angle.append(mf_front_ang)

        midfoot_length.append(npl.norm(midfoot_trans_points[1]-midfoot_trans_points[0]))

        # viewer.cForceWnd.insertData('midfoot_top_angle', frame, mf_top_ang*50)
        # viewer.cForceWnd.insertData('midfoot_front_angle', frame, mf_front_ang)

    plt.figure(1)
    plt.subplot(311)
    # plt.ylim(-100, 100)
    plt.ylabel('height(mm)')
    plt.plot(midfoot_length)
    # plt.plot(midfoot_height)
    plt.subplot(312)
    plt.ylim(0, 30)
    plt.ylabel('front angle(deg)')
    plt.plot(midfoot_front_angle)
    plt.subplot(313)
    plt.ylim(40, 65)
    plt.ylabel('top angle(deg)')
    plt.plot(midfoot_top_angle)
    plt.savefig('hehe.pdf')


def unit(x):
    _x = np.array(x)
    return _x/np.linalg.norm(_x)


def getCenterAndBasisFromPoints(points):
    center = sum(points)/3.
    first_coord = unit(points[0] - points[2])
    temp_coord = points[2] - points[1]
    third_coord = unit(np.cross(first_coord, temp_coord))
    second_coord = unit(temp_coord - np.dot(temp_coord, first_coord)*first_coord)

    return center, [third_coord, first_coord, second_coord]

init()




class Callback:

    def __init__(self):
        self.prevTime = 0
        self.frame = 0

    def setTimeStamp(self):
        if self.timeIndex == 0:
            self.prevTime = time.time()
            self.timeIndex += 1
            return
        if len(self.timeStamp) < self.timeIndex:
            self.timeStamp.append(0.)
        curTime = time.time()
        self.timeStamp[self.timeIndex - 1] += curTime - self.prevTime
        self.prevTime = curTime
        self.timeIndex += 1

    def simulateCallback(self, frame):
        self.frame = frame
        # print "main:frame : ", frame

        del rd_jointPos[:]
        del rd_tibia[:]
        del rd_backfoot[:]
        del rd_midfoot[:]
        del rd_forefoot[:]
        rd_jointPos.extend(markerPosi[frame][:39])

        rd_tibia.extend([markerPosi[frame][i] for i in tibia_marker_idx])
        rd_backfoot.extend([markerPosi[frame][i] for i in backfoot_marker_idx])
        rd_midfoot.extend([markerPosi[frame][i] for i in midfoot_marker_idx])
        rd_forefoot.extend([markerPosi[frame][i] for i in forefoot_marker_idx])

        rd_tibia_cog.append(sum([markerPosi[frame][i] for i in tibia_marker_idx[:3]])/3)
        rd_backfoot_cog.append(sum([markerPosi[frame][i] for i in backfoot_marker_idx[:3]])/3)
        rd_midfoot_cog.append(sum([markerPosi[frame][i] for i in midfoot_marker_idx[:2]])/2)

        tibia_points = [np.array(markerPosi[frame][i]) for i in tibia_marker_idx[:3]]
        backfoot_points = [np.array(markerPosi[frame][i]) for i in backfoot_marker_idx[:3]]
        midfoot_points = [np.array(markerPosi[frame][i]) for i in midfoot_marker_idx[:2]]
        forefoot_points = [np.array(markerPosi[frame][i]) for i in forefoot_marker_idx[:4]]

        center, basis = getCenterAndBasisFromPoints(backfoot_points)
        basis = [basis[2], basis[0], basis[1]]
        rotation = np.zeros((3, 3))
        for i in range(3):
            rotation[i, :] = basis[i]

        offset = np.array((0., 0.10, 0.))

        del rd_tibia[:]
        tibia_trans_points = [np.dot(rotation, point-center) + offset for point in tibia_points]
        rd_tibia.extend(tibia_trans_points)
        del rd_backfoot[:]
        backfoot_trans_points = [np.dot(rotation, point-center) + offset for point in backfoot_points]
        rd_backfoot.extend(backfoot_trans_points)
        del rd_midfoot[:]
        midfoot_trans_points = [np.dot(rotation, point-center) + offset for point in midfoot_points]
        rd_midfoot.extend(midfoot_trans_points)
        del rd_forefoot[:]
        forefoot_trans_points = [np.dot(rotation, point-center) + offset for point in forefoot_points]
        rd_forefoot.extend(forefoot_trans_points)

        del rd_tibia_tri[:]
        rd_tibia_tri.extend(tibia_trans_points)
        rd_tibia_tri.append(tibia_trans_points[0])
        del rd_backfoot_tri[:]
        rd_backfoot_tri.extend(backfoot_trans_points)
        rd_backfoot_tri.append(backfoot_trans_points[0])
        del rd_midfoot_tri[:]
        rd_midfoot_tri.extend(midfoot_trans_points)
        rd_midfoot_tri.append(midfoot_trans_points[0])
        del rd_forefoot_tri[:]
        rd_forefoot_tri.extend(forefoot_trans_points)
        rd_forefoot_tri.append(forefoot_trans_points[0])


        mf_ang_vec = midfoot_trans_points[1] - midfoot_trans_points[0]
        mf_front_ang = math.asin(abs(mf_ang_vec[1])/npl.norm(mf_ang_vec))
        mf_top_ang = math.atan2(mf_ang_vec[2], mf_ang_vec[0])

        # viewer.cForceWnd.insertData('midfoot_height', frame, sum(midfoot_trans_points)[1]*1000/2.)
        # viewer.cForceWnd.insertData('midfoot_top_angle', frame, mf_top_ang*50)
        # viewer.cForceWnd.insertData('midfoot_front_angle', frame, mf_front_ang)



callback = Callback()

viewer.setSimulateCallback(callback.simulateCallback)

viewer.startTimer(1 / 60.)
viewer.show()

Fl.run()
