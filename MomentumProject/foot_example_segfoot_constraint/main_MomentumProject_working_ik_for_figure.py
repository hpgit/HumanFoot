from fltk import *
import copy
import numpy as np
import math

import sys
if '../..' not in sys.path:
    sys.path.append('../..')
from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.Resource import ysMotionLoader as yf
from PyCommon.modules.Renderer import ysRenderer as yr
from PyCommon.modules.Simulator import csVpWorld as cvw
from PyCommon.modules.Simulator import csVpModel as cvm
# from PyCommon.modules.GUI import ysSimpleViewer as ysv
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Optimization import ysAnalyticConstrainedOpt as yac
from PyCommon.modules.Util import ysPythonEx as ype
from PyCommon.modules.ArticulatedBody import ysReferencePoints as yrp
from PyCommon.modules.ArticulatedBody import ysMomentum as ymt
from PyCommon.modules.ArticulatedBody import ysControl as yct
from PyCommon.modules.ArticulatedBody import hpInvKineDart as hik

from MomentumProject.foot_example_segfoot_constraint import mtOptimize as mot
from MomentumProject.foot_example_segfoot_constraint import mtInitialize as mit
from MomentumProject.foot_example_segfoot_constraint.foot_window import FootWindow

from PyCommon.modules.ArticulatedBody import hpFootIK as hfi
# from scipy.spatial import Delaunay

# import pydart2 as pydart
# from PyCommon.modules.Simulator import csDartModel as cdm
# from OpenGL.GL import *
# from OpenGL.GLUT import *


g_initFlag = 0
forceShowTime = 0

JsysPre = 0
JsupPreL = 0
JsupPreR = 0
JconstPre = 0

contactChangeCount = 0
contactChangeType = 0
contact = 0
maxContactChangeCount = 30

preFootCenter = [None]

DART_CONTACT_ON = False
SKELETON_ON = True


def main():
    # np.set_printoptions(precision=4, linewidth=200)
    np.set_printoptions(precision=5, threshold=np.inf, suppress=True, linewidth=3000)

    motionFile = 'wd2_tiptoe.bvh'
    motionFile = 'wd2_tiptoe_zygote.bvh'
    # motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = mit.create_biped(motionFile, SEGMENT_FOOT_RAD=0.008)
    motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = mit.create_biped(motionFile, SEGMENT_FOOT_MAG=0.01, SEGMENT_FOOT_RAD=0.008)
    # motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = mit.create_biped()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_jump_biped()

    vpWorld = cvw.VpWorld(wcfg)
    vpWorld.SetGlobalDamping(0.999)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    # controlModel_shadow_for_ik = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    controlModel.initializeHybridDynamics()

    # controlToMotionOffset = (1.5, -0.02, 0)
    controlToMotionOffset = (1.5, 0., 0)
    controlModel.translateByOffset(controlToMotionOffset)
    # controlModel_shadow_for_ik.set_q(controlModel.get_q())
    # controlModel_shadow_for_ik.computeJacobian(0, np.array([0., 0., 0.]))

    wcfg_ik = copy.deepcopy(wcfg)
    vpWorld_ik = cvw.VpWorld(wcfg_ik)
    controlModel_ik = cvm.VpControlModel(vpWorld_ik, motion[0], mcfg)
    vpWorld_ik.initialize()
    controlModel_ik.set_q(np.zeros_like(controlModel.get_q()))

    controlModel_q = np.zeros_like(controlModel.get_q())
    controlModel_q[4] = controlModel_q[4]+1.2 - 0.24
    controlModel.set_q(controlModel_q)
    print(controlModel.getBodyPositionGlobal(motion[0].skeleton.getJointIndex('RightFoot_foot_1_0')))



    totalDOF = controlModel.getTotalDOF()
    DOFs = controlModel.getDOFs()

    foot_dofs = []
    left_foot_dofs = []
    right_foot_dofs = []

    foot_seg_dofs = []
    left_foot_seg_dofs = []
    right_foot_seg_dofs = []

    # for joint_idx in range(motion[0].skeleton.getJointNum()):
    for joint_idx in range(controlModel.getJointNum()):
        joint_name = controlModel.index2name(joint_idx)
        # joint_name = motion[0].skeleton.getJointName(joint_idx)
        if 'Foot' in joint_name:
            foot_dofs_temp = controlModel.getJointDOFIndexes(joint_idx)
            foot_dofs.extend(foot_dofs_temp)
            if 'Left' in joint_name:
                left_foot_dofs.extend(foot_dofs_temp)
            elif 'Right' in joint_name:
                right_foot_dofs.extend(foot_dofs_temp)

        if 'foot' in joint_name:
            foot_dofs_temp = controlModel.getJointDOFIndexes(joint_idx)
            foot_seg_dofs.extend(foot_dofs_temp)
            if 'Left' in joint_name:
                left_foot_seg_dofs.extend(foot_dofs_temp)
            elif 'Right' in joint_name:
                right_foot_seg_dofs.extend(foot_dofs_temp)

    # parameter
    Kt = config['Kt']; Dt = config['Dt']  # tracking gain
    Kl = config['Kl']; Dl = config['Dl']  # linear balance gain
    Kh = config['Kh']; Dh = config['Dh']  # angular balance gain
    Ks = config['Ks']; Ds = config['Ds']  # penalty force spring gain

    Bt = config['Bt']
    Bl = config['Bl']
    Bh = config['Bh']

    selectedBody = motion[0].skeleton.getJointIndex(config['end'])
    constBody = motion[0].skeleton.getJointIndex('RightFoot')

    supL = motion[0].skeleton.getJointIndex('LeftFoot')
    supR = motion[0].skeleton.getJointIndex('RightFoot')

    # momentum matrix
    linkMasses = controlModel.getBodyMasses()
    totalMass = controlModel.getTotalMass()
    TO = ymt.make_TO(linkMasses)
    dTO = ymt.make_dTO(len(linkMasses))

    # optimization
    problem = yac.LSE(totalDOF, 12)
    # a_sup = (0,0,0, 0,0,0) #ori
    # a_sup = (0,0,0, 0,0,0) #L
    CP_old = [mm.v3(0., 0., 0.)]

    # penalty method
    bodyIDsToCheck = list(range(vpWorld.getBodyNum()))
    # mus = [1.]*len(bodyIDsToCheck)
    mus = [.5]*len(bodyIDsToCheck)

    # flat data structure
    ddth_des_flat = ype.makeFlatList(totalDOF)
    dth_flat = ype.makeFlatList(totalDOF)
    ddth_sol = ype.makeNestedList(DOFs)

    # viewer
    rd_footCenter = [None]
    rd_footCenter_ref = [None]
    rd_footCenterL = [None]
    rd_footCenterR = [None]
    rd_CM_plane = [None]
    rd_CM = [None]
    rd_CP = [None]
    rd_CP_des = [None]
    rd_dL_des_plane = [None]
    rd_dH_des = [None]
    rd_grf_des = [None]

    rd_exf_des = [None]
    rd_exfen_des = [None]
    rd_root_des = [None]

    rd_foot_ori = [None]
    rd_foot_pos = [None]

    rd_root_ori = [None]
    rd_root_pos = [None]

    rd_CF = [None]
    rd_CF_pos = [None]

    rootPos = [None]
    selectedBodyId = [selectedBody]
    extraForce = [None]
    extraForcePos = [None]

    rightFootVectorX = [None]
    rightFootVectorY = [None]
    rightFootVectorZ = [None]
    rightFootPos = [None]

    rightVectorX = [None]
    rightVectorY = [None]
    rightVectorZ = [None]
    rightPos = [None]

    def makeEmptyBasicSkeletonTransformDict(init=None):
        Ts = dict()
        Ts['pelvis'] = init
        Ts['spine_ribs'] = init
        Ts['head'] = init
        Ts['thigh_R'] = init
        Ts['shin_R'] = init
        Ts['foot_heel_R'] = init
        Ts['foot_R'] = init
        Ts['heel_R'] = init
        Ts['outside_metatarsal_R'] = init
        Ts['outside_phalanges_R'] = init
        Ts['inside_metatarsal_R'] = init
        Ts['inside_phalanges_R'] = init
        Ts['upper_limb_R'] = init
        Ts['lower_limb_R'] = init
        Ts['thigh_L'] = init
        Ts['shin_L'] = init
        Ts['foot_heel_L'] = init
        Ts['foot_L'] = init
        Ts['heel_L'] = init
        Ts['outside_metatarsal_L'] = init
        Ts['outside_phalanges_L'] = init
        Ts['inside_metatarsal_L'] = init
        Ts['inside_phalanges_L'] = init

        Ts['upper_limb_L'] = init
        Ts['lower_limb_L'] = init

        return Ts

    # viewer = ysv.SimpleViewer()
    # viewer = hsv.hpSimpleViewer(rect=[0, 0, 1024, 768], viewForceWnd=False)
    viewer = hsv.hpSimpleViewer(rect=[0, 0, 1920+300, 1+1080+55], viewForceWnd=False)
    # viewer.record(False)
    # viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('motionModel', yr.VpModelRenderer(motionModel, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.setRendererVisible('motionModel', False)
    viewer.doc.addRenderer('ikModel', yr.VpModelRenderer(controlModel_ik, (150,150,255), yr.POLYGON_LINE))
    viewer.doc.setRendererVisible('ikModel', False)
    # viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    control_model_renderer = yr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL)
    viewer.doc.addRenderer('controlModel', control_model_renderer)
    skeleton_renderer = None
    if SKELETON_ON:
        # skeleton_renderer = yr.BasicSkeletonRenderer(makeEmptyBasicSkeletonTransformDict(np.eye(4)), offset_Y=-0.08)
        # skeleton_renderer = yr.BasicSkeletonRenderer(makeEmptyBasicSkeletonTransformDict(np.eye(4)), color=(230, 230, 230), offset_draw=(0.8, -0.02, 0.))
        skeleton_renderer = yr.BasicSkeletonRenderer(makeEmptyBasicSkeletonTransformDict(np.eye(4)), color=(230, 230, 230), offset_draw=(0., -0.0, 0.))
        viewer.doc.addRenderer('skeleton', skeleton_renderer)
        viewer.doc.setRendererVisible('skeleton', False)
    viewer.doc.addRenderer('rd_footCenter', yr.PointsRenderer(rd_footCenter))
    viewer.doc.setRendererVisible('rd_footCenter', False)
    viewer.doc.addRenderer('rd_footCenter_ref', yr.PointsRenderer(rd_footCenter_ref))
    viewer.doc.setRendererVisible('rd_footCenter_ref', False)
    viewer.doc.addRenderer('rd_CM_plane', yr.PointsRenderer(rd_CM_plane, (255,255,0)))
    viewer.doc.setRendererVisible('rd_CM_plane', False)
    viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (0,255,0)))
    viewer.doc.setRendererVisible('rd_CP', False)
    viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (255,0,255)))
    viewer.doc.setRendererVisible('rd_CP_des', False)
    viewer.doc.addRenderer('rd_dL_des_plane', yr.VectorsRenderer(rd_dL_des_plane, rd_CM, (255,255,0)))
    viewer.doc.setRendererVisible('rd_dL_des_plane', False)
    viewer.doc.addRenderer('rd_dH_des', yr.VectorsRenderer(rd_dH_des, rd_CM, (0,255,0)))
    viewer.doc.setRendererVisible('rd_dH_des', False)
    # viewer.doc.addRenderer('rd_grf_des', yr.ForcesRenderer(rd_grf_des, rd_CP_des, (0,255,0), .001))
    viewer.doc.addRenderer('rd_CF', yr.VectorsRenderer(rd_CF, rd_CF_pos, (255,255,0)))
    viewer.doc.setRendererVisible('rd_CF', False)
    viewer.doc.addRenderer('rd_foot_ori', yr.OrientationsRenderer(rd_foot_ori, rd_foot_pos, (255,255,0)))
    viewer.doc.setRendererVisible('rd_foot_ori', False)

    viewer.doc.addRenderer('rd_root_ori', yr.OrientationsRenderer(rd_root_ori, rd_root_pos, (255,255,0)))
    viewer.doc.setRendererVisible('rd_root_ori', False)

    viewer.doc.addRenderer('extraForce', yr.VectorsRenderer(rd_exf_des, extraForcePos, (0,255,0)))
    viewer.doc.setRendererVisible('extraForce', False)
    viewer.doc.addRenderer('extraForceEnable', yr.VectorsRenderer(rd_exfen_des, extraForcePos, (255,0,0)))

    # viewer.doc.addRenderer('right_foot_oriX', yr.VectorsRenderer(rightFootVectorX, rightFootPos, (255,0,0)))
    # viewer.doc.addRenderer('right_foot_oriY', yr.VectorsRenderer(rightFootVectorY, rightFootPos, (0,255,0)))
    # viewer.doc.addRenderer('right_foot_oriZ', yr.VectorsRenderer(rightFootVectorZ, rightFootPos, (0,0,255)))

    # viewer.doc.addRenderer('right_oriX', yr.VectorsRenderer(rightVectorX, rightPos, (255,0,0)))
    # viewer.doc.addRenderer('right_oriY', yr.VectorsRenderer(rightVectorY, rightPos, (0,255,0)))
    # viewer.doc.addRenderer('right_oriZ', yr.VectorsRenderer(rightVectorZ, rightPos, (0,0,255)))

    # foot_viewer = FootWindow(viewer.x() + viewer.w() + 20, viewer.y(), 300, 400, 'foot contact modifier', controlModel)
    foot_viewer = None  # type: FootWindow

    initKt = 25.
    # initKt = 60.
    initKl = 100.
    initKh = 100.

    initBl = .1
    initBh = .13
    # initSupKt = 17
    initSupKt = 22

    initFm = 50.0

    initComX = 0.
    initComY = 0.
    initComZ = 0.

    viewer.objectInfoWnd.add1DSlider("Kt", 0., 300., 1., initKt)
    viewer.objectInfoWnd.add1DSlider("Kl", 0., 300., 1., initKl)
    viewer.objectInfoWnd.add1DSlider("Kh", 0., 300., 1., initKh)
    viewer.objectInfoWnd.add1DSlider("Bl", 0., 1., .001, initBl)
    viewer.objectInfoWnd.add1DSlider("Bh", 0., 1., .001, initBh)
    viewer.objectInfoWnd.add1DSlider("SupKt", 0., 300., 0.1, initSupKt)
    viewer.objectInfoWnd.add1DSlider("Fm", 0., 1000., 10., initFm)
    viewer.objectInfoWnd.add1DSlider("com X offset", -1., 1., 0.01, initComX)
    viewer.objectInfoWnd.add1DSlider("com Y offset", -1., 1., 0.01, initComY)
    viewer.objectInfoWnd.add1DSlider("com Z offset", -1., 1., 0.01, initComZ)
    viewer.objectInfoWnd.add1DSlider("tiptoe angle", -0.5, .5, 0.001, 0.)
    viewer.objectInfoWnd.add1DSlider("left tilt angle", -0.5, .5, 0.001, 0.)
    viewer.objectInfoWnd.add1DSlider("right tilt angle", -0.5, .5, 0.001, 0.)

    viewer.force_on = False

    def viewer_SetForceState(object):
        viewer.force_on = True

    def viewer_GetForceState():
        return viewer.force_on

    def viewer_ResetForceState():
        viewer.force_on = False

    viewer.objectInfoWnd.addBtn('Force on', viewer_SetForceState)
    viewer_ResetForceState()

    offset = 60

    viewer.objectInfoWnd.begin()
    viewer.objectInfoWnd.labelForceX = Fl_Value_Input(20, 30+offset*9, 40, 20, 'X')
    viewer.objectInfoWnd.labelForceX.value(0)

    viewer.objectInfoWnd.labelForceY = Fl_Value_Input(80, 30+offset*9, 40, 20, 'Y')
    viewer.objectInfoWnd.labelForceY.value(0)

    viewer.objectInfoWnd.labelForceZ = Fl_Value_Input(140, 30+offset*9, 40, 20, 'Z')
    viewer.objectInfoWnd.labelForceZ.value(1)

    viewer.objectInfoWnd.labelForceDur = Fl_Value_Input(220, 30+offset*9, 40, 20, 'Dur')
    viewer.objectInfoWnd.labelForceDur.value(0.1)

    viewer.objectInfoWnd.end()

    # self.sliderFm = Fl_Hor_Nice_Slider(10, 42+offset*6, 250, 10)

    def getParamVal(paramname):
        return viewer.objectInfoWnd.getVal(paramname)

    def getParamVals(paramnames):
        return (getParamVal(name) for name in paramnames)

    def setParamVal(paramname, val):
        viewer.objectInfoWnd.setVal(paramname, val)

    idDic = dict()
    for i in range(motion[0].skeleton.getJointNum()):
        idDic[motion[0].skeleton.getJointName(i)] = i

    # extendedFootName = ['Foot_foot_0_0', 'Foot_foot_0_1', 'Foot_foot_0_0_0', 'Foot_foot_0_1_0', 'Foot_foot_1_0']
    extendedFootName = ['Foot_foot_0_0', 'Foot_foot_0_0_0', 'Foot_foot_0_1_0', 'Foot_foot_1_0']
    lIDdic = {'Left'+name: motion[0].skeleton.getJointIndex('Left'+name) for name in extendedFootName}
    rIDdic = {'Right'+name: motion[0].skeleton.getJointIndex('Right'+name) for name in extendedFootName}
    footIdDic = lIDdic.copy()
    footIdDic.update(rIDdic)

    lIDlist = [motion[0].skeleton.getJointIndex('Left'+name) for name in extendedFootName]
    rIDlist = [motion[0].skeleton.getJointIndex('Right'+name) for name in extendedFootName]
    footIdlist = []
    footIdlist.extend(lIDlist)
    footIdlist.extend(rIDlist)

    foot_left_idx = motion[0].skeleton.getJointIndex('LeftFoot')
    foot_right_idx = motion[0].skeleton.getJointIndex('RightFoot')

    foot_left_idx_temp = motion[0].skeleton.getJointIndex('LeftFoot_foot_1_0')
    foot_right_idx_temp = motion[0].skeleton.getJointIndex('RightFoot_foot_1_0')

    # ik_solver = hik.numIkSolver(dartIkModel)
    # ik_solver.clear()

    # bodyIDsToCheck = rIDlist.copy()

    joint_names = [motion[0].skeleton.getJointName(i) for i in range(motion[0].skeleton.getJointNum())]

    def fix_dofs(_DOFs, nested_dof_values, _mcfg, _joint_names):
        fixed_nested_dof_values = list()
        fixed_nested_dof_values.append(nested_dof_values[0])
        for i in range(1, len(_DOFs)):
            dof = _DOFs[i]
            if dof == 1:
                node = _mcfg.getNode(_joint_names[i])
                axis = mm.unitZ()
                if node.jointAxes[0] == 'X':
                    axis = mm.unitX()
                elif node.jointAxes[0] == 'Y':
                    axis = mm.unitY()
                fixed_nested_dof_values.append(np.array([np.dot(nested_dof_values[i], axis)]))
            else:
                fixed_nested_dof_values.append(nested_dof_values[i])

        return fixed_nested_dof_values

    start_frame = 130

    up_vec_in_each_link = dict()
    for foot_id in footIdlist:
        up_vec_in_each_link[foot_id] = controlModel_ik.getBodyOrientationGlobal(foot_id)[1, :]
    controlModel_ik.set_q(controlModel.get_q())

    ###################################
    # simulate
    ###################################
    def preFrameCallback_Always(frame):
        if frame < 120+start_frame:
            viewer.doc.setRendererVisible('controlModel', True)
            viewer.doc.setRendererVisible('skeleton', False)
            viewer.motionViewWnd.glWindow.camera.rotateX = math.pi /180. * -25.
            if frame > start_frame:
                viewer.motionViewWnd.glWindow.camera.rotateY = mm.deg2Rad((frame-start_frame)*3+45.)
            else:
                viewer.motionViewWnd.glWindow.camera.rotateY = mm.deg2Rad(45.)

            viewer.motionViewWnd.glWindow.camera.distance = .4
            viewer.motionViewWnd.glWindow.camera.center = \
                .5*(controlModel.getBodyPositionGlobal(idDic['RightFoot']) + controlModel.getBodyPositionGlobal(idDic['LeftFoot'])) + np.array([0., -0.05, 0.])
        elif frame > 340:
            viewer.doc.setRendererVisible('controlModel', False)
            viewer.doc.setRendererVisible('skeleton', True)
        else:
            if 0 <= frame % 50 < 22:
                viewer.doc.setRendererVisible('controlModel', True)
                viewer.doc.setRendererVisible('skeleton', False)
            elif 22 <= frame % 50 < 25:
                viewer.doc.setRendererVisible('controlModel', False)
                viewer.doc.setRendererVisible('skeleton', False)
            elif 25 <= frame % 50 < 47:
                viewer.doc.setRendererVisible('controlModel', False)
                viewer.doc.setRendererVisible('skeleton', True)
            elif 47 <= frame % 50 < 50:
                viewer.doc.setRendererVisible('controlModel', False)
                viewer.doc.setRendererVisible('skeleton', False)

    def simulateCallback(frame):
        # print(frame)
        # print(motion[frame].getJointOrientationLocal(footIdDic['RightFoot_foot_0_1_0']))
        if False:
            if frame == 200:
                if motionFile == 'wd2_tiptoe.bvh':
                    setParamVal('tiptoe angle', 0.3)
                if motionFile == 'wd2_tiptoe_zygote.bvh':
                    setParamVal('tiptoe angle', 0.3)
            # elif 210 < frame < 240:
                # if motionFile == 'wd2_tiptoe_zygote.bvh':
                #     setParamVal('com Y offset', 0.01/30. * (frame-110))
            elif frame == 400:
                setParamVal('com Y offset', 0.)
                setParamVal('tiptoe angle', 0.)
            elif frame == 430:
                foot_viewer.check_all_seg()
                # setParamVal('SupKt', 30.)
            # elif frame == 400:
            #     setParamVal('SupKt', 17.)


        # hfi.footAdjust(motion[frame], idDic, SEGMENT_FOOT_MAG=.03, SEGMENT_FOOT_RAD=.015, baseHeight=0.02)

        if abs(getParamVal('tiptoe angle')) > 0.001:
            tiptoe_angle = getParamVal('tiptoe angle')
            motion[frame].mulJointOrientationLocal(idDic['LeftFoot_foot_0_0_0'],
                                                   mm.exp(mm.unitX(), -math.pi * tiptoe_angle))
            motion[frame].mulJointOrientationLocal(idDic['LeftFoot_foot_0_1_0'],
                                                   mm.exp(mm.unitX(), -math.pi * tiptoe_angle))
            motion[frame].mulJointOrientationLocal(idDic['RightFoot_foot_0_0_0'],
                                                   mm.exp(mm.unitX(), -math.pi * tiptoe_angle))
            motion[frame].mulJointOrientationLocal(idDic['RightFoot_foot_0_1_0'],
                                                   mm.exp(mm.unitX(), -math.pi * tiptoe_angle))
            # motion[frame].mulJointOrientationLocal(idDic['LeftFoot'], mm.exp(mm.unitX(), math.pi * tiptoe_angle * 0.95))
            # motion[frame].mulJointOrientationLocal(idDic['RightFoot'], mm.exp(mm.unitX(), math.pi * tiptoe_angle * 0.95))
            motion[frame].mulJointOrientationLocal(idDic['LeftFoot'], mm.exp(mm.unitX(), math.pi * tiptoe_angle))
            motion[frame].mulJointOrientationLocal(idDic['RightFoot'], mm.exp(mm.unitX(), math.pi * tiptoe_angle))

        if getParamVal('left tilt angle') > 0.001:
            left_tilt_angle = getParamVal('left tilt angle')
            if motion[0].skeleton.getJointIndex('LeftFoot_foot_0_1') is not None:
                motion[frame].mulJointOrientationLocal(idDic['LeftFoot_foot_0_1'], mm.exp(mm.unitZ(), -math.pi * left_tilt_angle))
            else:
                motion[frame].mulJointOrientationLocal(idDic['LeftFoot_foot_0_1_0'], mm.exp(mm.unitZ(), -math.pi * left_tilt_angle))
            motion[frame].mulJointOrientationLocal(idDic['LeftFoot'], mm.exp(mm.unitZ(), math.pi * left_tilt_angle))

        elif getParamVal('left tilt angle') < -0.001:
            left_tilt_angle = getParamVal('left tilt angle')
            motion[frame].mulJointOrientationLocal(idDic['LeftFoot_foot_0_0'], mm.exp(mm.unitZ(), -math.pi * left_tilt_angle))
            if motion[0].skeleton.getJointIndex('LeftFoot_foot_0_1') is not None:
                motion[frame].mulJointOrientationLocal(idDic['LeftFoot_foot_0_1'], mm.exp(mm.unitZ(), math.pi * left_tilt_angle))
            else:
                motion[frame].mulJointOrientationLocal(idDic['LeftFoot_foot_0_1_0'], mm.exp(mm.unitZ(), math.pi * left_tilt_angle))
            motion[frame].mulJointOrientationLocal(idDic['LeftFoot'], mm.exp(mm.unitZ(), math.pi * left_tilt_angle))

        if getParamVal('right tilt angle') > 0.001:
            right_tilt_angle = getParamVal('right tilt angle')
            if motion[0].skeleton.getJointIndex('RightFoot_foot_0_1') is not None:
                motion[frame].mulJointOrientationLocal(idDic['RightFoot_foot_0_1'], mm.exp(mm.unitZ(), math.pi * right_tilt_angle))
            else:
                motion[frame].mulJointOrientationLocal(idDic['RightFoot_foot_0_1_0'], mm.exp(mm.unitZ(), math.pi * right_tilt_angle))
            motion[frame].mulJointOrientationLocal(idDic['RightFoot'], mm.exp(mm.unitZ(), -math.pi * right_tilt_angle))
        elif getParamVal('right tilt angle') < -0.001:
            right_tilt_angle = getParamVal('right tilt angle')
            motion[frame].mulJointOrientationLocal(idDic['RightFoot_foot_0_0'], mm.exp(mm.unitZ(), math.pi * right_tilt_angle))
            if motion[0].skeleton.getJointIndex('RightFoot_foot_0_1') is not None:
                motion[frame].mulJointOrientationLocal(idDic['RightFoot_foot_0_1'], mm.exp(mm.unitZ(), -math.pi * right_tilt_angle))
            # else:
            #     motion[frame].mulJointOrientationLocal(idDic['RightFoot_foot_0_1_0'], mm.exp(mm.unitZ(), -math.pi * right_tilt_angle))
            motion[frame].mulJointOrientationLocal(idDic['RightFoot'], mm.exp(mm.unitZ(), -math.pi * right_tilt_angle))

        motionModel.update(motion[frame])
        motionModel.translateByOffset(np.array([getParamVal('com X offset'), getParamVal('com Y offset'), getParamVal('com Z offset')]))
        controlModel_ik.set_q(controlModel.get_q())

        global g_initFlag
        global forceShowTime

        global JsysPre
        global JsupPreL
        global JsupPreR

        global JconstPre

        global preFootCenter
        global maxContactChangeCount
        global contactChangeCount
        global contact
        global contactChangeType

        Kt, Kl, Kh, Bl, Bh, kt_sup = getParamVals(['Kt', 'Kl', 'Kh', 'Bl', 'Bh', 'SupKt'])
        Dt = 2*(Kt**.5)
        Dl = 2*(Kl**.5)
        Dh = 2*(Kh**.5)
        dt_sup = 2*(kt_sup**.5)

        # tracking
        th_r = motion.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)

        # ype.flatten(fix_dofs(DOFs, ddth_des, mcfg, joint_names), ddth_des_flat)
        # ype.flatten(fix_dofs(DOFs, dth, mcfg, joint_names), dth_flat)
        ype.flatten(ddth_des, ddth_des_flat)
        ype.flatten(dth, dth_flat)

        #################################################
        # jacobian
        #################################################

        contact_des_ids = list()  # desired contact segments
        if foot_viewer.check_om_l.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('LeftFoot_foot_0_0'))
        if foot_viewer.check_op_l.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('LeftFoot_foot_0_0_0'))
        if foot_viewer.check_im_l is not None and foot_viewer.check_im_l.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('LeftFoot_foot_0_1'))
        if foot_viewer.check_ip_l.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('LeftFoot_foot_0_1_0'))
        if foot_viewer.check_h_l.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('LeftFoot_foot_1_0'))

        if foot_viewer.check_om_r.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('RightFoot_foot_0_0'))
        if foot_viewer.check_op_r.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('RightFoot_foot_0_0_0'))
        if foot_viewer.check_im_r is not None and foot_viewer.check_im_r.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('RightFoot_foot_0_1'))
        if foot_viewer.check_ip_r.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('RightFoot_foot_0_1_0'))
        if foot_viewer.check_h_r.value():
            contact_des_ids.append(motion[0].skeleton.getJointIndex('RightFoot_foot_1_0'))

        contact_ids = list()  # temp idx for balancing
        contact_ids.extend(contact_des_ids)

        contact_joint_ori = list(map(controlModel.getJointOrientationGlobal, contact_ids))
        contact_joint_pos = list(map(controlModel.getJointPositionGlobal, contact_ids))
        contact_body_ori = list(map(controlModel.getBodyOrientationGlobal, contact_ids))
        contact_body_pos = list(map(controlModel.getBodyPositionGlobal, contact_ids))
        contact_body_vel = list(map(controlModel.getBodyVelocityGlobal, contact_ids))
        contact_body_angvel = list(map(controlModel.getBodyAngVelocityGlobal, contact_ids))

        ref_joint_ori = list(map(motion[frame].getJointOrientationGlobal, contact_ids))
        ref_joint_pos = list(map(motion[frame].getJointPositionGlobal, contact_ids))
        ref_joint_vel = [motion.getJointVelocityGlobal(joint_idx, frame) for joint_idx in contact_ids]
        ref_joint_angvel = [motion.getJointAngVelocityGlobal(joint_idx, frame) for joint_idx in contact_ids]
        ref_body_ori = list(map(motionModel.getBodyOrientationGlobal, contact_ids))
        ref_body_pos = list(map(motionModel.getBodyPositionGlobal, contact_ids))
        # ref_body_vel = list(map(controlModel.getBodyVelocityGlobal, contact_ids))
        ref_body_angvel = [motion.getJointAngVelocityGlobal(joint_idx, frame) for joint_idx in contact_ids]
        ref_body_vel = [ref_joint_vel[i] + np.cross(ref_joint_angvel[i], ref_body_pos[i] - ref_joint_pos[i])
                        for i in range(len(ref_joint_vel))]

        is_contact = [1] * len(contact_ids)
        contact_right = len(set(contact_des_ids).intersection(rIDlist)) > 0
        contact_left = len(set(contact_des_ids).intersection(lIDlist)) > 0

        contMotionOffset = th[0][0] - th_r[0][0]

        linkPositions = controlModel.getBodyPositionsGlobal()
        linkVelocities = controlModel.getBodyVelocitiesGlobal()
        linkAngVelocities = controlModel.getBodyAngVelocitiesGlobal()
        linkInertias = controlModel.getBodyInertiasGlobal()

        CM = yrp.getCM(linkPositions, linkMasses, totalMass)
        dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)
        CM_plane = copy.copy(CM)
        CM_plane[1] = 0.
        dCM_plane = copy.copy(dCM)
        dCM_plane[1] = 0.

        P = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions, CM, linkInertias)
        dP = ymt.getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities, dCM, linkAngVelocities, linkInertias)

        if foot_viewer is not None:
            foot_viewer.foot_pressure_gl_window.refresh_foot_contact_info(frame, vpWorld, bodyIDsToCheck, mus, Ks, Ds)
            foot_viewer.foot_pressure_gl_window.goToFrame(frame)

        # rendering
        for foot_seg_id in footIdlist:
            control_model_renderer.body_colors[foot_seg_id] = (255, 240, 255)

        for contact_id in contact_ids:
            control_model_renderer.body_colors[contact_id] = (255, 0, 0)



        rd_CM[0] = CM

        rd_CM_plane[0] = CM.copy()
        rd_CM_plane[0][1] = 0.

        del rd_foot_ori[:]
        del rd_foot_pos[:]
        # for seg_foot_id in footIdlist:
        #     rd_foot_ori.append(controlModel.getJointOrientationGlobal(seg_foot_id))
        #     rd_foot_pos.append(controlModel.getJointPositionGlobal(seg_foot_id))
        rd_foot_ori.append(controlModel.getJointOrientationGlobal(supL))
        rd_foot_ori.append(controlModel.getJointOrientationGlobal(supR))
        rd_foot_pos.append(controlModel.getJointPositionGlobal(supL))
        rd_foot_pos.append(controlModel.getJointPositionGlobal(supR))

        rd_root_des[0] = rootPos[0]
        rd_root_ori[0] = controlModel.getBodyOrientationGlobal(0)
        rd_root_pos[0] = controlModel.getBodyPositionGlobal(0)

        del rd_CF[:]
        del rd_CF_pos[:]

        # render contact_ids

        # render skeleton
        if SKELETON_ON:
            Ts = dict()
            Ts['pelvis'] = controlModel.getJointTransform(idDic['Hips'])
            Ts['thigh_R'] = controlModel.getJointTransform(idDic['RightUpLeg'])
            Ts['shin_R'] = controlModel.getJointTransform(idDic['RightLeg'])
            Ts['foot_R'] = controlModel.getJointTransform(idDic['RightFoot'])
            Ts['foot_heel_R'] = controlModel.getJointTransform(idDic['RightFoot'])
            Ts['heel_R'] = np.eye(4)
            Ts['outside_metatarsal_R'] = controlModel.getJointTransform(idDic['RightFoot_foot_0_0'])
            Ts['outside_phalanges_R'] = controlModel.getJointTransform(idDic['RightFoot_foot_0_0_0'])
            # Ts['inside_metatarsal_R'] = controlModel.getJointTransform(idDic['RightFoot_foot_0_1'])
            Ts['inside_metatarsal_R'] = np.eye(4)
            Ts['inside_phalanges_R'] = controlModel.getJointTransform(idDic['RightFoot_foot_0_1_0'])
            Ts['spine_ribs'] = controlModel.getJointTransform(idDic['Spine'])
            Ts['head'] = controlModel.getJointTransform(idDic['Spine1'])
            Ts['upper_limb_R'] = controlModel.getJointTransform(idDic['RightArm'])
            Ts['lower_limb_R'] = controlModel.getJointTransform(idDic['RightForeArm'])
            Ts['thigh_L'] = controlModel.getJointTransform(idDic['LeftUpLeg'])
            Ts['shin_L'] = controlModel.getJointTransform(idDic['LeftLeg'])
            Ts['foot_L'] = controlModel.getJointTransform(idDic['LeftFoot'])
            Ts['foot_heel_L'] = controlModel.getJointTransform(idDic['LeftFoot'])
            Ts['heel_L'] = np.eye(4)
            Ts['outside_metatarsal_L'] = controlModel.getJointTransform(idDic['LeftFoot_foot_0_0'])
            Ts['outside_phalanges_L'] = controlModel.getJointTransform(idDic['LeftFoot_foot_0_0_0'])
            # Ts['inside_metatarsal_L'] = controlModel.getJointTransform(idDic['LeftFoot_foot_0_1'])
            Ts['inside_metatarsal_L'] = np.eye(4)
            Ts['inside_phalanges_L'] = controlModel.getJointTransform(idDic['LeftFoot_foot_0_1_0'])
            Ts['upper_limb_L'] = controlModel.getJointTransform(idDic['LeftArm'])
            Ts['lower_limb_L'] = controlModel.getJointTransform(idDic['LeftForeArm'])

            skeleton_renderer.appendFrameState(Ts)

    viewer.setSimulateCallback(simulateCallback)
    viewer.setPreFrameCallback_Always(preFrameCallback_Always)
    viewer.startTimer(1/30.)
    # viewer.play()
    viewer.show()

    foot_viewer = FootWindow(viewer.x() + viewer.w() + 20, viewer.y(), 300, 500, 'foot contact modifier', controlModel)
    foot_viewer.show()
    foot_viewer.check_op_l.value(True)
    foot_viewer.check_ip_l.value(True)
    foot_viewer.check_op_r.value(True)
    foot_viewer.check_ip_r.value(True)
    foot_viewer.check_not_all_seg()
    viewer.motionViewWnd.goToFrame(0)

    Fl.run()


main()
