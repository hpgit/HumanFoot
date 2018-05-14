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
# from PyCommon.modules.Renderer import csVpRenderer as cvr
from PyCommon.modules.Simulator import csVpWorld as cvw
from PyCommon.modules.Simulator import csVpModel as cvm
# from PyCommon.modules.GUI import ysSimpleViewer as ysv
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Optimization import ysAnalyticConstrainedOpt as yac
from PyCommon.modules.ArticulatedBody import ysJacobian as yjc
from PyCommon.modules.Util import ysPythonEx as ype
from PyCommon.modules.ArticulatedBody import ysReferencePoints as yrp
from PyCommon.modules.ArticulatedBody import ysMomentum as ymt
from PyCommon.modules.ArticulatedBody import ysControl as yct
from PyCommon.modules.ArticulatedBody import hpInvKineDart as hik

from MomentumProject.foot_example_segfoot_constraint import mtOptimize as mot
from MomentumProject.foot_example_segfoot_constraint import mtInitialize as mit

# import mtOptimize as mot
# import mtInitialize as mit

# from PyCommon.modules.ArticulatedBody import hpFootIK as hfi
from scipy.spatial import Delaunay

# from PyCommon.modules import pydart2 as pydart
import pydart2 as pydart
from PyCommon.modules.Simulator import csDartModel as cdm

from OpenGL.GL import *
from OpenGL.GLUT import *


g_initFlag = 0
forceShowTime = 0

contactChangeCount = 0
contactChangeType = 0
contact = 0
maxContactChangeCount = 30

preFootCenter = [None]

DART_CONTACT_ON = False
SKELETON_ON = False


class PressureFrameInfo:
    def __init__(self):
        self.contact_seg_idx = []
        self.contact_seg_position_local = []
        self.contact_seg_forces = []


class FootPressureGlWindow(Fl_Gl_Window):
    def __init__(self, x, y, w, h, model):
        Fl_Gl_Window.__init__(self, x, y, w, h)
        self.initGL()
        self.rc = yr.RenderContext()
        self.model = model

        self.foot_index = []
        self.left_foot_index = []
        self.right_foot_index = []
        self.foot_seg_index = []
        self.left_seg_index = []
        self.right_seg_index = []

        self.geom_ids = []
        self.geom_names = []
        self.geom_types = []
        self.geom_trans = []
        self.geom_sizes = []
        self.body_trans = []

        self.pressure_info = {}  # type: dict[int, PressureFrameInfo]

        self.frame = -1

        self.init_model()

    def init_model(self):
        for joint_idx in range(self.model.getJointNum()):
            joint_name = self.model.index2name(joint_idx)
            if 'Foot' in joint_name:
                self.foot_index.append(joint_idx)
                if 'Left' in joint_name:
                    self.left_foot_index.append(joint_idx)
                elif 'Right' in joint_name:
                    self.right_foot_index.append(joint_idx)

            if 'foot' in joint_name:
                self.foot_seg_index.append(joint_idx)
                if 'Left' in joint_name:
                    self.left_seg_index.append(joint_idx)
                elif 'Right' in joint_name:
                    self.right_seg_index.append(joint_idx)

        q = self.model.get_q()
        q_zero = np.zeros_like(q)
        self.model.set_q(q_zero)
        for seg_idx in self.foot_seg_index:
            for i in range(self.model.getBodyGeomNum(seg_idx)):
                self.geom_names.append(self.model.index2name(seg_idx))
                self.geom_ids.append(seg_idx)
                self.body_trans.append(self.model.getBodyTransformGlobal(seg_idx))
            self.geom_types.extend(self.model.getBodyGeomsType(seg_idx))
            self.geom_sizes.extend(self.model.getBodyGeomsSize(seg_idx))
            self.geom_trans.extend(self.model.getBodyGeomsGlobalFrame(seg_idx))

        self.model.set_q(q)

    def refresh_foot_contact_info(self, frame, world, bodyIDsToCheck, mus, Ks, Ds):
        if frame not in self.pressure_info:
            self.pressure_info[frame] = PressureFrameInfo()
            bodyIDs, contactPositions, contactPositionLocals, contactForces = world.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            self.pressure_info[frame].contact_seg_idx.extend(bodyIDs)
            self.pressure_info[frame].contact_seg_position_local.extend(contactPositionLocals)
            self.pressure_info[frame].contact_seg_forces.extend(contactForces)
        self.redraw()

    def goToFrame(self, frame):
        self.frame = frame

    def initGL(self):
        glClearColor(1., 1., 1., 1.)
        self.projectOrtho(3)

    def draw(self):
        frame = self.frame
        glClear(GL_COLOR_BUFFER_BIT)
        # self.rc.drawCircle(1.)
        # self.rc.drawCapsule2D(.2, .2)
        force_max = None
        if self.pressure_info:
            force_max = max([mm.length(force) for force in self.pressure_info[frame].contact_seg_forces]) if self.pressure_info[frame].contact_seg_forces else 1000.
        for i in range(len(self.geom_types)):
            geom_seg_idx = self.geom_ids[i]
            geom_name = self.geom_names[i]
            geom_type = self.geom_types[i]
            geom_size = self.geom_sizes[i]
            geom_tran = self.geom_trans[i].copy()
            geom_body_tran = self.body_trans[i].copy()
            geom_tran[0, 3], geom_tran[1, 3], geom_tran[2, 3] = -geom_tran[0, 3], geom_tran[2, 3], 0.
            # geom_tran[1, 3] = geom_tran[2, 3]
            # geom_tran[2, 3] = 0
            geom_body_tran[0, 3], geom_body_tran[1, 3], geom_body_tran[2, 3] = -geom_body_tran[0, 3], geom_body_tran[2, 3], 0.
            # geom_body_tran[1, 3] = geom_body_tran[2, 3]
            # geom_body_tran[2, 3] = 0

            if False and geom_type is 'ELLIPSOID':
                # print(geom_tran)
                glPushMatrix()
                glMultMatrixf(geom_tran.T)
                glScalef(geom_size[0], geom_size[1], geom_size[2])
                self.rc.drawCircle(1.)
                glPopMatrix()
            elif geom_type in ('B', 'BOX'):
                pass

            if geom_type in ('C', 'D', 'E'):
                # print(geom_seg_idx, geom_name)
                # print(geom_tran)
                # print(geom_body_tran)
                glPushMatrix()
                if 'Left' in geom_name:
                    glTranslatef(-0.3, -0.3, 0.)
                else:
                    glTranslatef(0.3, -0.3, 0.)
                # glRotatef(180., 0., 1., 0.)
                glScalef(4., 4., 4.)
                glPushMatrix()
                glMultMatrixf(geom_tran.T)
                glColor3f(1., 1., 1.)
                self.rc.drawCapsule2D(geom_size[0], geom_size[1] - 2.*geom_size[0])
                glPopMatrix()

                # draw distribution of forces
                glMultMatrixf(geom_body_tran.T)

                if self.pressure_info:
                    for contact_idx in np.where(np.array(self.pressure_info[frame].contact_seg_idx) == geom_seg_idx)[0]:
                        glPushMatrix()
                        trans = self.pressure_info[frame].contact_seg_position_local[contact_idx]
                        # print(geom_seg_idx, geom_name, trans, geom_size[0])
                        # print(mm.length(self.contact_seg_forces[contact_idx]))
                        normalized_force = mm.length(self.pressure_info[frame].contact_seg_forces[contact_idx])/force_max
                        glTranslatef(trans[0], trans[1], trans[2])
                        if normalized_force < 0.5:
                            glColor3f(0., 2.*normalized_force, 1. - 2.*normalized_force)
                        else:
                            glColor3f(2.*(normalized_force-0.5), 1. - 2.*(normalized_force-0.5), 0.)
                        # glColor3f(1., 0., 0.)
                        self.rc.drawSphere(geom_size[0])
                        glPopMatrix()
                glPopMatrix()
            elif geom_type is 'CYLINDER':
                glPushMatrix()
                if 'Left' in geom_name:
                    glTranslatef(-0.3, -0.3, 0.)
                else:
                    glTranslatef(0.3, -0.3, 0.)
                glRotatef(180., 0., 1., 0.)
                glScalef(4., 4., 4.)
                glMultMatrixf(geom_tran.T)
                if geom_seg_idx in self.pressure_info[frame].contact_seg_idx:
                    glColor3f(1., 0., 0.)
                else:
                    glColor3f(1., 1., 1.)
                self.rc.drawCapsule2D(geom_size[1], geom_size[0] - 2.*geom_size[1])
                glPopMatrix()

    def projectOrtho(self, distance):
        glViewport(0, 0, self.w(), self.h())
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        x = float(self.w())/float(self.h()) * distance
        y = 1. * distance
        glOrtho(-.5*x, .5*x, -.5*y, .5*y, -1000., 1000.)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()


class FootWindow(Fl_Window):
    def __init__(self, x, y, w, h, title, model):
        Fl_Window.__init__(self, x, y, w, h, title)
        y_padding = 20

        self.model = model

        self.begin()

        self.check_op_l = Fl_Check_Button(10, 10, 30, 30, 'OP')
        self.check_ip_l = Fl_Check_Button(50, 10, 30, 30, 'IP')
        self.check_om_l = Fl_Check_Button(10, 50, 30, 30, 'OM')
        self.check_im_l = Fl_Check_Button(50, 50, 30, 30, 'IM')
        self.check_h_l = Fl_Check_Button(30, 90, 30, 30, 'H')

        self.check_op_r = Fl_Check_Button(150, 10, 30, 30, 'OP')
        self.check_ip_r = Fl_Check_Button(110, 10, 30, 30, 'IP')
        self.check_om_r = Fl_Check_Button(150, 50, 30, 30, 'OM')
        self.check_im_r = Fl_Check_Button(110, 50, 30, 30, 'IM')
        self.check_h_r = Fl_Check_Button(130, 90, 30, 30, 'H')

        self.foot_pressure_gl_window = FootPressureGlWindow(50, 150, 200, 200, model)

        self.end()


def main():
    # np.set_printoptions(precision=4, linewidth=200)
    np.set_printoptions(precision=5, threshold=np.inf, suppress=True, linewidth=3000)

    motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = mit.create_biped()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_jump_biped()

    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    # controlModel_shadow_for_ik = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    controlModel.initializeHybridDynamics()

    # controlToMotionOffset = (1.5, -0.02, 0)
    controlToMotionOffset = (1.5, 0, 0)
    controlModel.translateByOffset(controlToMotionOffset)
    # controlModel_shadow_for_ik.set_q(controlModel.get_q())
    # controlModel_shadow_for_ik.computeJacobian(0, np.array([0., 0., 0.]))

    wcfg_ik = copy.deepcopy(wcfg)
    vpWorld_ik = cvw.VpWorld(wcfg_ik)
    controlModel_ik = cvm.VpControlModel(vpWorld_ik, motion[0], mcfg)
    vpWorld_ik.initialize()
    controlModel_ik.set_q(controlModel.get_q())

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

    supL = motion[0].skeleton.getJointIndex(config['supLink1'])
    supR = motion[0].skeleton.getJointIndex(config['supLink2'])

    selectedBody = motion[0].skeleton.getJointIndex(config['end'])
    constBody = motion[0].skeleton.getJointIndex('RightFoot')

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
        Ts['foot_R'] = init
        Ts['upper_limb_R'] = init
        Ts['lower_limb_R'] = init
        Ts['thigh_L'] = init
        Ts['shin_L'] = init
        Ts['foot_L'] = init
        Ts['upper_limb_L'] = init
        Ts['lower_limb_L'] = init

        return Ts


    # viewer = ysv.SimpleViewer()
    viewer = hsv.hpSimpleViewer(rect=[0, 0, 1024, 768], viewForceWnd=False)
    # viewer.record(False)
    # viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('motionModel', yr.VpModelRenderer(motionModel, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('ikModel', yr.VpModelRenderer(controlModel_ik, (150,150,255), yr.POLYGON_LINE))
    # viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    control_model_renderer = yr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL)
    viewer.doc.addRenderer('controlModel', control_model_renderer)
    skeleton_renderer = None
    if SKELETON_ON:
        skeleton_renderer = yr.BasicSkeletonRenderer(makeEmptyBasicSkeletonTransformDict(np.eye(4)), offset_Y=-0.08)
        viewer.doc.addRenderer('skeleton', skeleton_renderer)
    viewer.doc.addRenderer('rd_footCenter', yr.PointsRenderer(rd_footCenter))
    viewer.doc.addRenderer('rd_footCenter_ref', yr.PointsRenderer(rd_footCenter_ref))
    viewer.doc.addRenderer('rd_CM_plane', yr.PointsRenderer(rd_CM_plane, (255,255,0)))
    viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (0,255,0)))
    viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (255,0,255)))
    viewer.doc.addRenderer('rd_dL_des_plane', yr.VectorsRenderer(rd_dL_des_plane, rd_CM, (255,255,0)))
    viewer.doc.addRenderer('rd_dH_des', yr.VectorsRenderer(rd_dH_des, rd_CM, (0,255,0)))
    # viewer.doc.addRenderer('rd_grf_des', yr.ForcesRenderer(rd_grf_des, rd_CP_des, (0,255,0), .001))
    viewer.doc.addRenderer('rd_CF', yr.VectorsRenderer(rd_CF, rd_CF_pos, (255,255,0)))

    viewer.doc.addRenderer('rd_root_ori', yr.OrientationsRenderer(rd_root_ori, rd_root_pos, (255,255,0)))

    viewer.doc.addRenderer('extraForce', yr.VectorsRenderer(rd_exf_des, extraForcePos, (0,255,0)))
    viewer.doc.addRenderer('extraForceEnable', yr.VectorsRenderer(rd_exfen_des, extraForcePos, (255,0,0)))

    # viewer.doc.addRenderer('right_foot_oriX', yr.VectorsRenderer(rightFootVectorX, rightFootPos, (255,0,0)))
    # viewer.doc.addRenderer('right_foot_oriY', yr.VectorsRenderer(rightFootVectorY, rightFootPos, (0,255,0)))
    # viewer.doc.addRenderer('right_foot_oriZ', yr.VectorsRenderer(rightFootVectorZ, rightFootPos, (0,0,255)))

    # viewer.doc.addRenderer('right_oriX', yr.VectorsRenderer(rightVectorX, rightPos, (255,0,0)))
    # viewer.doc.addRenderer('right_oriY', yr.VectorsRenderer(rightVectorY, rightPos, (0,255,0)))
    # viewer.doc.addRenderer('right_oriZ', yr.VectorsRenderer(rightVectorZ, rightPos, (0,0,255)))

    # foot_viewer = FootWindow(viewer.x() + viewer.w() + 20, viewer.y(), 300, 400, 'foot contact modifier', controlModel)
    foot_viewer = None  # type: FootWindow

    # success!!
    # initKt = 50
    # initKl = 10.1
    # initKh = 3.1

    # initBl = .1
    # initBh = .1
    # initSupKt = 21.6

    # initFm = 100.0

    # success!! -- 2015.2.12. double stance
    # initKt = 50
    # initKl = 37.1
    # initKh = 41.8

    # initBl = .1
    # initBh = .13
    # initSupKt = 21.6

    # initFm = 165.0

    # single stance
    # initKt = 25
    # initKl = 80.1
    # initKh = 10.8

    # initBl = .1
    # initBh = .13
    # initSupKt = 21.6

    # initFm = 50.0

    # single stance -> double stance
    # initKt = 25
    # initKl = 60.
    # initKh = 20.

    # initBl = .1
    # initBh = .13
    # initSupKt = 21.6

    # initFm = 50.0

    initKt = 25
    initKl = 100.
    initKh = 100.

    initBl = .1
    initBh = .13
    initSupKt = 17

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

    extendedFootName = ['Foot_foot_0_0', 'Foot_foot_0_1', 'Foot_foot_0_0_0', 'Foot_foot_0_1_0', 'Foot_foot_1_0']
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

    def get_jacobianbase_and_masks(skeleton, DOFs, joint_idx):
        J = yjc.makeEmptyJacobian(DOFs, 1)
        joint_masks = [yjc.getLinkJointMask(skeleton, joint_idx)]

        return J, joint_masks

    # ik_solver = hik.numIkSolver(dartIkModel)
    # ik_solver.clear()
    def getDesiredDOFAccelerations(_th_r, _th, _dth_r, _dth, _Kt, _Dt, weightMap=None):
        ddth_des = [None]*len(_th_r)  # type: list[np.ndarray]

        p_r0 = _th_r[0][0]
        p0 = _th[0][0]
        v_r0 = _dth_r[0][0:3]
        v0 = _dth[0][0:3]

        th_r0 = _th_r[0][1]
        th0 = _th[0][1]
        dth_r0 = _dth_r[0][3:6]
        dth0 = _dth[0][3:6]

        kt = Kt
        dt = Dt

        if weightMap is not None:
            kt = Kt * weightMap[0]
            dt = Dt * (weightMap[0]**.5)
            # dt = 0.
        a_des0 = kt*(p_r0 - p0) + dt*(v_r0 - v0)
        ddth_des0 = kt*(mm.logSO3(np.dot(th0.transpose(), th_r0))) + dt*(dth_r0 - dth0)
        ddth_des[0] = np.concatenate((a_des0, ddth_des0))

        for i in range(1, len(_th_r)):
            if weightMap is not None:
                kt = Kt * weightMap[i]
                dt = Dt * (weightMap[i]**.5)
                # dt = 0.

            # ddth_des[i] = kt*(mm.logSO3(np.dot(th[i].transpose(), th_r[i]))) + dt*(dth_r[i] - dth[i])
            ddth_des[i] = kt*(mm.logSO3(np.dot(_th[i].transpose(), _th_r[i]))) + dt*(-_dth[i])

        return ddth_des


    ###################################
    # simulate
    ###################################
    def simulateCallback(frame):
        # print(frame)
        # print(motion[frame].getJointOrientationLocal(footIdDic['RightFoot_foot_0_1_0']))
        if viewer_GetForceState():
            # print('force on, frame: ', frame)
            motion[frame].mulJointOrientationLocal(footIdDic['LeftFoot_foot_0_0_0'], mm.exp(mm.unitX(), -math.pi * mm.SCALAR_1_6))
            motion[frame].mulJointOrientationLocal(footIdDic['LeftFoot_foot_0_1_0'], mm.exp(mm.unitX(), -math.pi * mm.SCALAR_1_6))
            motion[frame].mulJointOrientationLocal(footIdDic['RightFoot_foot_0_0_0'], mm.exp(mm.unitX(), -math.pi * mm.SCALAR_1_6))
            motion[frame].mulJointOrientationLocal(footIdDic['RightFoot_foot_0_1_0'], mm.exp(mm.unitX(), -math.pi * mm.SCALAR_1_6))
        # print(motion[frame].getJointOrientationLocal(footIdDic['RightFoot_foot_0_1_0']))
        motionModel.update(motion[frame])
        controlModel_ik.set_q(controlModel.get_q())

        global g_initFlag
        global forceShowTime

        global preFootCenter
        global maxContactChangeCount
        global contactChangeCount
        global contact
        global contactChangeType

        # Kt, Kl, Kh, Bl, Bh, kt_sup = viewer.GetParam()
        Kt, Kl, Kh, Bl, Bh, kt_sup = getParamVals(['Kt', 'Kl', 'Kh', 'Bl', 'Bh', 'SupKt'])
        Dt = 2*(Kt**.5)
        Dl = 2*(Kl**.5)
        Dh = 2*(Kh**.5)
        dt_sup = 2*(kt_sup**.5)

        doubleTosingleOffset = 0.15
        singleTodoubleOffset = 0.30
        # doubleTosingleOffset = 0.09
        doubleTosingleVelOffset = 0.0

        # tracking
        th_r = motion.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(frame)
        # dth = controlModel.getDOFVelocities()
        dth = controlModel.get_dq_nested()
        # ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        ddth_des = getDesiredDOFAccelerations(th_r, th, dth_r, dth, Kt, Dt)

        ype.flatten(ddth_des, ddth_des_flat)
        # ddth_des_flat = Kt * (motion.get_q(frame) - np.array(controlModel.get_q())) - Dt * np.array(controlModel.get_dq())
        ype.flatten(dth, dth_flat)
        # dth_flat = np.array(controlModel.get_dq())

        #################################################
        # jacobian
        #################################################

        contact_des_ids = list()  # desired contact segments
        if foot_viewer.check_om_l.value():
            contact_des_ids.append(footIdDic['LeftFoot_foot_0_0'])
        if foot_viewer.check_op_l.value():
            contact_des_ids.append(footIdDic['LeftFoot_foot_0_0_0'])
        if foot_viewer.check_im_l.value():
            contact_des_ids.append(footIdDic['LeftFoot_foot_0_1'])
        if foot_viewer.check_ip_l.value():
            contact_des_ids.append(footIdDic['LeftFoot_foot_0_1_0'])
        if foot_viewer.check_h_l.value():
            contact_des_ids.append(footIdDic['LeftFoot_foot_1_0'])

        if foot_viewer.check_om_r.value():
            contact_des_ids.append(footIdDic['RightFoot_foot_0_0'])
        if foot_viewer.check_op_r.value():
            contact_des_ids.append(footIdDic['RightFoot_foot_0_0_0'])
        if foot_viewer.check_im_r.value():
            contact_des_ids.append(footIdDic['RightFoot_foot_0_1'])
        if foot_viewer.check_ip_r.value():
            contact_des_ids.append(footIdDic['RightFoot_foot_0_1_0'])
        if foot_viewer.check_h_r.value():
            contact_des_ids.append(footIdDic['RightFoot_foot_1_0'])

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

        # caution!! body orientation and joint orientation of foot are totally different!!
        footOriL = controlModel.getJointOrientationGlobal(supL)
        footOriR = controlModel.getJointOrientationGlobal(supR)

        # desire footCenter[1] = 0.041135
        # desire footCenter[1] = 0.0197
        footCenterL = controlModel.getBodyPositionGlobal(supL)
        footCenterR = controlModel.getBodyPositionGlobal(supR)
        footBodyOriL = controlModel.getBodyOrientationGlobal(supL)
        footBodyOriR = controlModel.getBodyOrientationGlobal(supR)
        footBodyVelL = controlModel.getBodyVelocityGlobal(supL)
        footBodyVelR = controlModel.getBodyVelocityGlobal(supR)
        footBodyAngVelL = controlModel.getBodyAngVelocityGlobal(supL)
        footBodyAngVelR = controlModel.getBodyAngVelocityGlobal(supR)

        refFootL = motionModel.getBodyPositionGlobal(supL)
        refFootR = motionModel.getBodyPositionGlobal(supR)
        refFootVelL = motionModel.getBodyVelocityGlobal(supL)
        refFootVelR = motionModel.getBodyVelocityGlobal(supR)
        refFootAngVelL = motionModel.getBodyAngVelocityGlobal(supL)
        refFootAngVelR = motionModel.getBodyAngVelocityGlobal(supR)

        refFootJointVelR = motion.getJointVelocityGlobal(supR, frame)
        refFootJointAngVelR = motion.getJointAngVelocityGlobal(supR, frame)
        refFootJointR = motion.getJointPositionGlobal(supR, frame)
        refFootVelR = refFootJointVelR + np.cross(refFootJointAngVelR, (refFootR-refFootJointR))

        refFootJointVelL = motion.getJointVelocityGlobal(supL, frame)
        refFootJointAngVelL = motion.getJointAngVelocityGlobal(supL, frame)
        refFootJointL = motion.getJointPositionGlobal(supL, frame)
        refFootVelL = refFootJointVelL + np.cross(refFootJointAngVelL, (refFootL-refFootJointL))

        is_contact = [1] * len(contact_ids)
        contactR = 1
        contactL = 1
        if refFootVelR[1] < 0 and refFootVelR[1]/30. + refFootR[1] > singleTodoubleOffset:
            contactR = 0
        if refFootVelL[1] < 0 and refFootVelL[1]/30. + refFootL[1] > singleTodoubleOffset:
            contactL = 0
        if refFootVelR[1] > 0 and refFootVelR[1]/30. + refFootR[1] > doubleTosingleOffset:
            contactR = 0
        if refFootVelL[1] > 0 and refFootVelL[1]/30. + refFootL[1] > doubleTosingleOffset:
            contactL = 0
        # contactR = 1

        contMotionOffset = th[0][0] - th_r[0][0]

        linkPositions = controlModel.getBodyPositionsGlobal()
        linkVelocities = controlModel.getBodyVelocitiesGlobal()
        linkAngVelocities = controlModel.getBodyAngVelocitiesGlobal()
        linkInertias = controlModel.getBodyInertiasGlobal()

        # CM = yrp.getCM(linkPositions, linkMasses, totalMass)
        CM = controlModel.getCOM()
        dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)
        CM_plane = copy.deepcopy(CM)
        CM_plane[1] = 0.
        dCM_plane = copy.deepcopy(dCM)
        dCM_plane[1] = 0.

        # P = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions, CM, linkInertias)
        P = ymt.get_P(linkMasses, linkPositions, CM, linkInertias)
        dP = ymt.getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities, dCM, linkAngVelocities, linkInertias)

        # calculate contact state
        if g_initFlag == 1:
            # contact state
            # 0: flying 1: right only 2: left only 3: double
            # if contact == 2 and refFootR[1] < doubleTosingleOffset:
            if contact == 2 and contactR == 1:
                contact = 3
                maxContactChangeCount += 30
                contactChangeCount += maxContactChangeCount
                contactChangeType = 'StoD'

            # elif contact == 3 and refFootL[1] < doubleTosingleOffset:
            elif contact == 1 and contactL == 1:
                contact = 3
                maxContactChangeCount += 30
                contactChangeCount += maxContactChangeCount
                contactChangeType = 'StoD'

            # elif contact == 3 and refFootR[1] > doubleTosingleOffset:
            elif contact == 3 and contactR == 0:
                contact = 2
                contactChangeCount += maxContactChangeCount
                contactChangeType = 'DtoS'

            # elif contact == 3 and refFootL[1] > doubleTosingleOffset:
            elif contact == 3 and contactL == 0:
                contact = 1
                contactChangeCount += maxContactChangeCount
                contactChangeType = 'DtoS'
            else:
                contact = 0
                # if refFootR[1] < doubleTosingleOffset:
                if contactR == 1:
                    contact += 1
                # if refFootL[1] < doubleTosingleOffset:
                if contactL == 1:
                    contact += 2

        # initialization
        if g_initFlag == 0:
            softConstPoint = footCenterR.copy()

            footCenter = footCenterL + (footCenterR - footCenterL)/2.0
            footCenter[1] = 0.
            preFootCenter = footCenter.copy()
            # footToBodyFootRotL = np.dot(np.transpose(footOriL), footBodyOriL)
            # footToBodyFootRotR = np.dot(np.transpose(footOriR), footBodyOriR)

            if refFootR[1] < doubleTosingleOffset:
                contact += 1
            if refFootL[1] < doubleTosingleOffset:
                contact += 2

            g_initFlag = 1

        # calculate jacobian
        Jsys, dJsys = controlModel.computeCom_J_dJdq()
        J_contacts = [Jsys[6*contact_id:6*contact_id+6, :] for contact_id in contact_ids]
        dJ_contacts = [dJsys[6*contact_id:6*contact_id+6] for contact_id in contact_ids]

        # calculate footCenter
        footCenter = sum(contact_body_pos) / len(contact_body_pos) if len(contact_body_pos) > 0 \
                        else .5 * (controlModel.getBodyPositionGlobal(supL) + controlModel.getBodyPositionGlobal(supR))
        # if len(contact_body_pos) > 2:
        #     hull = ConvexHull(contact_body_pos)

        footCenter_ref = sum(ref_body_pos) / len(ref_body_pos) if len(ref_body_pos) > 0 \
            else .5 * (motionModel.getBodyPositionGlobal(supL) + motionModel.getBodyPositionGlobal(supR))
        footCenter_ref = footCenter_ref + contMotionOffset
        # if len(ref_body_pos) > 2:
        #     hull = ConvexHull(ref_body_pos)
        footCenter_ref[1] = 0.

        # footCenter = footCenterL + (footCenterR - footCenterL)/2.0
        # if refFootR[1] >doubleTosingleOffset:
        # if refFootR[1] > doubleTosingleOffset or footCenterR[1] > 0.08:
        # if contact == 1 or footCenterR[1] > 0.08:
        # if contact == 2 or footCenterR[1] > doubleTosingleOffset/2:
        if contact == 2:
            footCenter = footCenterL.copy()
        # elif contact == 1 or footCenterL[1] > doubleTosingleOffset/2:
        if contact == 1:
            footCenter = footCenterR.copy()
        footCenter[1] = 0.

        footCenter[0] = footCenter[0] + getParamVal('com X offset')

        if contactChangeCount > 0 and contactChangeType == 'StoD':
            # change footcenter gradually
            footCenter = preFootCenter + (maxContactChangeCount - contactChangeCount)*(footCenter-preFootCenter)/maxContactChangeCount

        preFootCenter = footCenter.copy()

        # linear momentum
        # TODO:
        # We should consider dCM_ref, shouldn't we?
        # add getBodyPositionGlobal and getBodyPositionsGlobal in csVpModel!
        # to do that, set joint velocities to vpModel
        CM_ref_plane = footCenter
        # CM_ref_plane = footCenter_ref
        dL_des_plane = Kl * totalMass * (CM_ref_plane - CM_plane) - 0.1*Dl * totalMass * dCM_plane
        print('dL_des_plane: ', dL_des_plane)
        # dL_des_plane[1] = 0.
        # print('dCM_plane : ', np.linalg.norm(dCM_plane))

        # angular momentum
        CP_ref = footCenter
        # CP_ref = footCenter_ref
        bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
        # bodyIDs, contactPositions, contactPositionLocals, contactForces, contactVelocities = vpWorld.calcManyPenaltyForce(0, bodyIDsToCheck, mus, Ks, Ds)
        CP = yrp.getCP(contactPositions, contactForces)
        if CP_old[0] is None or CP is None:
            dCP = None
        else:
            dCP = (CP - CP_old[0])/(1/30.)
        CP_old[0] = CP

        if CP is not None and dCP is not None:
            ddCP_des = Kh*(CP_ref - CP) - Dh * dCP
            dCP_des = dCP + ddCP_des * (1/30.)
            CP_des = CP + dCP_des * (1/30.)
            # CP_des = footCenter
            CP_des = CP + dCP*(1/30.) + .5*ddCP_des*((1/30.)**2)
            dH_des = np.cross((CP_des - CM), (dL_des_plane + totalMass * mm.s2v(wcfg.gravity)))
            if contactChangeCount > 0:  # and contactChangeType == 'DtoS':
                dH_des *= (maxContactChangeCount - contactChangeCount)/maxContactChangeCount
        else:
            dH_des = None

        # convex hull
        contact_pos_2d = np.asarray([np.array([contactPosition[0], contactPosition[2]]) for contactPosition in contactPositions])
        p = np.array([CM_plane[0], CM_plane[2]])
        # hull = None  # type: Delaunay
        # if contact_pos_2d.shape[0] > 0:
        #     hull = Delaunay(contact_pos_2d)
        #     print(hull.find_simplex(p) >= 0)

        # set up equality constraint
        # TODO:
        # logSO3 is just q'', not acceleration.
        # To make a_oris acceleration, q'' -> a will be needed
        # body_ddqs = list(map(mm.logSO3, [mm.getSO3FromVectors(np.dot(body_ori, mm.unitY()), mm.unitY()) for body_ori in contact_body_ori]))
        body_ddqs = list(map(mm.logSO3, [np.dot(contact_body_ori[i].T, np.dot(ref_body_ori[i], mm.getSO3FromVectors(np.dot(ref_body_ori[i], mm.unitY()), mm.unitY()))) for i in range(len(contact_body_ori))]))
        body_qs = list(map(mm.logSO3, contact_body_ori))
        body_angs = [np.dot(contact_body_ori[i].T, contact_body_angvel[i]) for i in range(len(contact_body_ori))]
        body_dqs = [mm.vel2qd(body_angs[i], body_qs[i]) for i in range(len(body_angs))]
        a_oris = [np.dot(contact_body_ori[i], mm.qdd2accel(body_ddqs[i], body_dqs[i], body_qs[i])) for i in range(len(contact_body_ori))]

        # body_ddq = body_ddqs[0]
        # body_ori = contact_body_ori[0]
        # body_ang = np.dot(body_ori.T, contact_body_angvel[0])
        #
        # body_q = mm.logSO3(body_ori)
        # body_dq = mm.vel2qd(body_ang, body_q)
        # a_ori = np.dot(body_ori, mm.qdd2accel(body_ddq, body_dq, body_q))

        # a_oris = list(map(mm.logSO3, [mm.getSO3FromVectors(np.dot(body_ori, mm.unitY()), mm.unitY()) for body_ori in contact_body_ori]))
        a_sups = [np.append(kt_sup*(ref_body_pos[i] - contact_body_pos[i] + contMotionOffset) + dt_sup*(ref_body_vel[i] - contact_body_vel[i]),
                            kt_sup*a_oris[i]+dt_sup*(ref_body_angvel[i]-contact_body_angvel[i])) for i in range(len(a_oris))]

        # momentum matrix
        RS = np.dot(P, Jsys)
        R, S = np.vsplit(RS, 2)

        rs = np.dot(dP, np.dot(Jsys, dth_flat)) + np.dot(P, dJsys)
        r_bias, s_bias = np.hsplit(rs, 2)

        #######################################################
        # optimization
        #######################################################
        # if contact == 2 and footCenterR[1] > doubleTosingleOffset/2:
        if contact == 2:
            config['weightMap']['RightUpLeg'] = .8
            config['weightMap']['RightLeg'] = .8
            config['weightMap']['RightFoot'] = .8
        else:
            config['weightMap']['RightUpLeg'] = .1
            config['weightMap']['RightLeg'] = .25
            config['weightMap']['RightFoot'] = .2

        # if contact == 1 and footCenterL[1] > doubleTosingleOffset/2:
        if contact == 1:
            config['weightMap']['LeftUpLeg'] = .8
            config['weightMap']['LeftLeg'] = .8
            config['weightMap']['LeftFoot'] = .8
        else:
            config['weightMap']['LeftUpLeg'] = .1
            config['weightMap']['LeftLeg'] = .25
            config['weightMap']['LeftFoot'] = .2

        w = mot.getTrackingWeight(DOFs, motion[0].skeleton, config['weightMap'])

        # if contact == 2:
        #     mot.addSoftPointConstraintTerms(problem, totalDOF, Bsc, ddP_des1, Q1, q_bias1)

        mot.addTrackingTerms(problem, totalDOF, Bt, w, ddth_des_flat)
        if dH_des is not None:
            mot.addLinearTerms(problem, totalDOF, Bl, dL_des_plane, R, r_bias)
            # mot.addAngularTerms(problem, totalDOF, Bh, dH_des, S, s_bias)

            # mot.setConstraint(problem, totalDOF, Jsup, dJsup, dth_flat, a_sup)
            # mot.addConstraint(problem, totalDOF, Jsup, dJsup, dth_flat, a_sup)
            # if contact & 1 and contactChangeCount == 0:
            if False:
                for c_idx in range(len(contact_ids)):
                    mot.addConstraint2(problem, totalDOF, J_contacts[c_idx], dJ_contacts[c_idx], dth_flat, a_sups[c_idx])

        if contactChangeCount > 0:
            contactChangeCount = contactChangeCount - 1
            if contactChangeCount == 0:
                maxContactChangeCount = 30
                contactChangeType = 0

        r = problem.solve()
        problem.clear()
        ddth_sol_flat = np.asarray(r['x'])
        # print(np.dot(Jsys, ddth_sol_flat))
        print(np.dot(J_contacts[0], ddth_sol_flat))
        # ddth_sol_flat[foot_seg_dofs] = np.array(ddth_des_flat)[foot_seg_dofs]
        ype.nested(ddth_sol_flat, ddth_sol)

        rootPos[0] = controlModel.getBodyPositionGlobal(selectedBody)
        localPos = [[0, 0, 0]]

        for i in range(stepsPerFrame):
            # apply penalty force
            bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            # bodyIDs, contactPositions, contactPositionLocals, contactForces, contactVelocities = vpWorld.calcManyPenaltyForce(0, bodyIDsToCheck, mus, Ks, Ds)
            vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)

            # controlModel.setDOFAccelerations(ddth_sol)
            # controlModel.setDOFAccelerations(ddth_des)
            controlModel.set_ddq(ddth_sol_flat)
            # controlModel.set_ddq(ddth_des_flat)
            controlModel.solveHybridDynamics()

            if forceShowTime > viewer.objectInfoWnd.labelForceDur.value():
                forceShowTime = 0
                viewer_ResetForceState()

            forceforce = np.array([viewer.objectInfoWnd.labelForceX.value(), viewer.objectInfoWnd.labelForceY.value(), viewer.objectInfoWnd.labelForceZ.value()])
            extraForce[0] = getParamVal('Fm') * mm.normalize2(forceforce)
            if viewer_GetForceState():
                forceShowTime += wcfg.timeStep
                vpWorld.applyPenaltyForce(selectedBodyId, localPos, extraForce)

            vpWorld.step()

        controlModel_ik.set_q(controlModel.get_q())

        if foot_viewer is not None:
            foot_viewer.foot_pressure_gl_window.refresh_foot_contact_info(frame, vpWorld, bodyIDsToCheck, mus, Ks, Ds)
            foot_viewer.foot_pressure_gl_window.goToFrame(frame)

        # rendering
        for foot_seg_id in footIdlist:
            control_model_renderer.body_colors[foot_seg_id] = (255, 240, 255)

        for contact_id in contact_ids:
            control_model_renderer.body_colors[contact_id] = (255, 0, 0)

        rightFootVectorX[0] = np.dot(footOriL, np.array([.1, 0, 0]))
        rightFootVectorY[0] = np.dot(footOriL, np.array([0, .1, 0]))
        rightFootVectorZ[0] = np.dot(footOriL, np.array([0, 0, .1]))
        rightFootPos[0] = footCenterL

        rightVectorX[0] = np.dot(footBodyOriL, np.array([.1, 0, 0]))
        rightVectorY[0] = np.dot(footBodyOriL, np.array([0, .1, 0]))
        rightVectorZ[0] = np.dot(footBodyOriL, np.array([0, 0, .1]))
        rightPos[0] = footCenterL + np.array([.1, 0, 0])

        rd_footCenter[0] = footCenter
        rd_footCenter_ref[0] = footCenter_ref
        rd_footCenterL[0] = footCenterL
        rd_footCenterR[0] = footCenterR

        rd_CM[0] = CM

        rd_CM_plane[0] = CM.copy()
        rd_CM_plane[0][1] = 0.

        if CP is not None and dCP is not None:
            rd_CP[0] = CP
            rd_CP_des[0] = CP_des

            rd_dL_des_plane[0] = [dL_des_plane[0]/100, dL_des_plane[1]/100, dL_des_plane[2]/100]
            rd_dH_des[0] = dH_des

            rd_grf_des[0] = dL_des_plane - totalMass * mm.s2v(wcfg.gravity)

        rd_root_des[0] = rootPos[0]
        rd_root_ori[0] = controlModel.getBodyOrientationGlobal(0)
        rd_root_pos[0] = controlModel.getBodyPositionGlobal(0)

        del rd_CF[:]
        del rd_CF_pos[:]
        for i in range(len(contactPositions)):
            rd_CF.append(contactForces[i]/400)
            rd_CF_pos.append(contactPositions[i].copy())

        if viewer_GetForceState():
            rd_exfen_des[0] = [extraForce[0][0]/100, extraForce[0][1]/100, extraForce[0][2]/100]
            rd_exf_des[0] = [0, 0, 0]
        else:
            rd_exf_des[0] = [extraForce[0][0]/100, extraForce[0][1]/100, extraForce[0][2]/100]
            rd_exfen_des[0] = [0, 0, 0]

        extraForcePos[0] = controlModel.getBodyPositionGlobal(selectedBody)

        # render contact_ids

        # render skeleton
        if SKELETON_ON:
            Ts = dict()
            Ts['pelvis'] = controlModel.getJointTransform(0)
            Ts['thigh_R'] = controlModel.getJointTransform(1)
            Ts['shin_R'] = controlModel.getJointTransform(2)
            Ts['foot_R'] = controlModel.getJointTransform(3)
            Ts['spine_ribs'] = controlModel.getJointTransform(9)
            Ts['head'] = controlModel.getJointTransform(10)
            Ts['upper_limb_R'] = controlModel.getJointTransform(13)
            Ts['lower_limb_R'] = controlModel.getJointTransform(14)
            Ts['thigh_L'] = controlModel.getJointTransform(15)
            Ts['shin_L'] = controlModel.getJointTransform(16)
            Ts['foot_L'] = controlModel.getJointTransform(17)
            Ts['upper_limb_L'] = controlModel.getJointTransform(11)
            Ts['lower_limb_L'] = controlModel.getJointTransform(12)

            skeleton_renderer.appendFrameState(Ts)

    viewer.setSimulateCallback(simulateCallback)
    viewer.startTimer(1/30.)
    # viewer.play()
    viewer.show()

    foot_viewer = FootWindow(viewer.x() + viewer.w() + 20, viewer.y(), 300, 500, 'foot contact modifier', controlModel)
    foot_viewer.show()

    Fl.run()


main()
