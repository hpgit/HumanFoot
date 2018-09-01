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
from PyCommon.modules.Simulator import csVpDartModel as cvdm
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Optimization import ysAnalyticConstrainedOpt as yac
from PyCommon.modules.Util import ysPythonEx as ype
from PyCommon.modules.ArticulatedBody import ysReferencePoints as yrp
from PyCommon.modules.ArticulatedBody import ysMomentum as ymt
from PyCommon.modules.ArticulatedBody import ysControl as yct

from MomentumProject.working_example import mtOptimize as mot
from MomentumProject.working_example import mtInitialize as mit
from MomentumProject.working_example.foot_window import FootWindow

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

LEG_FLEXIBLE = True


def main():
    # np.set_printoptions(precision=4, linewidth=200)
    np.set_printoptions(precision=5, threshold=np.inf, suppress=True, linewidth=3000)

    config = dict()
    config['weightMap'] = {'j_scapula_left':.2, 'j_bicep_left':.2, 'j_forearm_left':.2, 'j_hand_left':.2,
                           'j_scapula_right':.2, 'j_bicep_right':.2, 'j_forearm_right':.2, 'j_hand_right':.2,
                           'j_abdomen':.6, 'j_spine':.6, 'j_head':.6, 'j_heel_right':.2, 'j_heel_left':.2, 'j_pelvis':0.5,
                           'j_thigh_left':5., 'j_shin_left':.5, 'j_thigh_right':5., 'j_shin_right':.5}

    motionModel = cvdm.VpDartModel("cart_pole_blade.skel")
    motionModel.translateByOffset((1.5, 0.9, 0.))
    controlModel = cvdm.VpDartModel("cart_pole_blade.skel")
    controlModel.translateByOffset((0., 0.9, 0.))
    # vpWorld.SetGlobalDamping(0.999)
    controlModel.initializeHybridDynamics()

    render_fps = 40
    stepsPerFrame = round(1./(controlModel.getTimeStep() * render_fps))

    # for j in range(1, dartModel.getJointNum()):
    #     print(j, dartModel.getJointOrientationLocal(j))

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
    # tracking gain
    Kt = 25.
    Dt = 2.*(Kt**.5)

    # linear balance gain
    Kl = 100.
    Dl = 2.*(Kt**.5)

    # angular balance gain
    Kh = 100.
    Dh = 2.*(Kt**.5)

    # penalty force spring gain
    Ks = 15000.
    Ds = 2.*(Kt**.5)

    # objective weight
    Bt = 1.
    Bl = 0.1
    Bh = 0.13

    # selectedBody = motion[0].skeleton.getJointIndex(config['end'])
    selectedBody = controlModel.name2index('h_spine')

    supL = controlModel.name2index('h_blade_left')
    supR = controlModel.name2index('h_blade_right')

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
    # bodyIDsToCheck = list(range(controlModel.getBodyNum()))
    bodyIDsToCheck = [supL]
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

    rd_body_ori = [None]
    rd_body_pos = [None]

    rd_root_ori = [None]
    rd_root_pos = [None]

    rd_CF = [None]
    rd_CF_pos = [None]

    rootPos = [None]
    selectedBodyId = [controlModel.index2vpid(selectedBody)]
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

    # viewer = ysv.SimpleViewer()
    # viewer = hsv.hpSimpleViewer(rect=[0, 0, 1024, 768], viewForceWnd=False)
    viewer = hsv.hpSimpleViewer(rect=[0, 0, 960+300, 1+1080+55], viewForceWnd=False)
    # viewer.record(False)
    # viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    # viewer.doc.addObject('motion', motion)
    viewer.setMaxFrame(2000)
    viewer.doc.addRenderer('motionModel', yr.VpModelRenderer(motionModel, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.setRendererVisible('motionModel', False)
    control_model_renderer = yr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL)
    viewer.doc.addRenderer('controlModel', control_model_renderer)

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
    # viewer.doc.setRendererVisible('rd_CF', False)
    viewer.doc.addRenderer('rd_foot_ori', yr.OrientationsRenderer(rd_foot_ori, rd_foot_pos, (255,255,0)))
    viewer.doc.setRendererVisible('rd_foot_ori', False)

    viewer.doc.addRenderer('rd_root_ori', yr.OrientationsRenderer(rd_root_ori, rd_root_pos, (255,255,0)))
    viewer.doc.setRendererVisible('rd_root_ori', False)

    viewer.doc.addRenderer('rd_body_ori', yr.OrientationsRenderer(rd_body_ori, rd_body_pos, (255,255,0)))

    viewer.doc.addRenderer('extraForce', yr.VectorsRenderer(rd_exf_des, extraForcePos, (0,255,0)))
    viewer.doc.setRendererVisible('extraForce', False)
    # viewer.doc.addRenderer('extraForceEnable', yr.VectorsRenderer(rd_exfen_des, extraForcePos, (255,0,0)))
    viewer.doc.addRenderer('extraForceEnable', yr.WideArrowRenderer(rd_exfen_des, extraForcePos, (255,0,0), lineWidth=.05, fromPoint=False))

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
    # initSupKt = 32.
    initSupKt = 28.

    initFm = 45.0

    initComX = 0.
    initComY = 0.
    initComZ = 0.

    viewer.objectInfoWnd.add1DSlider("Kt", 0., 300., 1., initKt)
    viewer.objectInfoWnd.add1DSlider("Kl", 0., 300., 1., initKl)
    viewer.objectInfoWnd.add1DSlider("Kh", 0., 300., 1., initKh)
    viewer.objectInfoWnd.add1DSlider("Bl", 0., 1., .001, initBl)
    viewer.objectInfoWnd.add1DSlider("Bh", 0., 1., .001, initBh)
    viewer.objectInfoWnd.add1DSlider("SupKt", 0., 300., 0.1, initSupKt)
    viewer.objectInfoWnd.add1DSlider("Fm", 0., 1000., 1., initFm)
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
    viewer.objectInfoWnd.labelForceZ.value(-1)

    viewer.objectInfoWnd.labelForceDur = Fl_Value_Input(220, 30+offset*9, 40, 20, 'Dur')
    viewer.objectInfoWnd.labelForceDur.value(0.4)

    viewer.objectInfoWnd.end()

    # self.sliderFm = Fl_Hor_Nice_Slider(10, 42+offset*6, 250, 10)

    def getParamVal(paramname):
        return viewer.objectInfoWnd.getVal(paramname)

    def getParamVals(paramnames):
        return (getParamVal(name) for name in paramnames)

    def setParamVal(paramname, val):
        viewer.objectInfoWnd.setVal(paramname, val)

    idDic = dict()
    for i in range(controlModel.getBodyNum()):
        idDic[controlModel.index2name(i)] = i

    # extendedFootName = ['Foot_foot_0_0', 'Foot_foot_0_1', 'Foot_foot_0_0_0', 'Foot_foot_0_1_0', 'Foot_foot_1_0']
    extendedFootName = ['h_blade']
    # lIDdic = {'Left'+name: motion[0].skeleton.getJointIndex('Left'+name) for name in extendedFootName}
    # rIDdic = {'Right'+name: motion[0].skeleton.getJointIndex('Right'+name) for name in extendedFootName}
    lIDdic = {name + '_left' : controlModel.name2index(name + '_left') for name in extendedFootName}
    rIDdic = {name + '_right': controlModel.name2index(name + '_right') for name in extendedFootName}
    footIdDic = lIDdic.copy()
    footIdDic.update(rIDdic)

    lIDlist = [controlModel.name2index(name + '_left') for name in extendedFootName]
    rIDlist = [controlModel.name2index(name + '_right') for name in extendedFootName]
    footIdlist = []
    footIdlist.extend(lIDlist)
    footIdlist.extend(rIDlist)
    print(footIdlist)

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

    start_frame = 200

    up_vec_in_each_link = dict()
    for foot_id in footIdlist:
        up_vec_in_each_link[foot_id] = mm.unitY()

    ###################################
    # simulate
    ###################################
    def simulateCallback(frame):
        # print(frame)
        # print(motion[frame].getJointOrientationLocal(footIdDic['RightFoot_foot_0_1_0']))

        # hfi.footAdjust(motion[frame], idDic, SEGMENT_FOOT_MAG=.03, SEGMENT_FOOT_RAD=.015, baseHeight=0.02)

        # motionModel.update(motion[frame])
        motionModel.translateByOffset(np.array([getParamVal('com X offset'), getParamVal('com Y offset'), getParamVal('com Z offset')]))
        # controlModel_ik.set_q(controlModel.get_q())

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
        # th_r = motion.getDOFPositions(frame)
        th_r = motionModel.getDOFPositions()
        th = controlModel.getDOFPositions()
        # dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        # ddth_r = motion.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, None, dth, None, Kt, Dt)

        # ype.flatten(fix_dofs(DOFs, ddth_des, mcfg, joint_names), ddth_des_flat)
        # ype.flatten(fix_dofs(DOFs, dth, mcfg, joint_names), dth_flat)
        # print(ddth_des)
        ype.flatten(ddth_des, ddth_des_flat)
        ype.flatten(dth, dth_flat)

        #################################################
        # jacobian
        #################################################

        contact_des_ids = list()  # desired contact segments
        contact_des_ids.append(controlModel.name2index('h_blade_left'))
        # if foot_viewer.check_h_l.value():
        #     contact_des_ids.append(motion[0].skeleton.getJointIndex('LeftFoot'))
        #
        # if foot_viewer.check_h_r.value():
        #     contact_des_ids.append(motion[0].skeleton.getJointIndex('RightFoot'))

        contact_ids = list()  # temp idx for balancing
        contact_ids.extend(contact_des_ids)

        contact_joint_ori = list(map(controlModel.getJointOrientationGlobal, contact_ids))
        contact_joint_pos = list(map(controlModel.getJointPositionGlobal, contact_ids))
        contact_body_ori = list(map(controlModel.getBodyOrientationGlobal, contact_ids))
        contact_body_pos = list(map(controlModel.getBodyPositionGlobal, contact_ids))
        contact_body_vel = list(map(controlModel.getBodyVelocityGlobal, contact_ids))
        contact_body_angvel = list(map(controlModel.getBodyAngVelocityGlobal, contact_ids))

        ref_joint_ori = list(map(motionModel.getJointOrientationGlobal, contact_ids))
        ref_joint_pos = list(map(motionModel.getJointPositionGlobal, contact_ids))
        ref_joint_vel = [motionModel.getJointVelocityGlobal(joint_idx) for joint_idx in contact_ids]
        ref_joint_angvel = [motionModel.getJointAngVelocityGlobal(joint_idx) for joint_idx in contact_ids]
        ref_body_ori = list(map(motionModel.getBodyOrientationGlobal, contact_ids))
        ref_body_pos = list(map(motionModel.getBodyPositionGlobal, contact_ids))
        # ref_body_vel = list(map(controlModel.getBodyVelocityGlobal, contact_ids))
        ref_body_angvel = [motionModel.getJointAngVelocityGlobal(joint_idx) for joint_idx in contact_ids]
        ref_body_vel = [ref_joint_vel[i] + np.cross(ref_joint_angvel[i], ref_body_pos[i] - ref_joint_pos[i])
                        for i in range(len(ref_joint_vel))]

        is_contact = [1] * len(contact_ids)
        contact_right = len(set(contact_des_ids).intersection(rIDlist)) > 0
        contact_left = len(set(contact_des_ids).intersection(lIDlist)) > 0

        contMotionOffset = th[0][0] - th_r[0][0]

        linkPositions = [controlModel.getBodyComPositionGlobal(i) for i in range(controlModel.getBodyNum())]
        linkVelocities = [controlModel.getBodyComVelocityGlobal(i) for i in range(controlModel.getBodyNum())]
        linkAngVelocities = [controlModel.getBodyAngVelocityGlobal(i) for i in range(controlModel.getBodyNum())]
        linkInertias = [controlModel.getBodyInertiaGlobal(i) for i in range(controlModel.getBodyNum())]

        CM = yrp.getCM(linkPositions, linkMasses, totalMass)
        dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)
        CM_plane = copy.copy(CM)
        CM_plane[1] = 0.
        dCM_plane = copy.copy(dCM)
        dCM_plane[1] = 0.

        P = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions, CM, linkInertias)
        dP = ymt.getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities, dCM, linkAngVelocities, linkInertias)

        # calculate jacobian
        Jsys, dJsys = controlModel.computeCom_J_dJdq()
        J_contacts = []  # type: list[np.ndarray]
        dJ_contacts = []  # type: list[np.ndarray]
        for contact_id in contact_ids:
            J_contacts.append(Jsys[6*contact_id:6*contact_id + 6, :])
            dJ_contacts.append(dJsys[6*contact_id:6*contact_id + 6])

        # calculate footCenter
        footCenter = sum(contact_body_pos) / len(contact_body_pos) if len(contact_body_pos) > 0 \
            else .5 * (controlModel.getBodyComPositionGlobal(supL) + controlModel.getBodyComPositionGlobal(supR))
        footCenter[1] = 0.
        # if len(contact_body_pos) > 2:
        #     hull = ConvexHull(contact_body_pos)

        footCenter_ref = sum(ref_body_pos) / len(ref_body_pos) if len(ref_body_pos) > 0 \
            else .5 * (motionModel.getBodyComPositionGlobal(supL) + motionModel.getBodyComPositionGlobal(supR))
        footCenter_ref = footCenter_ref + contMotionOffset
        # if len(ref_body_pos) > 2:
        #     hull = ConvexHull(ref_body_pos)
        footCenter_ref[1] = 0.

        # footCenter[0] = footCenter[0] + getParamVal('com X offset')
        # footCenter[1] = footCenter[0] + getParamVal('com Y offset')
        # footCenter[2] = footCenter[2] + getParamVal('com Z offset')

        # initialization
        if g_initFlag == 0:
            preFootCenter[0] = footCenter.copy()
            g_initFlag = 1

        # if contactChangeCount == 0 and np.linalg.norm(footCenter - preFootCenter[0]) > 0.01:
        #     contactChangeCount += 30
        if contactChangeCount > 0:
            # change footcenter gradually
            footCenter = preFootCenter[0] + (maxContactChangeCount - contactChangeCount)*(footCenter-preFootCenter[0])/maxContactChangeCount
        else:
            preFootCenter[0] = footCenter.copy()

        # linear momentum
        # TODO:
        # We should consider dCM_ref, shouldn't we?
        # add getBodyPositionGlobal and getBodyPositionsGlobal in csVpModel!
        # to do that, set joint velocities to vpModel
        CM_ref_plane = footCenter
        # CM_ref_plane = footCenter_ref
        CM_ref = footCenter + np.array([getParamVal('com X offset'), motionModel.getCOM()[1] + getParamVal('com Y offset'), getParamVal('com Z offset')])
        dL_des_plane = Kl * totalMass * (CM_ref - CM) - Dl * totalMass * dCM
        # dL_des_plane = Kl * totalMass * (CM_ref_plane - CM_plane) - Dl * totalMass * dCM_plane
        # dL_des_plane[1] = 0.
        # print('dCM_plane : ', np.linalg.norm(dCM_plane))

        # angular momentum
        CP_ref = footCenter
        # CP_ref = footCenter_ref
        bodyIDs, contactPositions, contactPositionLocals, contactForces = controlModel.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
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
            dH_des = np.cross((CP_des - CM), (dL_des_plane + totalMass * controlModel.getGravity()))
            if contactChangeCount > 0:  # and contactChangeType == 'DtoS':
                dH_des *= (maxContactChangeCount - contactChangeCount)/maxContactChangeCount
        else:
            dH_des = None

        # set up equality constraint
        # TODO:
        # logSO3 is just q'', not acceleration.
        # To make a_oris acceleration, q'' -> a will be needed
        # body_ddqs = list(map(mm.logSO3, [mm.getSO3FromVectors(np.dot(body_ori, mm.unitY()), mm.unitY()) for body_ori in contact_body_ori]))
        # body_ddqs = list(map(mm.logSO3, [np.dot(contact_body_ori[i].T, np.dot(ref_body_ori[i], mm.getSO3FromVectors(np.dot(ref_body_ori[i], mm.unitY()), mm.unitY()))) for i in range(len(contact_body_ori))]))
        # body_ddqs = list(map(mm.logSO3, [np.dot(contact_body_ori[i].T, np.dot(ref_body_ori[i], mm.getSO3FromVectors(np.dot(ref_body_ori[i], up_vec_in_each_link[contact_ids[i]]), mm.unitY()))) for i in range(len(contact_body_ori))]))
        a_oris = list(map(mm.logSO3, [np.dot(contact_body_ori[i].T, np.dot(ref_body_ori[i], mm.getSO3FromVectors(np.dot(ref_body_ori[i], up_vec_in_each_link[contact_ids[i]]), mm.unitY()))) for i in range(len(contact_body_ori))]))
        a_oris = list(map(mm.logSO3, [np.dot(np.dot(ref_body_ori[i], mm.getSO3FromVectors(np.dot(ref_body_ori[i], up_vec_in_each_link[contact_ids[i]]), mm.unitY())), contact_body_ori[i].T) for i in range(len(contact_body_ori))]))
        body_qs = list(map(mm.logSO3, contact_body_ori))
        body_angs = [np.dot(contact_body_ori[i], contact_body_angvel[i]) for i in range(len(contact_body_ori))]
        body_dqs = [mm.vel2qd(body_angs[i], body_qs[i]) for i in range(len(body_angs))]
        # a_oris = [np.dot(contact_body_ori[i], mm.qdd2accel(body_ddqs[i], body_dqs[i], body_qs[i])) for i in range(len(contact_body_ori))]

        # body_ddq = body_ddqs[0]
        # body_ori = contact_body_ori[0]
        # body_ang = np.dot(body_ori.T, contact_body_angvel[0])
        #
        # body_q = mm.logSO3(body_ori)
        # body_dq = mm.vel2qd(body_ang, body_q)
        # a_ori = np.dot(body_ori, mm.qdd2accel(body_ddq, body_dq, body_q))

        KT_SUP = np.diag([kt_sup/10., kt_sup, kt_sup/10.])
        # KT_SUP = np.diag([kt_sup, kt_sup, kt_sup])

        # a_oris = list(map(mm.logSO3, [mm.getSO3FromVectors(np.dot(body_ori, mm.unitY()), mm.unitY()) for body_ori in contact_body_ori]))
        a_oris = list(map(mm.logSO3, [mm.getSO3FromVectors(np.dot(contact_body_ori[i], up_vec_in_each_link[contact_ids[i]]), mm.unitY()) for i in range(len(contact_body_ori))]))
        # a_sups = [np.append(kt_sup*(ref_body_pos[i] - contact_body_pos[i] + contMotionOffset) + dt_sup*(ref_body_vel[i] - contact_body_vel[i]),
        #                     kt_sup*a_oris[i]+dt_sup*(ref_body_angvel[i]-contact_body_angvel[i])) for i in range(len(a_oris))]
        # a_sups = [np.append(kt_sup*(ref_body_pos[i] - contact_body_pos[i] + contMotionOffset) - dt_sup * contact_body_vel[i],
        #                     kt_sup*a_oris[i] - dt_sup * contact_body_angvel[i]) for i in range(len(a_oris))]
        a_sups = [np.append(np.dot(KT_SUP, (ref_body_pos[i] - contact_body_pos[i] + contMotionOffset)) - dt_sup * contact_body_vel[i],
                            kt_sup*a_oris[i] - dt_sup * contact_body_angvel[i]) for i in range(len(a_oris))]
        # for i in range(len(a_sups)):
        #     a_sups[i][1] = -kt_sup * contact_body_pos[i][1] - dt_sup * contact_body_vel[i][1]

        # momentum matrix
        RS = np.dot(P, Jsys)
        R, S = np.vsplit(RS, 2)

        # rs = np.dot((np.dot(dP, Jsys) + np.dot(P, dJsys)), dth_flat)
        rs = np.dot(dP, np.dot(Jsys, dth_flat)) + np.dot(P, dJsys)
        r_bias, s_bias = np.hsplit(rs, 2)

        #######################################################
        # optimization
        #######################################################
        if LEG_FLEXIBLE:
            if contact == 2:
                config['weightMap']['j_thigh_right'] = .8
                config['weightMap']['j_shin_right'] = .8
                config['weightMap']['j_heel_right'] = .8
            else:
                config['weightMap']['j_thigh_right'] = .1
                config['weightMap']['j_shin_right'] = .25
                config['weightMap']['j_heel_right'] = .2

            if contact == 1:
                config['weightMap']['j_thigh_left'] = .8
                config['weightMap']['j_shin_left'] = .8
                config['weightMap']['j_heel_left'] = .8
            else:
                config['weightMap']['j_thigh_left'] = .1
                config['weightMap']['j_shin_left'] = .25
                config['weightMap']['j_heel_left'] = .2

        w = mot.getTrackingWeightVp(DOFs, controlModel, config['weightMap'])

        mot.addTrackingTerms(problem, totalDOF, Bt, w, ddth_des_flat)
        if dH_des is not None:
            mot.addLinearTerms(problem, totalDOF, Bl, dL_des_plane, R, r_bias)
            mot.addAngularTerms(problem, totalDOF, Bh, dH_des, S, s_bias)

            if True:
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
        # ddth_sol_flat[foot_seg_dofs] = np.array(ddth_des_flat)[foot_seg_dofs]
        ype.nested(ddth_sol_flat, ddth_sol)

        rootPos[0] = controlModel.getBodyPositionGlobal(selectedBody)
        localPos = [[0, 0, 0]]

        for i in range(stepsPerFrame):
            # apply penalty force
            controlModel.setDOFAccelerations(ddth_sol)
            # controlModel.setDOFAccelerations(ddth_des)
            # controlModel.set_ddq(ddth_sol_flat)
            # controlModel.set_ddq(ddth_des_flat)
            controlModel.solveHybridDynamics()

            if forceShowTime > viewer.objectInfoWnd.labelForceDur.value():
                forceShowTime = 0
                viewer_ResetForceState()

            forceforce = np.array([viewer.objectInfoWnd.labelForceX.value(), viewer.objectInfoWnd.labelForceY.value(), viewer.objectInfoWnd.labelForceZ.value()])
            extraForce[0] = getParamVal('Fm') * mm.normalize2(forceforce)
            if viewer_GetForceState():
                forceShowTime += controlModel.getTimeStep()
                controlModel.applyPenaltyForce(selectedBodyId, localPos, extraForce)

            bodyIDs, contactPositions, contactPositionLocals, contactForces = controlModel.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            controlModel.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)
            controlModel.step()

        # rendering
        # bodyIDs, geomIDs, positionLocalsForGeom = vpWorld.getContactInfoForcePlate(bodyIDsToCheck)
        # for foot_seg_id in footIdlist:
        #     control_model_renderer.body_colors[foot_seg_id] = (255, 240, 255)
        #     control_model_renderer.geom_colors[foot_seg_id] = [(255, 240, 255)] * controlModel.getBodyGeomNum(foot_seg_id)

        # for i in range(len(geomIDs)):
        #     if controlModel.vpid2index(bodyIDs[i]) in footIdlist:
        #         control_model_renderer.geom_colors[controlModel.vpid2index(bodyIDs[i])][geomIDs[i]] = (255, 0, 0)
        # for foot_seg_id in footIdlist:
        #     control_model_renderer.body_colors[foot_seg_id] = (255, 240, 255)
        #
        # for contact_id in contact_ids:
        #     control_model_renderer.body_colors[contact_id] = (255, 0, 0)


        rd_footCenter[0] = footCenter
        rd_footCenter_ref[0] = footCenter_ref

        rd_CM[0] = CM

        rd_CM_plane[0] = CM.copy()
        rd_CM_plane[0][1] = 0.

        if CP is not None and dCP is not None:
            rd_CP[0] = CP
            rd_CP_des[0] = CP_des

            rd_dL_des_plane[0] = [dL_des_plane[0]/100, dL_des_plane[1]/100, dL_des_plane[2]/100]
            rd_dH_des[0] = dH_des

            rd_grf_des[0] = dL_des_plane - totalMass * controlModel.getGravity()

        del rd_foot_ori[:]
        del rd_foot_pos[:]
        # for seg_foot_id in footIdlist:
        #     rd_foot_ori.append(controlModel.getJointOrientationGlobal(seg_foot_id))
        #     rd_foot_pos.append(controlModel.getJointPositionGlobal(seg_foot_id))
        rd_foot_ori.append(controlModel.getJointOrientationGlobal(supL))
        rd_foot_ori.append(controlModel.getJointOrientationGlobal(supR))
        rd_foot_pos.append(controlModel.getJointPositionGlobal(supL))
        rd_foot_pos.append(controlModel.getJointPositionGlobal(supR))

        del rd_body_ori[:]
        del rd_body_pos[:]
        # for body_idx in range(dartModel.getBodyNum()):

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

        # extraForcePos[0] = controlModel.getBodyPositionGlobal(selectedBody)
        extraForcePos[0] = controlModel.getBodyPositionGlobal(selectedBody) - 0.1 * np.array([viewer.objectInfoWnd.labelForceX.value(), 0., viewer.objectInfoWnd.labelForceZ.value()])


    def postFrameCallback_Always(frame):
        pass
        # if foot_viewer is not None:
        #     foot_viewer.foot_pressure_gl_window.refresh_foot_contact_info(frame, vpWorld, bodyIDsToCheck, mus, Ks, Ds)
        #     foot_viewer.foot_pressure_gl_window.goToFrame(frame)

    viewer.setPostFrameCallback_Always(postFrameCallback_Always)
    viewer.setSimulateCallback(simulateCallback)
    viewer.startTimer(1./render_fps)
    # viewer.play()
    viewer.show()

    # foot_viewer = FootWindow(viewer.x() + viewer.w() + 20, viewer.y(), 300, 500, 'foot contact modifier', controlModel)
    # foot_viewer.show()
    # foot_viewer.check_all_seg()
    # foot_viewer.check_h_r.value(False)
    # viewer.motionViewWnd.goToFrame(0)

    Fl.run()


main()
