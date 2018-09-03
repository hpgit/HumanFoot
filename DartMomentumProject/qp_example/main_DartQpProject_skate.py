from fltk import *
import copy
import numpy as np

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')
if '../..' not in sys.path:
    sys.path.append('../..')
import PyCommon.modules.Math.mmMath as mm
import PyCommon.modules.Resource.ysMotionLoader as yf
import PyCommon.modules.Renderer.ysRenderer as yr
# import PyCommon.modules.Renderer.csVpRenderer as cvr
# import PyCommon.modules.Simulator.csVpWorld as cvw
# import PyCommon.modules.Simulator.csVpModel as cvm
import PyCommon.modules.GUI.ysSimpleViewer as ysv
import PyCommon.modules.Optimization.ysAnalyticConstrainedOpt as yac
import PyCommon.modules.ArticulatedBody.ysJacobian as yjc
import PyCommon.modules.Util.ysPythonEx as ype
import PyCommon.modules.ArticulatedBody.ysReferencePoints as yrp
import PyCommon.modules.ArticulatedBody.ysMomentum as ymt
import PyCommon.modules.ArticulatedBody.ysControl as yct
from PyCommon.modules.GUI import hpSimpleViewer as hsv
from PyCommon.modules.Simulator import hpDartQpSimulator as hqp

from DartMomentumProject.qp_example import mtOptimize as mot
from DartMomentumProject.qp_example import mtInitialize as mit

import pydart2 as pydart
from PyCommon.modules.Simulator import csDartModel as cdm

from PyCommon.modules.ArticulatedBody import hpInvKineDart as hikd

from DartMomentumProject.qp_example.pdcontroller import *

g_initFlag = 0
forceShowTime = 0


contactChangeCount = 0
contactChangeType = 0
contact = 0
maxContactChangeCount = 30

preFootCenter = [None]

DART_CONTACT_ON = False

LEG_FLEXIBLE = False


def main():
    np.set_printoptions(precision=4, linewidth=200)
    # np.set_printoptions(precision=4, linewidth=1000, threshold=np.inf)

    pydart.init()
    dartModel = cdm.DartModel(None, None, None, None, 'cart_pole_blade.skel')
    dartMotionModel = cdm.DartModel(None, None, None, None, 'cart_pole_blade.skel')

    footIdlist = list(dartMotionModel.skeleton.body('h_'+name).index_in_skeleton() for name in ['blade_left', 'blade_right'])
    up_vec_in_each_link = dict()
    for foot_id in footIdlist:
        up_vec_in_each_link[foot_id] = dartMotionModel.getBodyOrientationGlobal(foot_id)[1, :]

    pelvis_pos = dartMotionModel.skeleton.dof_indices((["j_pelvis_pos_x", 'j_pelvis_pos_y', 'j_pelvis_pos_z']))
    pelvis_x = dartMotionModel.skeleton.dof_indices((["j_pelvis_rot_x"]))
    pelvis = dartMotionModel.skeleton.dof_indices((["j_pelvis_rot_y", "j_pelvis_rot_z"]))
    upper_body = dartMotionModel.skeleton.dof_indices(["j_abdomen_1", "j_abdomen_2"])
    right_leg = dartMotionModel.skeleton.dof_indices(["j_thigh_right_x", "j_thigh_right_y", "j_thigh_right_z", "j_shin_right"])
    left_leg = dartMotionModel.skeleton.dof_indices(["j_thigh_left_x", "j_thigh_left_y", "j_thigh_left_z", "j_shin_left"])
    arms = dartMotionModel.skeleton.dof_indices(["j_bicep_left_x", "j_bicep_right_x"])
    foot = dartMotionModel.skeleton.dof_indices(["j_heel_left_1", "j_heel_left_2", "j_heel_right_1", "j_heel_right_2"])
    leg_y = dartMotionModel.skeleton.dof_indices(["j_thigh_right_y", "j_thigh_left_y"])


    INIT_ANGLE = 0.09

    s0q = np.zeros(dartMotionModel.skeleton.ndofs)
    s0q[pelvis_pos] = 0., .92, 0.
    # s0q[pelvis] = 0., -0.
    # s0q[upper_body] = 0.3, -0.
    s0q[right_leg] = -0., -0., 0.9, -1.5
    s0q[left_leg] = -INIT_ANGLE, 0., 0.0, 0.0
    s0q[foot] = 0., INIT_ANGLE, 0., 0.
    # s0q[right_leg] = -0., -0., 0.2, -.4
    # s0q[left_leg] = -0., 0., 0.2, -.4
    # s0q[foot] = 0.2, 0., 0.2, 0.
    # s0q[leg_y] = -0.785, 0.785
    s0q[arms] = 1.5, -1.5

    dartModel.set_q(s0q)
    dartMotionModel.set_q(s0q)

    frame_step_size = 1./40.
    stepsPerFrame = 25
    time_step = dartModel.world.time_step()

    # wcfg.lockingVel = 0.01
    # dartModel.initializeHybridDynamics()

    #controlToMotionOffset = (1.5, -0.02, 0)
    controlToMotionOffset = (0, 0, 2.0)
    dartModel.translateByOffset(controlToMotionOffset)

    totalDOF = dartModel.getTotalDOF()
    DOFs = dartModel.getDOFs()

    # parameter
    Kt = 25.
    Dt = 2.*(Kt**.5)

    Kl = 100.
    Dl = 2.*(Kt**.5)

    Kh = 100.
    Dh = 2.*(Kt**.5)

    Ks = 20000.
    Ds = 2.*(Kt**.5)

    Bt = 1.
    Bl = 0.1
    Bh = 0.13

    supL = dartModel.skeleton.body('h_blade_left').index_in_skeleton()
    supR = dartModel.skeleton.body('h_blade_right').index_in_skeleton()

    selectedBody = dartModel.skeleton.body('h_head').index_in_skeleton()

    # momentum matrix
    linkMasses = dartModel.getBodyMasses()
    print([body.name for body in dartModel.skeleton.bodynodes])
    print(linkMasses)
    totalMass = dartModel.getTotalMass()
    TO = ymt.make_TO(linkMasses)
    dTO = ymt.make_dTO(len(linkMasses))

    # optimization
    problem = yac.LSE(totalDOF, 12)
    #a_sup = (0,0,0, 0,0,0) #ori
    #a_sup = (0,0,0, 0,0,0) #L
    a_supL = (0,0,0, 0,0,0)
    a_supR = (0,0,0, 0,0,0)
    a_sup_2 = (0,0,0, 0,0,0, 0,0,0, 0,0,0)
    CP_old = [mm.v3(0.,0.,0.)]
    CP_des = [None]
    dCP_des = [np.zeros(3)]

    # penalty method
    # bodyIDsToCheck = range(dartModel.getBodyNum())
    bodyIDsToCheck = [supL, supR]
    #mus = [1.]*len(bodyIDsToCheck)
    mus = [.5]*len(bodyIDsToCheck)

    # flat data structure
    # ddth_des_flat = ype.makeFlatList(totalDOF)
    # dth_flat = ype.makeFlatList(totalDOF)
    # ddth_sol = ype.makeNestedList(DOFs)

    config = dict()
    config['weightMap'] = {'j_scapula_left':.2, 'j_bicep_left':.2, 'j_forearm_left':.2, 'j_hand_left':.2,
                           'j_scapula_right':.2, 'j_bicep_right':.2, 'j_forearm_right':.2, 'j_hand_right':.2,
                         'j_abdomen':.6, 'j_spine':.6, 'j_head':.6, 'j_heel_right':.2, 'j_heel_left':.2, 'j_pelvis':0.5,
                         'j_thigh_left':5., 'j_shin_left':.5, 'j_thigh_right':5., 'j_shin_right':.5}


    # viewer
    rd_footCenter = [None]
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


    # viewer = ysv.SimpleViewer()
    viewer = hsv.hpSimpleViewer(viewForceWnd=False)
    viewer.setMaxFrame(1000)
    #viewer.record(False)
    # viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    # viewer.doc.addObject('motion', motion)
    # viewer.doc.addRenderer('motionModel', cvr.VpModelRenderer(motionModel, (150,150,255), yr.POLYGON_FILL))
    # viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    #viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL))

    viewer.doc.addRenderer('motionModel', yr.DartRenderer(dartMotionModel.world, (255,240,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('controlModel', yr.DartRenderer(dartModel.world, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('rd_footCenter', yr.PointsRenderer(rd_footCenter))
    viewer.doc.addRenderer('rd_CM_plane', yr.PointsRenderer(rd_CM_plane, (255,255,0)))
    viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (0,255,0)))
    viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (255,0,255)))
    viewer.doc.addRenderer('rd_dL_des_plane', yr.VectorsRenderer(rd_dL_des_plane, rd_CM, (255,255,0)))
    viewer.doc.addRenderer('rd_dH_des', yr.VectorsRenderer(rd_dH_des, rd_CM, (0,255,0)))
    #viewer.doc.addRenderer('rd_grf_des', yr.ForcesRenderer(rd_grf_des, rd_CP_des, (0,255,0), .001))
    viewer.doc.addRenderer('rd_CF', yr.VectorsRenderer(rd_CF, rd_CF_pos, (255,0,0)))

    viewer.doc.addRenderer('extraForce', yr.VectorsRenderer(rd_exf_des, extraForcePos, (0,255,0)))
    viewer.doc.addRenderer('extraForceEnable', yr.VectorsRenderer(rd_exfen_des, extraForcePos, (255,0,0)))

    #viewer.doc.addRenderer('right_foot_oriX', yr.VectorsRenderer(rightFootVectorX, rightFootPos, (255,0,0)))
    #viewer.doc.addRenderer('right_foot_oriY', yr.VectorsRenderer(rightFootVectorY, rightFootPos, (0,255,0)))
    #viewer.doc.addRenderer('right_foot_oriZ', yr.VectorsRenderer(rightFootVectorZ, rightFootPos, (0,0,255)))

    #viewer.doc.addRenderer('right_oriX', yr.VectorsRenderer(rightVectorX, rightPos, (255,0,0)))
    #viewer.doc.addRenderer('right_oriY', yr.VectorsRenderer(rightVectorY, rightPos, (0,255,0)))
    #viewer.doc.addRenderer('right_oriZ', yr.VectorsRenderer(rightVectorZ, rightPos, (0,0,255)))

    #success!!
    #initKt = 50
    #initKl = 10.1
    #initKh = 3.1

    #initBl = .1
    #initBh = .1
    #initSupKt = 21.6

    #initFm = 100.0

    #success!! -- 2015.2.12. double stance
    #initKt = 50
    #initKl = 37.1
    #initKh = 41.8

    #initBl = .1
    #initBh = .13
    #initSupKt = 21.6

    #initFm = 165.0

    #single stance
    #initKt = 25
    #initKl = 80.1
    #initKh = 10.8

    #initBl = .1
    #initBh = .13
    #initSupKt = 21.6

    #initFm = 50.0

    #single stance -> double stance
    #initKt = 25
    #initKl = 60.
    #initKh = 20.

    #initBl = .1
    #initBh = .13
    #initSupKt = 21.6

    #initFm = 50.0

    initKt = 25.
    # initKl = 11.
    # initKh = 22.
    initKl = 100.
    initKh = 100.

    initBl = .1
    initBh = .13
    initSupKt = 17.
    # initSupKt = 2.5

    initFm = 50.0

    initComX = 0.
    initComY = 0.
    initComZ = 0.

    viewer.objectInfoWnd.add1DSlider("Kt", 0., 300., 1., initKt)
    viewer.objectInfoWnd.add1DSlider("Kl", 0., 300., 1., initKl)
    viewer.objectInfoWnd.add1DSlider("Kh", 0., 300., 1., initKh)
    viewer.objectInfoWnd.add1DSlider("Bl", 0., 1., .001, initBl)
    viewer.objectInfoWnd.add1DSlider("Bh", 0., 1., .001, initBh)
    viewer.objectInfoWnd.add1DSlider("SupKt", 0., 100., 0.1, initSupKt)
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

    #self.sliderFm = Fl_Hor_Nice_Slider(10, 42+offset*6, 250, 10)

    pdcontroller = PDController(dartModel, dartModel.skeleton, dartModel.world.time_step(), Kt, Dt)

    def getParamVal(paramname):
        return viewer.objectInfoWnd.getVal(paramname)
    def getParamVals(paramnames):
        return (getParamVal(name) for name in paramnames)

    ik_solver = hikd.numIkSolver(dartMotionModel)

    body_num = dartModel.getBodyNum()
    # dJsys = np.zeros((6*body_num, totalDOF))
    # dJsupL = np.zeros((6, totalDOF))
    # dJsupR = np.zeros((6, totalDOF))
    # Jpre = [np.zeros((6*body_num, totalDOF)), np.zeros((6, totalDOF)), np.zeros((6, totalDOF))]

    ###################################
    #simulate
    ###################################
    bodyIDs, contactPositions, contactPositionLocals, contactForces = [], [], [], []

    def simulateCallback(frame):
        # print()
        # print(dartModel.getJointVelocityGlobal(0))
        # print(dartModel.getDOFVelocities()[0])
        # print(dartModel.get_dq()[:6])
        # dartMotionModel.update(motion[frame])

        global g_initFlag
        global forceShowTime

        global preFootCenter
        global maxContactChangeCount
        global contactChangeCount
        global contact
        global contactChangeType
        # print('contactstate:', contact, contactChangeCount)

        Kt, Kl, Kh, Bl, Bh, kt_sup = getParamVals(['Kt', 'Kl', 'Kh', 'Bl', 'Bh', 'SupKt'])
        Dt = 2.*(Kt**.5)
        Dl = (Kl**.5)
        Dh = (Kh**.5)
        dt_sup = 2.*(kt_sup**.5)
        # Dt = .2*(Kt**.5)
        # Dl = .2*(Kl**.5)
        # Dh = .2*(Kh**.5)
        # dt_sup = .2*(kt_sup**.5)

        pdcontroller.setKpKd(Kt, Dt)

        footHeight = dartModel.getBody(supL).shapenodes[0].shape.size()[1]/2.

        doubleTosingleOffset = 0.15
        singleTodoubleOffset = 0.30

        com_offset_x, com_offset_y, com_offset_z = getParamVals(['com X offset', 'com Y offset', 'com Z offset'])
        footOffset = np.array((com_offset_x, com_offset_y, com_offset_z))

        # tracking
        # th_r = motion.getDOFPositions(frame)
        th_r = dartMotionModel.getDOFPositions()
        th = dartModel.getDOFPositions()
        th_r_flat = dartMotionModel.get_q()
        # dth_r = motion.getDOFVelocities(frame)
        # dth = dartModel.getDOFVelocities()
        # ddth_r = motion.getDOFAccelerations(frame)
        # ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)
        dth_flat = dartModel.get_dq()
        # dth_flat = np.concatenate(dth)
        # ddth_des_flat = pdcontroller.compute(dartMotionModel.get_q())
        # ddth_des_flat = pdcontroller.compute(th_r)
        ddth_des_flat = pdcontroller.compute_flat(th_r_flat)

        # ype.flatten(ddth_des, ddth_des_flat)
        # ype.flatten(dth, dth_flat)

        print(dartModel.skeleton.get_spd_tau(th_r_flat, Kt, Dt))

        #################################################
        # jacobian
        #################################################

        footOriL = dartModel.getJointOrientationGlobal(supL)
        footOriR = dartModel.getJointOrientationGlobal(supR)

        footCenterL = dartModel.getBodyPositionGlobal(supL)
        footCenterR = dartModel.getBodyPositionGlobal(supR)
        footBodyOriL = dartModel.getBodyOrientationGlobal(supL)
        footBodyOriR = dartModel.getBodyOrientationGlobal(supR)
        footBodyVelL = dartModel.getBodyVelocityGlobal(supL)
        footBodyVelR = dartModel.getBodyVelocityGlobal(supR)
        footBodyAngVelL = dartModel.getBodyAngVelocityGlobal(supL)
        footBodyAngVelR = dartModel.getBodyAngVelocityGlobal(supR)

        refFootL = dartMotionModel.getBodyPositionGlobal(supL)
        refFootR = dartMotionModel.getBodyPositionGlobal(supR)
        # refFootAngVelL = motion.getJointAngVelocityGlobal(supL, frame)
        # refFootAngVelR = motion.getJointAngVelocityGlobal(supR, frame)
        refFootAngVelL = np.zeros(3)
        refFootAngVelR = np.zeros(3)

        refFootVelR = np.zeros(3)
        refFootVelL = np.zeros(3)

        contactR = 1
        contactL = 1
        if refFootVelR[1] < 0 and refFootVelR[1]*frame_step_size + refFootR[1] > singleTodoubleOffset:
            contactR = 0
        if refFootVelL[1] < 0 and refFootVelL[1]*frame_step_size + refFootL[1] > singleTodoubleOffset:
            contactL = 0
        if refFootVelR[1] > 0 and refFootVelR[1]*frame_step_size + refFootR[1] > doubleTosingleOffset:
            contactR = 0
        if refFootVelL[1] > 0 and refFootVelL[1]*frame_step_size + refFootL[1] > doubleTosingleOffset:
            contactL = 0
        # contactR = 0

        # contMotionOffset = th[0][0] - th_r[0][0]
        # contMotionOffset = dartModel.getBodyPositionGlobal(0) - dartMotionModel.getBodyPositionGlobal(0)
        contMotionOffset = controlToMotionOffset

        linkPositions = dartModel.getBodyPositionsGlobal()
        linkVelocities = dartModel.getBodyVelocitiesGlobal()
        linkAngVelocities = dartModel.getBodyAngVelocitiesGlobal()
        linkInertias = dartModel.getBodyInertiasGlobal()

        CM = dartModel.skeleton.com()
        dCM = dartModel.skeleton.com_velocity()
        CM_plane = copy.copy(CM)
        CM_plane[1]=0.
        dCM_plane = copy.copy(dCM)
        dCM_plane[1]=0.

        P = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions, CM, linkInertias)
        dP = ymt.getPureInertiaMatrixDerivative(dTO, linkMasses, linkVelocities, dCM, linkAngVelocities, linkInertias)

        #calculate contact state
        #if g_initFlag == 1 and contact == 1 and refFootR[1] < doubleTosingleOffset and footCenterR[1] < 0.08:
        if g_initFlag == 1:
            #contact state
            # 0: flying 1: right only 2: left only 3: double
            #if contact == 2 and refFootR[1] < doubleTosingleOffset:
            if contact == 2 and contactR==1:
                contact = 3
                maxContactChangeCount+=30
                contactChangeCount += maxContactChangeCount
                contactChangeType = 'StoD'

            #elif contact == 3 and refFootL[1] < doubleTosingleOffset:
            elif contact == 1 and contactL==1:
                contact = 3
                maxContactChangeCount+=30
                contactChangeCount += maxContactChangeCount
                contactChangeType = 'StoD'

            #elif contact == 3 and refFootR[1] > doubleTosingleOffset:
            elif contact == 3 and contactR == 0:
                contact = 2
                contactChangeCount += maxContactChangeCount
                contactChangeType = 'DtoS'

            #elif contact == 3 and refFootL[1] > doubleTosingleOffset:
            elif contact == 3 and contactL == 0:
                contact = 1
                contactChangeCount += maxContactChangeCount
                contactChangeType = 'DtoS'

            else:
                contact = 0
                #if refFootR[1] < doubleTosingleOffset:
                if contactR == 1:
                    contact +=1
                #if refFootL[1] < doubleTosingleOffset:
                if contactL == 1:
                    contact +=2

        #initialization
        if g_initFlag == 0:
            softConstPoint = footCenterR.copy()

            footCenter = footCenterL + (footCenterR - footCenterL)/2.0
            footCenter[1] = 0.
            preFootCenter = footCenter.copy()
            #footToBodyFootRotL = np.dot(np.transpose(footOriL), footBodyOriL)
            #footToBodyFootRotR = np.dot(np.transpose(footOriR), footBodyOriR)

            # if refFootR[1] < doubleTosingleOffset:
            #     contact +=1
            # if refFootL[1] < doubleTosingleOffset:
            #     contact +=2
            if refFootR[1] < footHeight:
                contact +=1
            if refFootL[1] < footHeight:
                contact +=2

            g_initFlag = 1

        contact = 2
        # contact = 1 + 2

        # calculate jacobian
        body_num = dartModel.getBodyNum()
        Jsys = np.zeros((6*body_num, totalDOF))
        dJsys = np.zeros((6*body_num, totalDOF))
        for i in range(dartModel.getBodyNum()):
            Jsys[6*i:6*i+6, :] = dartModel.getBody(i).world_jacobian()[range(-3, 3), :]
            dJsys[6*i:6*i+6, :] = dartModel.getBody(i).world_jacobian_classic_deriv()[range(-3, 3), :]

        JsupL = dartModel.getBody(supL).world_jacobian()[range(-3, 3), :]
        dJsupL = dartModel.getBody(supL).world_jacobian_classic_deriv()[range(-3, 3), :]

        JsupR = dartModel.getBody(supR).world_jacobian()[range(-3, 3), :]
        dJsupR = dartModel.getBody(supR).world_jacobian_classic_deriv()[range(-3, 3), :]

        # calculate footCenter
        footCenter = .5 * (footCenterL + footCenterR) + footOffset
        if contact == 2:
            footCenter = footCenterL.copy() + footOffset
        if contact == 1:
            footCenter = footCenterR.copy() + footOffset
        footCenter[1] = 0.
        footCenter[0] += 0.02

        preFootCenter = footCenter.copy()

        # linear momentum
        # CM_ref_plane = footCenter.copy()
        # CM_ref_plane += np.array([0., 0.9, 0.])
        # dL_des_plane = Kl*totalMass*(CM_ref_plane - CM_plane) - Dl*totalMass*dCM_plane
        # dL_des_plane[1] = 0.

        kl = np.diagflat([Kl*5., Kl, Kl*5.])
        dl = np.diagflat([2.2*Dl, Dl, 2.2*Dl])

        CM_ref = footCenter.copy()
        CM_ref[1] = dartMotionModel.getCOM()[1] - 0.1
        # CM_ref += np.array((0., com_offset_y, 0.))
        # dL_des_plane = Kl*totalMass*(CM_ref - CM) - Dl*totalMass*dCM
        dL_des_plane = kl.dot(totalMass*(CM_ref - CM)) - dl.dot(totalMass*dCM)

        # angular momentum
        CP_ref = footCenter

        CP = yrp.getCP(contactPositions, contactForces)
        if CP_old[0] is None or CP is None:
            dCP = None
        else:
            dCP = (CP - CP_old[0])/frame_step_size
        CP_old[0] = CP

        CP_des[0] = None
        # if CP_des[0] is None:
        #     CP_des[0] = footCenter

        if CP is not None and dCP is not None:
            ddCP_des = Kh*(CP_ref - CP) - Dh*(dCP)
            CP_des[0] = CP + dCP * frame_step_size + .5 * ddCP_des*(frame_step_size**2)
            dH_des = np.cross(CP_des[0] - CM, dL_des_plane - totalMass*mm.s2v(dartModel.world.gravity()))
            # dH_des = np.cross(footCenter - CM, dL_des_plane - totalMass*mm.s2v(dartModel.world.gravity()))
            # H = np.dot(P, np.dot(Jsys, dth_flat))
            # dH_des = -Kh * H[3:]
        else:
            dH_des = None

        # set up equality constraint
        a_oriL = mm.logSO3(mm.getSO3FromVectors(np.dot(footBodyOriL, np.array([0,1,0])), np.array([0,1,0])))
        a_oriR = mm.logSO3(mm.getSO3FromVectors(np.dot(footBodyOriR, np.array([0,1,0])), np.array([0,1,0])))

        footErrorL = refFootL.copy()
        footErrorL[1] = dartModel.getBody(supL).shapenodes[0].shape.size()[1]/2.
        footErrorL += -footCenterL + contMotionOffset

        footErrorR = refFootR.copy()
        footErrorR[1] = dartModel.getBody(supR).shapenodes[0].shape.size()[1]/2.
        footErrorR += -footCenterR + contMotionOffset

        a_supL = np.append(kt_sup*footErrorL + dt_sup*(refFootVelL - footBodyVelL), kt_sup*a_oriL+dt_sup*(refFootAngVelL-footBodyAngVelL))
        a_supR = np.append(kt_sup*footErrorR + dt_sup*(refFootVelR - footBodyVelR), kt_sup*a_oriR+dt_sup*(refFootAngVelR-footBodyAngVelR))

        # momentum matrix
        RS = np.dot(P, Jsys)
        R, S = np.vsplit(RS, 2)

        rs = np.dot((np.dot(dP, Jsys) + np.dot(P, dJsys)), dth_flat)
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

        w = mot.getTrackingWeightDart(DOFs, dartModel.skeleton, config['weightMap'])

        mot.addTrackingTerms(problem, totalDOF, Bt, w, ddth_des_flat)
        mot.addLinearTerms(problem, totalDOF, Bl, dL_des_plane, R, r_bias)
        if dH_des is not None:
            mot.addAngularTerms(problem, totalDOF, Bh, dH_des, S, s_bias)

            if contact & 1:
                mot.addConstraint(problem, totalDOF, JsupR, dJsupR, dth_flat, a_supR)
            if contact & 2:
                mot.addConstraint(problem, totalDOF, JsupL, dJsupL, dth_flat, a_supL)

        if contactChangeCount >0:
            contactChangeCount -= 1
            if contactChangeCount == 0:
                maxContactChangeCount = 30
                contactChangeType = 0

        r = problem.solve()
        problem.clear()
        # ype.nested(r['x'], ddth_sol)
        ddth_sol = np.asarray(r['x'])
        # ddth_sol[:6] = np.zeros(6)
        if dH_des is None:
            ddth_sol = ddth_des_flat

        rootPos[0] = dartModel.getBodyPositionGlobal(selectedBody)
        localPos = [[0, 0, 0]]
        inv_h = 1./time_step

        _bodyIDs, _contactPositions, _contactPositionLocals, _contactForces = [], [], [], []
        for iii in range(stepsPerFrame):
            _ddq, _tau, _bodyIDs, _contactPositions, _contactPositionLocals, _contactForces = hqp.calc_QP(dartModel.skeleton, ddth_sol, inv_h)
            # _ddq, _tau, _bodyIDs, _contactPositions, _contactPositionLocals, _contactForces = hqp.calc_QP(dartModel.skeleton, ddth_des_flat, inv_h)
            # print(frame, i, tau)
            dartModel.applyPenaltyForce(_bodyIDs, _contactPositionLocals, _contactForces)

            dartModel.skeleton.set_forces(_tau)

            if forceShowTime > viewer.objectInfoWnd.labelForceDur.value():
                forceShowTime = 0
                viewer_ResetForceState()

            forceforce = np.array([viewer.objectInfoWnd.labelForceX.value(), viewer.objectInfoWnd.labelForceY.value(), viewer.objectInfoWnd.labelForceZ.value()])
            extraForce[0] = getParamVal('Fm') * mm.normalize2(forceforce)
            if viewer_GetForceState():
                forceShowTime += time_step
                dartModel.applyPenaltyForce(selectedBodyId, localPos, extraForce)

            dartModel.step()

        del bodyIDs[:]
        del contactPositions[:]
        del contactPositions[:]
        del contactPositionLocals[:]
        del contactForces[:]
        bodyIDs.extend(_bodyIDs)
        contactPositions.extend(_contactPositions)
        contactPositionLocals.extend(_contactPositionLocals)
        contactForces.extend(_contactForces)

        # rendering
        rightFootVectorX[0] = np.dot(footOriL, np.array([.1, 0, 0]))
        rightFootVectorY[0] = np.dot(footOriL, np.array([0, .1, 0]))
        rightFootVectorZ[0] = np.dot(footOriL, np.array([0,  0,.1]))
        rightFootPos[0] = footCenterL

        rightVectorX[0] = np.dot(footBodyOriL, np.array([.1,0,0]))
        rightVectorY[0] = np.dot(footBodyOriL, np.array([0,.1,0]))
        rightVectorZ[0] = np.dot(footBodyOriL, np.array([0,0,.1]))
        rightPos[0] = footCenterL + np.array([.1,0,0])

        rd_footCenter[0] = footCenter
        rd_footCenterL[0] = footCenterL
        rd_footCenterR[0] = footCenterR

        rd_CM[0] = CM

        rd_CM_plane[0] = CM.copy()
        rd_CM_plane[0][1] = 0.

        if CP is not None and dCP is not None:
            rd_CP[0] = CP
            rd_CP_des[0] = CP_des[0]

            rd_dL_des_plane[0] = [dL_des_plane[0]/100, dL_des_plane[1]/100, dL_des_plane[2]/100]
            rd_dH_des[0] = dH_des

            rd_grf_des[0] = dL_des_plane - totalMass*mm.s2v(dartModel.world.gravity())

        rd_root_des[0] = rootPos[0]

        del rd_CF[:]
        del rd_CF_pos[:]
        for i in range(len(contactPositions)):
            rd_CF.append( contactForces[i]/100)
            rd_CF_pos.append(contactPositions[i].copy())

        if viewer_GetForceState():
            rd_exfen_des[0] = [extraForce[0][0]/100, extraForce[0][1]/100, extraForce[0][2]/100]
            rd_exf_des[0] = [0,0,0]
        else:
            rd_exf_des[0] = [extraForce[0][0]/100, extraForce[0][1]/100, extraForce[0][2]/100]
            rd_exfen_des[0] = [0,0,0]

        extraForcePos[0] = dartModel.getBodyPositionGlobal(selectedBody)

    viewer.setSimulateCallback(simulateCallback)

    viewer.startTimer(1/30.)
    viewer.show()

    Fl.run()

main()
