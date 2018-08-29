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

import mtOptimize as mot
import mtInitialize as mit

import pydart2 as pydart
from PyCommon.modules.Simulator import csDartModel as cdm

from PyCommon.modules.ArticulatedBody import hpInvKineDart as hikd

from pdcontroller import *
from DartMomentumProject.qp_example.dart_jacobian_3dof import *
from PyCommon.modules.Simulator import hpDartQpSimulator as hqp

g_initFlag = 0
forceShowTime = 0


contactChangeCount = 0
contactChangeType = 0
contact = 0
maxContactChangeCount = 30

preFootCenter = [None]

DART_CONTACT_ON = False


def main():
    np.set_printoptions(precision=4, linewidth=200)
    # np.set_printoptions(precision=4, linewidth=1000, threshold=np.inf)

    #motion, mcfg, wcfg, stepsPerFrame, config = mit.create_vchain_5()
    motion, mcfg, wcfg, stepsPerFrame, config, frame_rate= mit.create_biped()
    #motion, mcfg, wcfg, stepsPerFrame, config = mit.create_jump_biped()


    frame_step_size = 1./frame_rate

    pydart.init()
    dartModel = cdm.DartModel(wcfg, motion[0], mcfg, DART_CONTACT_ON)
    dartMotionModel = cdm.DartModel(wcfg, motion[0], mcfg, DART_CONTACT_ON)

    # wcfg.lockingVel = 0.01
    time_step = dartModel.world.time_step()
    # dartModel.initializeForwardDynamics()
    # dartModel.initializeHybridDynamics()

    #controlToMotionOffset = (1.5, -0.02, 0)
    controlToMotionOffset = (1.5, 0, 0)
    # dartModel.translateByOffset(controlToMotionOffset)

    totalDOF = dartModel.getTotalDOF()
    DOFs = dartModel.getDOFs()

    # parameter
    Kt = config['Kt']; Dt = config['Dt'] # tracking gain
    Kl = config['Kl']; Dl = config['Dl'] # linear balance gain
    Kh = config['Kh']; Dh = config['Dh'] # angular balance gain
    Ks = config['Ks']; Ds = config['Ds']  # penalty force spring gain

    Bt = config['Bt']
    Bl = config['Bl']
    Bh = config['Bh']

    w = mot.getTrackingWeight(DOFs, motion[0].skeleton, config['weightMap'])

    supL =  motion[0].skeleton.getJointIndex(config['supLink1'])
    supR =  motion[0].skeleton.getJointIndex(config['supLink2'])

    selectedBody = motion[0].skeleton.getJointIndex(config['end'])
    constBody = motion[0].skeleton.getJointIndex('RightFoot')

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

    viewer = hsv.hpSimpleViewer(viewForceWnd=False)
    # viewer.record(False)
    # viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('motionModel', yr.DartModelRenderer(dartMotionModel, (150,150,255), yr.POLYGON_FILL))
    viewer.doc.addRenderer('controlModel', yr.DartRenderer(dartModel.world, (255,240,255), yr.POLYGON_FILL))
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

    viewer.doc.addRenderer('right_oriX', yr.VectorsRenderer(rightVectorX, rightPos, (255,0,0)))
    viewer.doc.addRenderer('right_oriY', yr.VectorsRenderer(rightVectorY, rightPos, (0,255,0)))
    viewer.doc.addRenderer('right_oriZ', yr.VectorsRenderer(rightVectorZ, rightPos, (0,0,255)))

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

    pdcontroller = PDController(dartModel, dartModel.skeleton, wcfg.timeStep, Kt, Dt)

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
    extendedFootName = ['Foot']
    lIDdic = {'Left'+name: motion[0].skeleton.getJointIndex('Left'+name) for name in extendedFootName}
    rIDdic = {'Right'+name: motion[0].skeleton.getJointIndex('Right'+name) for name in extendedFootName}

    lIDlist = [motion[0].skeleton.getJointIndex('Left'+name) for name in extendedFootName]
    rIDlist = [motion[0].skeleton.getJointIndex('Right'+name) for name in extendedFootName]

    bodyIDs, contactPositions, contactPositionLocals, contactForces = [], [], [], []

    ###################################
    # simulate
    ###################################
    def simulateCallback(frame):
        # print()
        # print(dartModel.getJointVelocityGlobal(0))
        # print(dartModel.getDOFVelocities()[0])
        # print(dartModel.get_dq()[:6])
        dartMotionModel.update(motion[frame])

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
        #doubleTosingleOffset = 0.09
        doubleTosingleVelOffset = 0.0

        com_offset_x, com_offset_y, com_offset_z = getParamVals(['com X offset', 'com Y offset', 'com Z offset'])
        footOffset = np.array((com_offset_x, com_offset_y, com_offset_z))
        des_com = dartMotionModel.getCOM() + footOffset

        footCenterL = dartMotionModel.getBodyPositionGlobal(supL)
        footCenterR = dartMotionModel.getBodyPositionGlobal(supR)
        footBodyOriL = dartMotionModel.getBodyOrientationGlobal(supL)
        footBodyOriR = dartMotionModel.getBodyOrientationGlobal(supR)

        torso_pos = dartMotionModel.getBodyPositionGlobal(4)
        torso_ori = dartMotionModel.getBodyOrientationGlobal(4)

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

        #################################################
        # jacobian
        #################################################
        contact_des_ids = [dartModel.skeleton.bodynode_index("LeftFoot"), dartModel.skeleton.bodynode_index("RightFoot")]
        contact_ids = list()  # temp idx for balancing
        contact_ids.extend(contact_des_ids)

        contact_joint_ori = list(map(dartModel.getJointOrientationGlobal, contact_ids))
        contact_joint_pos = list(map(dartModel.getJointPositionGlobal, contact_ids))
        contact_body_ori = list(map(dartModel.getBodyOrientationGlobal, contact_ids))
        contact_body_pos = list(map(dartModel.getBodyPositionGlobal, contact_ids))
        contact_body_vel = list(map(dartModel.getBodyVelocityGlobal, contact_ids))
        contact_body_angvel = list(map(dartModel.getBodyAngVelocityGlobal, contact_ids))

        ref_joint_ori = list(map(motion[frame].getJointOrientationGlobal, contact_ids))
        ref_joint_pos = list(map(motion[frame].getJointPositionGlobal, contact_ids))
        ref_joint_vel = [motion.getJointVelocityGlobal(joint_idx, frame) for joint_idx in contact_ids]
        ref_joint_angvel = [motion.getJointAngVelocityGlobal(joint_idx, frame) for joint_idx in contact_ids]
        ref_body_ori = list(map(dartMotionModel.getBodyOrientationGlobal, contact_ids))
        ref_body_pos = list(map(dartMotionModel.getBodyPositionGlobal, contact_ids))

        for idx in range(len(ref_body_pos)):
            ref_body_pos[1] = dartModel.skeleton.body("RightFoot").shapenodes[0].shape.size()[1]/2.
        # ref_body_vel = list(map(controlModel.getBodyVelocityGlobal, contact_ids))
        ref_body_angvel = [motion.getJointAngVelocityGlobal(joint_idx, frame) for joint_idx in contact_ids]
        ref_body_vel = [ref_joint_vel[i] + np.cross(ref_joint_angvel[i], ref_body_pos[i] - ref_joint_pos[i])
                        for i in range(len(ref_joint_vel))]

        is_contact = [1] * len(contact_ids)
        contact_right = len(set(contact_des_ids).intersection(rIDlist)) > 0
        contact_left = len(set(contact_des_ids).intersection(lIDlist)) > 0

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

        refFootJointVelR = motion.getJointVelocityGlobal(supR, frame)
        refFootJointAngVelR = motion.getJointAngVelocityGlobal(supR, frame)
        refFootJointR = motion.getJointPositionGlobal(supR, frame)
        # refFootVelR = refFootJointVelR + np.cross(refFootJointAngVelR, (refFootR-refFootJointR))
        refFootVelR = np.zeros(3)

        refFootJointVelL = motion.getJointVelocityGlobal(supL, frame)
        refFootJointAngVelL = motion.getJointAngVelocityGlobal(supL, frame)
        refFootJointL = motion.getJointPositionGlobal(supL, frame)
        # refFootVelL = refFootJointVelL + np.cross(refFootJointAngVelL, (refFootL-refFootJointL))
        refFootVelL = np.zeros(3)

        contactR = 1
        contactL = 1

        # contMotionOffset = th[0][0] - th_r[0][0]
        contMotionOffset = dartModel.getBodyPositionGlobal(0) - dartMotionModel.getBodyPositionGlobal(0)
        contMotionOffset = np.zeros(3)

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

        #calculate jacobian
        body_num = dartModel.getBodyNum()
        Jsys = np.zeros((6*body_num, totalDOF))
        dJsys = np.zeros((6*body_num, totalDOF))
        _, dJsysdq = compute_J_dJdq(dartModel.skeleton)
        # dJsys = np.zeros((6*body_num, totalDOF))
        for i in range(dartModel.getBodyNum()):
            Jsys[6*i:6*i+6, :] = dartModel.getBody(i).world_jacobian()[range(-3, 3), :]
            dJsys[6*i:6*i+6, :] = dartModel.getBody(i).world_jacobian_classic_deriv()[range(-3, 3), :]
        dJsysdq = np.dot(dJsys, dartModel.skeleton.dq)

        J_contacts = []  # type: list[np.ndarray]
        dJ_contacts = []  # type: list[np.ndarray]
        for contact_id in contact_ids:
            J_contacts.append(Jsys[6*contact_id:6*contact_id + 6, :])
            dJ_contacts.append(dJsysdq[6*contact_id:6*contact_id + 6])

        #calculate footCenter
        footCenter = .5 * (footCenterL + footCenterR) + footOffset
        if contact == 2:
            footCenter = footCenterL.copy() + footOffset
        #elif contact == 1 or footCenterL[1] > doubleTosingleOffset/2:
        if contact == 1:
            footCenter = footCenterR.copy() + footOffset
        footCenter[1] = 0.

        if contactChangeCount > 0 and contactChangeType == 'StoD':
            # change footcenter gradually
            footCenter = preFootCenter + (maxContactChangeCount - contactChangeCount)*(footCenter-preFootCenter)/maxContactChangeCount

        preFootCenter = footCenter.copy()

        # linear momentum
        # CM_ref_plane = footCenter
        # dL_des_plane = Kl*totalMass*(CM_ref_plane - CM_plane) - Dl*totalMass*dCM_plane
        # dL_des_plane[1] = 0.

        CM_ref_plane = footCenter
        CM_ref_plane[1] = dartMotionModel.skeleton.com()[1]
        dL_des_plane = Kl*totalMass*(CM_ref_plane - CM) - Dl*totalMass*dCM
        # dL_des_plane[1] = 0.

        # angular momentum
        CP_ref = footCenter

        CP = yrp.getCP(contactPositions, contactForces)
        if CP_old[0] is None or CP is None:
            dCP = None
        else:
            dCP = (CP - CP_old[0])/frame_step_size
        CP_old[0] = CP

        CP_des[0] = None

        if CP is not None and dCP is not None:
            ddCP_des = Kh*(CP_ref - CP) - Dh*(dCP)
            CP_des[0] = CP + dCP * frame_step_size + .5 * ddCP_des*(frame_step_size**2)
            dH_des = mm.cross(CP_des[0] - CM, dL_des_plane + totalMass*mm.s2v(wcfg.gravity))
        else:
            dH_des = None

        # set up equality constraint
        a_oris = list(map(mm.logSO3, [np.dot(np.dot(ref_body_ori[i], mm.getSO3FromVectors(np.dot(ref_body_ori[i], mm.unitY()), mm.unitY())), contact_body_ori[i].T) for i in range(len(contact_body_ori))]))
        KT_SUP = np.diag([kt_sup/10., kt_sup, kt_sup/10.])
        a_sups = [np.append(np.dot(KT_SUP, (ref_body_pos[i] - contact_body_pos[i] + contMotionOffset)) - dt_sup * contact_body_vel[i],
                            kt_sup*a_oris[i] - dt_sup * contact_body_angvel[i]) for i in range(len(a_oris))]
        print(a_sups)

        # momentum matrix
        RS = np.dot(P, Jsys)
        R, S = np.vsplit(RS, 2)

        # rs = np.dot((np.dot(dP, Jsys) + np.dot(P, dJsys)), dth_flat)
        rs = np.dot(dP, np.dot(Jsys, dth_flat)) + np.dot(P, dJsysdq)
        r_bias, s_bias = np.hsplit(rs, 2)

        #######################################################
        # optimization
        #######################################################
        #if contact == 2 and footCenterR[1] > doubleTosingleOffset/2:
        if contact == 2:
            config['weightMap']['RightUpLeg'] = .8
            config['weightMap']['RightLeg'] = .8
            config['weightMap']['RightFoot'] = .8
        else:
            config['weightMap']['RightUpLeg'] = .1
            config['weightMap']['RightLeg'] = .25
            config['weightMap']['RightFoot'] = .2

        #if contact == 1 and footCenterL[1] > doubleTosingleOffset/2:
        if contact == 1:
            config['weightMap']['LeftUpLeg'] = .8
            config['weightMap']['LeftLeg'] = .8
            config['weightMap']['LeftFoot'] = .8
        else:
            config['weightMap']['LeftUpLeg'] = .1
            config['weightMap']['LeftLeg'] = .25
            config['weightMap']['LeftFoot'] = .2

        w = mot.getTrackingWeight(DOFs, motion[0].skeleton, config['weightMap'])

        #if contact == 2:
            #mot.addSoftPointConstraintTerms(problem, totalDOF, Bsc, ddP_des1, Q1, q_bias1)
        mot.addTrackingTerms(problem, totalDOF, Bt, w, ddth_des_flat)
        mot.addLinearTerms(problem, totalDOF, Bl, dL_des_plane, R, r_bias)
        if dH_des is not None:
            mot.addAngularTerms(problem, totalDOF, Bh, dH_des, S, s_bias)

            if True:
                for c_idx in range(len(contact_ids)):
                    mot.addConstraint2(problem, totalDOF, J_contacts[c_idx], dJ_contacts[c_idx], a_sups[c_idx])

        if contactChangeCount > 0:
            contactChangeCount -= 1
            if contactChangeCount == 0:
                maxContactChangeCount = 30
                contactChangeType = 0

        r = problem.solve()
        problem.clear()
        # ype.nested(r['x'], ddth_sol)
        ddth_sol = np.asarray(r['x'])
        # ddth_sol[:6] = np.zeros(6)

        rootPos[0] = dartModel.getBodyPositionGlobal(selectedBody)
        localPos = [[0, 0, 0]]
        inv_h = 1./time_step

        _bodyIDs, _contactPositions, _contactPositionLocals, _contactForces = [], [], [], []
        for i in range(stepsPerFrame):
            # apply penalty force
            _ddq, _tau, _bodyIDs, _contactPositions, _contactPositionLocals, _contactForces = hqp.calc_QP(dartModel.skeleton, ddth_sol, inv_h)
            dartModel.applyPenaltyForce(_bodyIDs, _contactPositionLocals, _contactForces)
            dartModel.skeleton.set_forces(_tau)

            if forceShowTime > viewer.objectInfoWnd.labelForceDur.value():
                forceShowTime = 0
                viewer_ResetForceState()

            forceforce = np.array([viewer.objectInfoWnd.labelForceX.value(), viewer.objectInfoWnd.labelForceY.value(), viewer.objectInfoWnd.labelForceZ.value()])
            extraForce[0] = getParamVal('Fm') * mm.normalize2(forceforce)
            if viewer_GetForceState():
                forceShowTime += wcfg.timeStep
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

            rd_grf_des[0] = dL_des_plane - totalMass*mm.s2v(wcfg.gravity)

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
