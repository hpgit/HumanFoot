from fltk import *
import copy
import numpy as np

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

import mtOptimize as mot
import mtInitialize as mit

from PyCommon.modules.ArticulatedBody import hpFootIK as hfi
from scipy.spatial import ConvexHull

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


def main():
    np.set_printoptions(precision=4, linewidth=200)

    motion, mcfg, wcfg, stepsPerFrame, config, frame_rate = mit.create_biped()
    # motion, mcfg, wcfg, stepsPerFrame, config = mit.create_jump_biped()

    vpWorld = cvw.VpWorld(wcfg)
    motionModel = cvm.VpMotionModel(vpWorld, motion[0], mcfg)
    controlModel = cvm.VpControlModel(vpWorld, motion[0], mcfg)
    vpWorld.initialize()
    controlModel.initializeHybridDynamics()

    #controlToMotionOffset = (1.5, -0.02, 0)
    controlToMotionOffset = (1.5, 0, 0)
    controlModel.translateByOffset(controlToMotionOffset)

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
        print(joint_idx, joint_name)
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
                left_foot_dofs.extend(foot_dofs_temp)
            elif 'Right' in joint_name:
                right_foot_dofs.extend(foot_dofs_temp)

    # parameter
    Kt = config['Kt']; Dt = config['Dt'] # tracking gain
    Kl = config['Kl']; Dl = config['Dl'] # linear balance gain
    Kh = config['Kh']; Dh = config['Dh'] # angular balance gain
    Ks = config['Ks']; Ds = config['Ds']  # penalty force spring gain

    Bt = config['Bt']
    Bl = config['Bl']
    Bh = config['Bh']

    supL = motion[0].skeleton.getJointIndex(config['supLink1'])
    supR = motion[0].skeleton.getJointIndex(config['supLink2'])

    selectedBody = motion[0].skeleton.getJointIndex(config['end'])
    constBody = motion[0].skeleton.getJointIndex('RightFoot')

    # jacobian
    JsupL = yjc.makeEmptyJacobian(DOFs, 1)
    dJsupL = JsupL.copy()
    JsupPreL = JsupL.copy()

    JsupR = yjc.makeEmptyJacobian(DOFs, 1)
    dJsupR = JsupR.copy()
    JsupPreR = JsupR.copy()

    Jconst = yjc.makeEmptyJacobian(DOFs, 1)
    dJconst = Jconst.copy()
    JconstPre = Jconst.copy()

    Jsys = yjc.makeEmptyJacobian(DOFs, controlModel.getBodyNum())
    dJsys = Jsys.copy()
    JsysPre = Jsys.copy()

    supLJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, supL)]
    supRJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, supR)]
    constJointMasks = [yjc.getLinkJointMask(motion[0].skeleton, constBody)]
    allLinkJointMasks = yjc.getAllLinkJointMasks(motion[0].skeleton)

    # momentum matrix
    linkMasses = controlModel.getBodyMasses()
    totalMass = controlModel.getTotalMass()
    TO = ymt.make_TO(linkMasses)
    dTO = ymt.make_dTO(len(linkMasses))

    # optimization
    problem = yac.LSE(totalDOF, 12)
    # a_sup = (0,0,0, 0,0,0) #ori
    # a_sup = (0,0,0, 0,0,0) #L
    a_supL = (0,0,0, 0,0,0)
    a_supR = (0,0,0, 0,0,0)
    a_sup_2 = (0,0,0, 0,0,0, 0,0,0, 0,0,0)
    CP_old = [mm.v3(0.,0.,0.)]

    # penalty method
    bodyIDsToCheck = range(vpWorld.getBodyNum())
    # mus = [1.]*len(bodyIDsToCheck)
    mus = [.5]*len(bodyIDsToCheck)

    # flat data structure
    ddth_des_flat = ype.makeFlatList(totalDOF)
    dth_flat = ype.makeFlatList(totalDOF)
    ddth_sol = ype.makeNestedList(DOFs)

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
    # viewer.record(False)
    # viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion, (0,255,255), yr.LINK_BONE))
    viewer.doc.addObject('motion', motion)
    viewer.doc.addRenderer('motionModel', yr.VpModelRenderer(motionModel, (150,150,255), yr.POLYGON_FILL))
    # viewer.doc.addRenderer('controlModel', cvr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_LINE))
    control_model_renderer = yr.VpModelRenderer(controlModel, (255,240,255), yr.POLYGON_FILL)
    viewer.doc.addRenderer('controlModel', control_model_renderer)
    control_model_renderer.body_colors[supL] = (255, 0, 0)
    control_model_renderer.body_colors[supR] = (255, 0, 0)
    viewer.doc.addRenderer('rd_footCenter', yr.PointsRenderer(rd_footCenter))
    viewer.doc.addRenderer('rd_CM_plane', yr.PointsRenderer(rd_CM_plane, (255,255,0)))
    viewer.doc.addRenderer('rd_CP', yr.PointsRenderer(rd_CP, (0,255,0)))
    viewer.doc.addRenderer('rd_CP_des', yr.PointsRenderer(rd_CP_des, (255,0,255)))
    viewer.doc.addRenderer('rd_dL_des_plane', yr.VectorsRenderer(rd_dL_des_plane, rd_CM, (255,255,0)))
    viewer.doc.addRenderer('rd_dH_des', yr.VectorsRenderer(rd_dH_des, rd_CM, (0,255,0)))
    # viewer.doc.addRenderer('rd_grf_des', yr.ForcesRenderer(rd_grf_des, rd_CP_des, (0,255,0), .001))
    viewer.doc.addRenderer('rd_CF', yr.VectorsRenderer(rd_CF, rd_CF_pos, (255,255,0)))

    viewer.doc.addRenderer('extraForce', yr.VectorsRenderer(rd_exf_des, extraForcePos, (0,255,0)))
    viewer.doc.addRenderer('extraForceEnable', yr.VectorsRenderer(rd_exfen_des, extraForcePos, (255,0,0)))

    # viewer.doc.addRenderer('right_foot_oriX', yr.VectorsRenderer(rightFootVectorX, rightFootPos, (255,0,0)))
    # viewer.doc.addRenderer('right_foot_oriY', yr.VectorsRenderer(rightFootVectorY, rightFootPos, (0,255,0)))
    # viewer.doc.addRenderer('right_foot_oriZ', yr.VectorsRenderer(rightFootVectorZ, rightFootPos, (0,0,255)))

    # viewer.doc.addRenderer('right_oriX', yr.VectorsRenderer(rightVectorX, rightPos, (255,0,0)))
    # viewer.doc.addRenderer('right_oriY', yr.VectorsRenderer(rightVectorY, rightPos, (0,255,0)))
    # viewer.doc.addRenderer('right_oriZ', yr.VectorsRenderer(rightVectorZ, rightPos, (0,0,255)))

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

    # self.sliderFm = Fl_Hor_Nice_Slider(10, 42+offset*6, 250, 10)

    def getParamVal(paramname):
        return viewer.objectInfoWnd.getVal(paramname)

    def getParamVals(paramnames):
        return (getParamVal(name) for name in paramnames)

    extendedFootName = ['Foot_foot_0_0', 'Foot_foot_0_1', 'Foot_foot_0_0_0', 'Foot_foot_0_1_0', 'Foot_foot_1_0']
    lIDdic = {'Left'+name:motion[0].skeleton.getJointIndex('Left'+name) for name in extendedFootName}
    rIDdic = {'Right'+name:motion[0].skeleton.getJointIndex('Right'+name) for name in extendedFootName}
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

    ###################################
    # simulate
    ###################################
    def simulateCallback(frame):
        motionModel.update(motion[frame])

        global g_initFlag
        global forceShowTime

        global JsysPre
        global JsupPreL
        global JsupPreR
        global JsupPre

        global JconstPre

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
        #doubleTosingleOffset = 0.09
        doubleTosingleVelOffset = 0.0

        # tracking
        th_r = motion.getDOFPositions(frame)
        th = controlModel.getDOFPositions()
        dth_r = motion.getDOFVelocities(frame)
        dth = controlModel.getDOFVelocities()
        ddth_r = motion.getDOFAccelerations(frame)
        ddth_des = yct.getDesiredDOFAccelerations(th_r, th, dth_r, dth, ddth_r, Kt, Dt)

        ype.flatten(ddth_des, ddth_des_flat)
        ype.flatten(dth, dth_flat)

        #################################################
        # jacobian
        #################################################

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
        # contactR = 0

        contMotionOffset = th[0][0] - th_r[0][0]

        linkPositions = controlModel.getBodyPositionsGlobal()
        linkVelocities = controlModel.getBodyVelocitiesGlobal()
        linkAngVelocities = controlModel.getBodyAngVelocitiesGlobal()
        linkInertias = controlModel.getBodyInertiasGlobal()

        jointPositions = controlModel.getJointPositionsGlobal()
        jointAxeses = controlModel.getDOFAxeses()

        CM = yrp.getCM(linkPositions, linkMasses, totalMass)
        dCM = yrp.getCM(linkVelocities, linkMasses, totalMass)
        CM_plane = copy.copy(CM); CM_plane[1]=0.
        dCM_plane = copy.copy(dCM); dCM_plane[1]=0.

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
            JsysPre = Jsys.copy()
            # JsupPreL = JsupL.copy()
            # JsupPreR = JsupR.copy()
            JconstPre = Jconst.copy()
            softConstPoint = footCenterR.copy()
            yjc.computeJacobian2(JsysPre, DOFs, jointPositions, jointAxeses, linkPositions, allLinkJointMasks)
            # yjc.computeJacobian2(JsupPreL, DOFs, jointPositions, jointAxeses, [footCenterL], supLJointMasks)
            # yjc.computeJacobian2(JsupPreR, DOFs, jointPositions, jointAxeses, [footCenterR], supRJointMasks)
            yjc.computeJacobian2(JconstPre, DOFs, jointPositions, jointAxeses, [softConstPoint], constJointMasks)

            footCenter = footCenterL + (footCenterR - footCenterL)/2.0
            footCenter[1] = 0.
            preFootCenter = footCenter.copy()
            #footToBodyFootRotL = np.dot(np.transpose(footOriL), footBodyOriL)
            #footToBodyFootRotR = np.dot(np.transpose(footOriR), footBodyOriR)

            if refFootR[1] < doubleTosingleOffset:
                contact += 1
            if refFootL[1] < doubleTosingleOffset:
                contact += 2

            g_initFlag = 1

        # calculate jacobian
        yjc.computeJacobian2(Jsys, DOFs, jointPositions, jointAxeses, linkPositions, allLinkJointMasks)
        # dJsys = (Jsys - JsysPre)/(1/30.)
        # JsysPre = Jsys.copy()
        yjc.computeJacobianDerivative2(dJsys, DOFs, jointPositions, jointAxeses, linkAngVelocities, linkPositions, allLinkJointMasks)

        JsupL, supLJointMasks = get_jacobianbase_and_masks(motion[0].skeleton, DOFs, supL)
        yjc.computeJacobian2(JsupL, DOFs, jointPositions, jointAxeses, [footCenterL], supLJointMasks)
        # dJsupL = (JsupL - JsupPreL)/(1/30.)
        # JsupPreL = JsupL.copy()
        yjc.computeJacobianDerivative2(dJsupL, DOFs, jointPositions, jointAxeses, linkAngVelocities, [footCenterL], supLJointMasks)

        JsupR, supRJointMasks = get_jacobianbase_and_masks(motion[0].skeleton, DOFs, supR)
        yjc.computeJacobian2(JsupR, DOFs, jointPositions, jointAxeses, [footCenterR], supRJointMasks)
        # dJsupR = (JsupR - JsupPreR)/(1/30.)
        # JsupPreR = JsupR.copy()
        yjc.computeJacobianDerivative2(dJsupR, DOFs, jointPositions, jointAxeses, linkAngVelocities, [footCenterR], supRJointMasks)

        # calculate footCenter
        footCenter = footCenterL + (footCenterR - footCenterL)/2.0
        #if refFootR[1] >doubleTosingleOffset:
        #if refFootR[1] > doubleTosingleOffset or footCenterR[1] > 0.08:
        #if contact == 1 or footCenterR[1] > 0.08:
        #if contact == 2 or footCenterR[1] > doubleTosingleOffset/2:
        if contact == 2:
            footCenter = footCenterL.copy()
        #elif contact == 1 or footCenterL[1] > doubleTosingleOffset/2:
        if contact == 1:
            footCenter = footCenterR.copy()
        footCenter[1] = 0.

        if contactChangeCount >0 and contactChangeType == 'StoD':
            #change footcenter gradually
            footCenter = preFootCenter + (maxContactChangeCount - contactChangeCount)*(footCenter-preFootCenter)/maxContactChangeCount

        preFootCenter = footCenter.copy()

        # linear momentum
        # TODO:
        # We should consider dCM_ref, shouldn't we?
        # add getBodyPositionGlobal and getBodyPositionsGlobal in csVpModel!
        # to do that, set joint velocities to vpModel
        CM_ref_plane = footCenter
        dL_des_plane = Kl*totalMass*(CM_ref_plane - CM_plane) - Dl*totalMass*dCM_plane
        # dL_des_plane[1] = 0.

        # angular momentum
        CP_ref = footCenter
        bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
        # bodyIDs, contactPositions, contactPositionLocals, contactForces, contactVelocities = vpWorld.calcManyPenaltyForce(0, bodyIDsToCheck, mus, Ks, Ds)
        CP = yrp.getCP(contactPositions, contactForces)
        if CP_old[0] is None or CP is None:
            dCP = None
        else:
            dCP = (CP - CP_old[0])/(1/30.)
        CP_old[0] = CP

        if CP is not None and dCP is not None:
            ddCP_des = Kh*(CP_ref - CP) - Dh*(dCP)
            CP_des = CP + dCP*(1/30.) + .5*ddCP_des*((1/30.)**2)
            dH_des = np.cross((CP_des - CM), (dL_des_plane + totalMass*mm.s2v(wcfg.gravity)))
            if contactChangeCount > 0:  # and contactChangeType == 'DtoS':
                dH_des *= (maxContactChangeCount - contactChangeCount)/(maxContactChangeCount)
        else:
            dH_des = None

        # set up equality constraint
        # left_foot_up_vec, right_foot_up_vec = hfi.get_foot_up_vector(motion[frame], footIdDic, None)
        left_foot_up_vec, right_foot_up_vec = hfi.get_foot_up_vector(controlModel, footIdDic, None)
        # print(left_foot_up_vec, right_foot_up_vec)
        # a_oriL = mm.logSO3(mm.getSO3FromVectors(np.dot(footBodyOriL, np.array([0,1,0])), np.array([0,1,0])))
        # a_oriR = mm.logSO3(mm.getSO3FromVectors(np.dot(footBodyOriR, np.array([0,1,0])), np.array([0,1,0])))
        a_oriL = mm.logSO3(mm.getSO3FromVectors(left_foot_up_vec, np.array([0,1,0])))
        a_oriR = mm.logSO3(mm.getSO3FromVectors(right_foot_up_vec, np.array([0,1,0])))

        a_supL = np.append(kt_sup*(refFootL - footCenterL + contMotionOffset) + dt_sup*(refFootVelL - footBodyVelL), kt_sup*a_oriL+dt_sup*(refFootAngVelL-footBodyAngVelL))
        a_supR = np.append(kt_sup*(refFootR - footCenterR + contMotionOffset) + dt_sup*(refFootVelR - footBodyVelR), kt_sup*a_oriR+dt_sup*(refFootAngVelR-footBodyAngVelR))

        if contactChangeCount > 0 and contactChangeType == 'DtoS':
            a_supL = np.append(kt_sup*(refFootL - footCenterL + contMotionOffset) + dt_sup*(refFootVelL - footBodyVelL), 4*kt_sup*a_oriL+2*dt_sup*(refFootAngVelL-footBodyAngVelL))
            a_supR = np.append(kt_sup*(refFootR - footCenterR + contMotionOffset) + dt_sup*(refFootVelR - footBodyVelR), 4*kt_sup*a_oriR+2*dt_sup*(refFootAngVelR-footBodyAngVelR))
        elif contactChangeCount > 0 and contactChangeType == 'StoD':
            linkt = (13.*contactChangeCount)/(maxContactChangeCount)+1.
            lindt = 2*(linkt**.5)
            angkt = (13.*contactChangeCount)/(maxContactChangeCount)+1.
            angdt = 2*(angkt**.5)
            a_supL = np.append(linkt*kt_sup*(refFootL - footCenterL + contMotionOffset) + lindt*dt_sup*(refFootVelL - footBodyVelL), angkt*kt_sup*a_oriL+angdt*dt_sup*(refFootAngVelL-footBodyAngVelL))
            a_supR = np.append(linkt*kt_sup*(refFootR - footCenterR + contMotionOffset) + lindt*dt_sup*(refFootVelR - footBodyVelR), angkt*kt_sup*a_oriR+angdt*dt_sup*(refFootAngVelR-footBodyAngVelR))

        # momentum matrix
        RS = np.dot(P, Jsys)
        R, S = np.vsplit(RS, 2)

        rs = np.dot((np.dot(dP, Jsys) + np.dot(P, dJsys)), dth_flat)
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
            mot.addAngularTerms(problem, totalDOF, Bh, dH_des, S, s_bias)

            if contact & 1:
                mot.addConstraint(problem, totalDOF, JsupR, dJsupR, dth_flat, a_supR)
            if contact & 2:
                mot.addConstraint(problem, totalDOF, JsupL, dJsupL, dth_flat, a_supL)

        if contactChangeCount > 0:
            contactChangeCount -= 1
            if contactChangeCount == 0:
                maxContactChangeCount = 30
                contactChangeType = 0

        r = problem.solve()
        problem.clear()
        ddth_sol_flat = np.asarray(r['x'])
        # ddth_sol_flat[foot_seg_dofs] = np.array(ddth_des_flat)[foot_seg_dofs]
        print(foot_seg_dofs)
        ype.nested(ddth_sol_flat, ddth_sol)

        rootPos[0] = controlModel.getBodyPositionGlobal(selectedBody)
        localPos = [[0, 0, 0]]

        for i in range(stepsPerFrame):
            # apply penalty force
            bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            # bodyIDs, contactPositions, contactPositionLocals, contactForces, contactVelocities = vpWorld.calcManyPenaltyForce(0, bodyIDsToCheck, mus, Ks, Ds)
            vpWorld.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)

            controlModel.setDOFAccelerations(ddth_sol)
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

        # rendering
        rightFootVectorX[0] = np.dot(footOriL, np.array([.1, 0, 0]))
        rightFootVectorY[0] = np.dot(footOriL, np.array([0, .1, 0]))
        rightFootVectorZ[0] = np.dot(footOriL, np.array([0, 0, .1]))
        rightFootPos[0] = footCenterL

        rightVectorX[0] = np.dot(footBodyOriL, np.array([.1, 0, 0]))
        rightVectorY[0] = np.dot(footBodyOriL, np.array([0, .1, 0]))
        rightVectorZ[0] = np.dot(footBodyOriL, np.array([0, 0, .1]))
        rightPos[0] = footCenterL + np.array([.1, 0, 0])

        rd_footCenter[0] = footCenter
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

            rd_grf_des[0] = dL_des_plane - totalMass*mm.s2v(wcfg.gravity)

        rd_root_des[0] = rootPos[0]

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

    viewer.setSimulateCallback(simulateCallback)

    viewer.startTimer(1/30.)
    viewer.show()

    Fl.run()


main()
