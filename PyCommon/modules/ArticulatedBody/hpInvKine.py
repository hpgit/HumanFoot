import copy
import numpy as np
import numpy.linalg as npl
import ysJacobian as yjc
import Util.ysPythonEx as ype
import ArticulatedBody.ysReferencePoints as yrp
import ArticulatedBody.ysMomentum as ymt
import ArticulatedBody.ysControl as yct
import Math.mmMath as mm

import Simulator.csVpModel as cvm
import Simulator.csVpWorld as cvw

class numIkSolver:
    def __init__(self, _wcfg, _pose, _mcfg):
        self.world = cvw.VpWorld(_wcfg)
        self.model = cvm.VpMotionModel(self.world, _pose, _mcfg)

        self.bodyIdx = []
        self.localPos = []
        self.desPos = []
        self.desOri = []
        self.desPosMask = []

        self.pose = _pose.copy()

    def clear(self):
        del self.bodyIdx[:]
        del self.localPos[:]
        del self.desPos[:]
        del self.desOri[:]
        del self.desPosMask[:]

    def setInitPose(self, _pose):
        self.pose = _pose.copy()

    def addConstraints(self, _bodyIdx, _localPos, _despos, _desori, _desposMask=(True,True,True,True)):
        self.bodyIdx.append(_bodyIdx)
        self.localPos.append(_localPos)
        self.desPos.append(copy.deepcopy(_despos))
        self.desOri.append(copy.deepcopy(_desori))
        self.desPosMask.append(copy.deepcopy(_desposMask))

    def getConstJacobian(self, _model):
        Jc_IK = None
        DOFs = _model.getDOFs()
        jointPositions = _model.getJointPositionsGlobal()
        jointAxeses = _model.getDOFAxeses()

        for i in range(len(self.bodyIdx)):
            Jc = yjc.makeEmptyJacobian(DOFs, 1)
            contactJointMasks = [yjc.getLinkJointMask(self.pose.skeleton, self.bodyIdx[i])]
            bodyPos = _model.getBodyPositionGlobal(self.bodyIdx[i])
            bodyOri = _model.getBodyOrientationGlobal(self.bodyIdx[i])
            cPos = bodyPos + np.dot(bodyOri, self.localPos[i])
            yjc.computeJacobian2(Jc, DOFs, jointPositions, jointAxeses, [cPos], contactJointMasks)
            for j in range(3):
                if self.desPosMask[i][j]:
                    if Jc_IK is None:
                        Jc_IK = Jc[j, :].copy()
                    else:
                        Jc_IK = np.vstack((Jc_IK, Jc[j, :]))
            if self.desPosMask[i][3]:
                if Jc_IK is None:
                    Jc_IK = Jc[3:6, :].copy()
                else:
                    Jc_IK = np.vstack((Jc_IK, Jc[3:6, :]))

        return Jc_IK

    def solve(self, _model, desComPos, cmW = 10., posW = 1., oriW = 1.):
        IKModel = self.model

        totalDOF = _model.getTotalDOF()
        DOFs = _model.getDOFs()

        Jsys_IK = yjc.makeEmptyJacobian(DOFs, _model.getBodyNum())
        allLinkJointMasks = yjc.getAllLinkJointMasks(self.pose.skeleton)

        dth_IK = ype.makeNestedList(DOFs)

        # momentum matrix
        linkMasses = IKModel.getBodyMasses()
        totalMass = IKModel.getTotalMass()
        TO = ymt.make_TO(linkMasses)
        dTO = ymt.make_dTO(len(linkMasses))

        numItr = 100
        dt = .5
        threshold = 0.01

        for i in range(0, numItr):
            IKModel.update(self.pose)

            th_r_IK = self.pose.getDOFPositions()
            jointPositions_IK = self.pose.getJointPositionsGlobal()
            jointAxeses_IK = self.pose.getDOFAxeses()
            linkPositions_IK = IKModel.getBodyPositionsGlobal()
            linkInertias_IK = IKModel.getBodyInertiasGlobal()

            CM_IK = yrp.getCM(linkPositions_IK, linkMasses, totalMass)
            P_IK = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions_IK, CM_IK, linkInertias_IK)

            yjc.computeJacobian2(Jsys_IK, DOFs, jointPositions_IK, jointAxeses_IK, linkPositions_IK, allLinkJointMasks)

            J_IK, JAngCom_IK = np.vsplit(np.dot(P_IK, Jsys_IK), 2)
            dv_IK = cmW*(desComPos - CM_IK)

            Jc_IK = self.getConstJacobian(_model)
            if Jc_IK is not None:
                J_IK = np.vstack(( J_IK, Jc_IK ))

            for j in range(len(self.bodyIdx)):
                ori_IK = IKModel.getBodyOrientationGlobal(self.bodyIdx[j])
                pos_IK = IKModel.getBodyPositionGlobal(self.bodyIdx[j]) + np.dot(ori_IK, self.localPos[j])
                dv_IK_tmp = posW * (self.desPos[j]-pos_IK)
                for k in range(3):
                    if self.desPosMask[j][k]:
                        dv_IK = np.append(dv_IK, dv_IK_tmp[k])
                if self.desPosMask[j][3]:
                    dv_IK = np.append(dv_IK, oriW*mm.logSO3(self.desOri[j]*ori_IK.T))

            dth_IK_solve = npl.lstsq(J_IK, dv_IK)
            dth_IK_x = dth_IK_solve[0][:totalDOF]
            ype.nested(dth_IK_x, dth_IK)
            th_IK = yct.getIntegralDOF(th_r_IK, dth_IK, dt)
            self.pose.setDOFPositions(th_IK)

            if np.dot(dv_IK, dv_IK) < threshold:
                break

        self.model.update(self.pose)
