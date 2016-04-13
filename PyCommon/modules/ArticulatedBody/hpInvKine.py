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

    def addConstraints(self, _bodyIdx, _localPos, _despos, _desori, _desposMask):
        self.bodyIdx.append(_bodyIdx)
        self.localPos.append(_localPos)
        self.desPos.append(_despos.copy())
        self.desOri.append(_desori.copy())
        self.desPosMask.append(copy.deepcopy(_desposMask))

    def getConstJacobian(self):
        Jc_IK = None
        DOFs = self.model.getDOFs()
        jointPositions = self.model.getJointPositionsGlobal()
        jointAxeses = self.model.getDOFAxeses()

        for i in range(len(self.bodyIdx)):
            Jc = yjc.makeEmptyJacobian(DOFs, 1)
            contactJointMasks = [yjc.getLinkJointMask(self.pose.skeleton, self.bodyIdx[i])]
            bodyPos = self.model.getBodyPositionGlobal(self.bodyIdx[i])
            bodyOri = self.model.getBodyOrientationGlobal(self.bodyIdx[i])
            cPos = bodyPos + np.dot(bodyOri, self.localPos[i])
            yjc.computeJacobian2(Jc, DOFs, jointPositions, jointAxeses, [cPos], contactJointMasks)
            for j in self.desPosMask[i]:
                if Jc_IK is None:
                    Jc_IK = Jc[j, :].copy()
                else:
                    Jc_IK = np.vstack((Jc_IK, Jc[j, :]))

        return Jc_IK



    def solve(self, desComPos, cmW = 10., posW = 1., oriW = 1.):
        IKModel = self.model

        totalDOF = IKModel.getTotalDOF()
        DOFs = IKModel.getDOFs()

        Jsys_IK = yjc.makeEmptyJacobian(DOFs, IKModel.getBodyNum())
        allLinkJointMasks = yjc.getAllLinkJointMasks(self.pose.skeleton)

        dth_IK = ype.makeNestedList(DOFs)

        # momentum matrix
        linkMasses = IKModel.getBodyMasses()
        totalMass = IKModel.getTotalMass()
        TO = ymt.make_TO(linkMasses)
        dTO = ymt.make_dTO(len(linkMasses))

        numItr = 100
        dt = .5
        threshold = 0.1

        for i in range(0, numItr):
            jPart_IK = []
            print '----iter num', i
            IKModel.update(self.pose)

            th_r_IK = self.pose.getDOFPositions()
            jointPositions_IK = self.pose.getJointPositionsGlobal()
            jointAxeses_IK = self.pose.getDOFAxeses()
            linkPositions_IK = IKModel.getBodyPositionsGlobal()
            linkInertias_IK = IKModel.getBodyInertiasGlobal()

            CM_IK = yrp.getCM(linkPositions_IK, linkMasses, totalMass)
            print CM_IK
            P_IK = ymt.getPureInertiaMatrix(TO, linkMasses, linkPositions_IK, CM_IK, linkInertias_IK)

            yjc.computeJacobian2(Jsys_IK, DOFs, jointPositions_IK, jointAxeses_IK, linkPositions_IK, allLinkJointMasks)

            J_IK, JAngCom_IK = np.vsplit(np.dot(P_IK, Jsys_IK), 2)
            dv_IK = cmW*(desComPos - CM_IK)

            J_IK = np.vstack(( J_IK, self.getConstJacobian() ))

            for j in range(len(self.bodyIdx)):
                pos_IK = IKModel.getBodyPositionGlobal(self.bodyIdx[j])
                dv_IK = np.append(dv_IK, posW*(self.desPos[j] - pos_IK))
                ori_IK = IKModel.getBodyOrientationGlobal(self.bodyIdx[j])
                dv_IK = np.append(dv_IK, oriW*mm.logSO3(self.desOri[j]*ori_IK.T))

            dth_IK_solve = npl.lstsq(J_IK, dv_IK)
            dth_IK_x = dth_IK_solve[0][:totalDOF]
            ype.nested(dth_IK_x, dth_IK)
            th_IK = yct.getIntegralDOF(th_r_IK, dth_IK, dt)
            self.pose.setDOFPositions(th_IK)

            if np.dot(dv_IK, dv_IK) < threshold:
                break

        self.model.update(self.pose)
