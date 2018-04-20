import copy
import numpy as np
import numpy.linalg as npl
# from PyCommon.modules.ArticulatedBody import ysJacobian as yjc
# from PyCommon.modules.Util import ysPythonEx as ype
# from PyCommon.modules.ArticulatedBody import ysReferencePoints as yrp
# from PyCommon.modules.ArticulatedBody import ysMomentum as ymt
# from PyCommon.modules.ArticulatedBody import ysControl as yct
from PyCommon.modules.Math import mmMath as mm

from PyCommon.modules.Simulator import csDartModel as cpm

class numIkSolver:
    # def __init__(self, _wcfg, _pose, _mcfg):
    def __init__(self, _model):
        """
        
        :type _wcfg: ypc.WorldConfig
        :type _pose: ym.JointPosture
        :type _mcfg: ypc.ModelConfig
        """
        self.model = _model # type: cpm.DartModel

        self.bodyIdx = [] # type: list[np.array]
        self.localPos = [] # type: list[np.array]
        self.desPos = [] # type: list[np.array]
        self.desOri = [] # type: list[np.array]
        self.desPosMask = []

        # self.pose = _pose.copy()

    def clear(self):
        del self.bodyIdx[:]
        del self.localPos[:]
        del self.desPos[:]
        del self.desOri[:]
        del self.desPosMask[:]

    def setInitPose(self, _pose):
        """
        :type _pose: ym.JointPosture
        """
        self.pose = _pose.copy()

    def addConstraints(self, _bodyIdx, _localPos, _despos, _desori, _desposMask=(True, True, True, True)):
        self.bodyIdx.append(_bodyIdx)
        self.localPos.append(_localPos)
        self.desPos.append(copy.deepcopy(_despos))
        self.desOri.append(copy.deepcopy(_desori))
        self.desPosMask.append(copy.deepcopy(_desposMask))

    def getConstJacobian(self, _model):
        """
        
        :type _model: cpm.DartModel
        :return: 
        """
        # Jc_IK = np.zeros((0, _model.getTotalDOF()))
        Jc_IK = _model.getCOMJacobian()
        for i in range(len(self.bodyIdx)):
            Jc_IK_tmp = _model.getBody(self.bodyIdx[i]).world_jacobian(self.localPos[i])
            for k in range(3):
                if self.desPosMask[i][k]:
                    Jc_IK = np.vstack((Jc_IK, Jc_IK_tmp[k+3:k+4, :]))
                    # Jc_IK = np.vstack((Jc_IK, Jc_IK_tmp[k:k+1, :]))
            if self.desPosMask[i][3]:
                Jc_IK = np.vstack((Jc_IK, Jc_IK_tmp[:3, :]))
            # Jc_IK = np.vstack((Jc_IK, _model.getBody(self.bodyIdx[i]).world_jacobian(self.localPos[i])[:3, :]))

        # TODO:
        # consider self.desPosMask

        return Jc_IK

    def solve(self, desComPos, cmW=1., posW=1., oriW=1.):
        """
        
        :type desComPos: np.array
        :type cmW: float
        :type posW: float
        :type oriW: float
        :return: 
        """
        numItr = 1000
        dt = .05
        threshold = 0.00001

        totalDOF = self.model.getTotalDOF()

        for i in range(numItr):
            Jc_IK = self.getConstJacobian(self.model)
            dv_IK = cmW * (desComPos - self.model.getCOM())
            for j in range(len(self.bodyIdx)):
                ori_IK = self.model.getBodyOrientationGlobal(self.bodyIdx[j])
                pos_IK = self.model.getBody(self.bodyIdx[j]).to_world(self.localPos[j])
                dv_IK_tmp = posW * (self.desPos[j]-pos_IK) # type: np.array
                for k in range(3):
                    if self.desPosMask[j][k]:
                        dv_IK = np.append(dv_IK, dv_IK_tmp[k])
                if self.desPosMask[j][3]:
                    dv_IK = np.append(dv_IK, oriW*mm.logSO3(np.dot(self.desOri[j], ori_IK.T)))

            dth_IK_solve = npl.lstsq(Jc_IK, dv_IK)
            dth_IK_x = dth_IK_solve[0][:totalDOF]
            self.model.skeleton.q += dt * dth_IK_x
            # ype.nested(dth_IK_x, dth_IK)
            # th_IK = yct.getIntegralDOF(th_r_IK, dth_IK, dt)
            # self.pose.setDOFPositions(th_IK)

            if np.dot(dv_IK, dv_IK) < threshold:
                break


class numPartIkSolver:
    pass
