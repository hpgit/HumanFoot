import copy
import numpy as np
import gym

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
import PyCommon.modules.ArticulatedBody.ysControl as yct
from PyCommon.modules.GUI import hpSimpleViewer as hsv

import mtOptimize as mot
import mtInitialize as mit

import pydart2 as pydart
from PyCommon.modules.Simulator import csDartModel as cdm

from PyCommon.modules.ArticulatedBody import hpInvKineDart as hikd

from pdcontroller import *

from baselines.ppo2 import ppo2

forceShowTime = 0

contactChangeCount = 0
contactChangeType = 0
contact = 0
maxContactChangeCount = 30

preFootCenter = [None]

DART_CONTACT_ON = True


def main(fm=50., fv=np.array([0., 0., 50.]), ts=2., te=2.1):
    np.set_printoptions(precision=4, linewidth=200)
    # np.set_printoptions(precision=4, linewidth=1000, threshold=np.inf)

    motion, mcfg, wcfg, stepsPerFrame, config, frame_rate= mit.create_biped()

    frame_step_size = 1./frame_rate

    pydart.init()
    dartModel = cdm.DartModel(wcfg, motion[0], mcfg, DART_CONTACT_ON)
    dartMotionModel = cdm.DartModel(wcfg, motion[0], mcfg, DART_CONTACT_ON)

    # wcfg.lockingVel = 0.01
    dartModel.initializeHybridDynamics()

    #controlToMotionOffset = (1.5, -0.02, 0)
    controlToMotionOffset = (1.5, 0, 0)
    dartModel.translateByOffset(controlToMotionOffset)

    totalDOF = dartModel.getTotalDOF()
    DOFs = dartModel.getDOFs()

    # parameter
    Kt = 25.
    Dt = 5.
    Ks = config['Ks']
    Ds = config['Ds']

    w = mot.getTrackingWeight(DOFs, motion[0].skeleton, config['weightMap'])

    supL = motion[0].skeleton.getJointIndex(config['supLink1'])
    supR = motion[0].skeleton.getJointIndex(config['supLink2'])

    selectedBody = motion[0].skeleton.getJointIndex(config['end'])
    constBody = motion[0].skeleton.getJointIndex('RightFoot')

    # momentum matrix
    linkMasses = dartModel.getBodyMasses()
    totalMass = dartModel.getTotalMass()
    dartModel.world.time()

    # penalty method
    # bodyIDsToCheck = range(dartModel.getBodyNum())
    bodyIDsToCheck = [supL, supR]
    #mus = [1.]*len(bodyIDsToCheck)
    mus = [.5]*len(bodyIDsToCheck)

    # viewer
    selectedBodyId = [selectedBody]
    extraForce = [None]
    extraForcePos = [None]

    pdcontroller = PDController(dartModel, dartModel.skeleton, wcfg.timeStep, Kt, Dt)

    ###################################
    #simulate
    ###################################
    def simulateCallback(frame):
        dartMotionModel.update(motion[frame])

        # tracking
        # th_r = motion.getDOFPositions(frame)
        th_r = dartMotionModel.getDOFPositions()
        th = dartModel.getDOFPositions()
        th_r_flat = dartMotionModel.get_q()
        dth_flat = dartModel.get_dq()
        ddth_des_flat = pdcontroller.compute_flat(th_r_flat)

        localPos = [[0, 0, 0]]
        for i in range(stepsPerFrame):
            # apply penalty force
            if not DART_CONTACT_ON:
                bodyIDs, contactPositions, contactPositionLocals, contactForces = dartModel.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
                dartModel.applyPenaltyForce(bodyIDs, contactPositionLocals, contactForces)

            # dartModel.skeleton.set_accelerations(ddth_sol)
            dartModel.skeleton.set_accelerations(ddth_des_flat)
            # dartModel.skeleton.set_forces(np.zeros(totalDOF))

            extraForce[0] = fm * fv
            if ts <= dartModel.world.time() <= te:
                dartModel.applyPenaltyForce(selectedBodyId, localPos, extraForce)

            dartModel.step()

        if DART_CONTACT_ON:
            bodyIDs, contactPositions, contactPositionLocals, contactForces = dartModel.get_dart_contact_info()
        else:
            bodyIDs, contactPositions, contactPositionLocals, contactForces = dartModel.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)


if __name__ == '__main__':
    # main()
    ppo2.Model


