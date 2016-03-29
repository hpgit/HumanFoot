import sys
import math
import numpy as np
import numpy.linalg as npl
import itertools

from csVpModel_py import *

sys.path.append('../../..')

from PyCommon.modules.pyVirtualPhysics import *


class VpWorld:
    def __init__(self, config):
        self._world = vpWorld()
        self._world.SetTimeStep(config.timeStep)
        self._world.SetGravity(pyVec3_2_Vec3(config.gravity))
        self.setOpenMP()

        self._planeHeight = config.planeHeight
        self._lockingVel = config.lockingVel

        self._ground = vpBody()

        if config.useDefaultContactModel:
            vpMaterial.GetDefaultMaterial().SetRestitution(0.01)
            vpMaterial.GetDefaultMaterial().SetDynamicFriction(100.)
            vpMaterial.GetDefaultMaterial().SetStaticFriction(100.)
            self._ground.AddGeometry(vpBox(Vec3(100., 0., 100.)))
            self._ground.SetFrame(Vec3(0., self._planeHeight, 0.))
            self._ground.SetGround()
            self._world.AddBody(self._ground)

    def step(self):
        self._world.StepAhead()

    def initialize(self):
        self._world.Initialize()
        self._world.SetIntegrator(IMPLICIT_EULER_FAST)

    def setOpenMP(self):
        self._world.SetNumThreads(100)
        numTh = self._world.GetNumThreads()-1
        self._world.SetNumThreads(numTh)

    def calcPenaltyForce(self, bodyIDsToCheck, mus, Ks, Ds, notForce=False):
        bodyIDs = []
        positions = []
        forces = []
        positionLocals = []
        velocities = []

        for bodyIdx in bodyIDsToCheck:
            bodyID = bodyIDsToCheck(bodyIdx)
            pBody = self._world.GetBody(bodyIDsToCheck(bodyIdx))

            for geomIdx in range(len(pBody.GetNumGeometry())):
                pGeom = pBody.GetGeometry(geomIdx)
                geomType = 'C'
                # TODO:
                data = [0., 0., 0.]
                pGeom.GetShape(geomType, data)

                if geomType == ord('C'):
                    verticesLocal = pGeom.getVerticesLocal()
                    verticesGlobal = pGeom.getVerticesGlobal()

                    for vertIdx in range(len(verticesLocal)):
                        positionLocal = Vec3_2_pyVec3(verticesLocal[vertIdx])
                        position = Vec3_2_pyVec3(verticesGlobal[vertIdx])
                        velocity = Vec3_2_pyVec3(pBody.GetLinVelocity(pyVec3_2_Vec3(positionLocal)))
                        if notForce:
                            penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, 0., True)
                        else:
                            penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, mus[bodyIdx])

                        if penentrated:
                            bodyIDs.append(bodyID)
                            positions.append(position)
                            positionLocals.append(positionLocal)
                            forces.append(force)
                            velocities.append(velocity)
                elif True:
                    # TODO:
                    # how to deal with SE3?
                    geomFrame = pGeom.GetGlobalFrame()
                    for perm in itertools.product([1, -1], repeat=2):
                        positionLocal = np.multiply(np.array(data), np.array(perm))
                        position = Vec3_2_pyVec3(geomFrame * pyVec3_2_Vec3(positionLocal))
                        velocity = Vec3_2_pyVec3(pBody.GetLinVelocity(pyVec3_2_Vec3(positionLocal)))
                        if notForce:
                            penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, 0., True)
                        else:
                            penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, mus[bodyIdx])

                        if penentrated:
                            bodyIDs.append(bodyID)
                            positions.append(position)
                            positionLocals.append(positionLocal)
                            forces.append(force)
                            velocities.append(velocity)

        if notForce:
            return bodyIDs, positions, positionLocals, velocities
        return bodyIDs, positions, positionLocals, forces

    def _calcPenaltyForce(self, position, velocity, Ks, Ds, mu, notForce=False):
        vNormal = np.array([0., 1., 0.])

        vRelVel = np.array(velocity)
        normalRelVel = np.dot(vRelVel, vNormal)
        vNormalRelVel = normalRelVel * vNormal
        vTangentialRelVel = vRelVel - vNormalRelVel
        tangentialRelVel = npl.norm(vTangentialRelVel)

        depth = self._planeHeight - position[1]

        if depth < 0:
            return False, np.array([0., 0., 0.])
        elif notForce:
            return True, np.array([0., 0., 0.])
        else:
            normalForce = Ks * depth - Ds * velocity[1]
            if normalForce < 0.:
                return True, np.array([0., 0., 0.])

            vNormalForce = normalForce * vNormal
            frictionForce = mu * normalForce

            if tangentialRelVel < self._lockingVel:
                frictionForce *= tangentialRelVel / self._lockingVel

            vFrictionForce = -frictionForce * vTangentialRelVel / npl.norm(vTangentialRelVel)

            force = vNormalForce + vFrictionForce

        return True, force

    def applyPenaltyForce(self, bodyIDs, positionLocals, forces):
        for bodyIdx in range(len(bodyIDs)):
            bodyID = bodyIDs[bodyIdx]
            pBody = self._world.GetBody(bodyID)
            pBody.ApplyGlobalForce(pyVec3_2_Vec3(forces[bodyIdx]), pyVec3_2_Vec3(positionLocals[bodyIdx]))

    def getContactPoints(self, bodyIDsToCheck):
        return self.calcPenaltyForce(bodyIDsToCheck, None, 0., 0., True)
