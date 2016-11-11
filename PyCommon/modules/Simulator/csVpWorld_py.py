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
        # self.models = [VpControlModel(None, None, None)]
        self.models = []

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

    def getBodyNum(self):
        return self._world.GetNumBody()

    def addVpModel(self, model):
        self.models.append(model)

    def GetTimeStep(self):
        return self._world.GetTimeStep()

    def calcPenaltyForce(self, bodyIDsToCheck, mus, Ks, Ds, notForce=False):
        bodyIDs = []
        positions = []
        forces = []
        positionLocals = []
        velocities = []

        if True:
            for model in self.models:
                for node in model._nodes:
                    bodyID = node.body.GetID()
                    if bodyID in bodyIDsToCheck:
                        for pGeom in node.geoms:
                            geomType = pGeom.GetType()
                            if geomType == 'C':
                                verticesLocal = pGeom.getVerticesLocal()
                                verticesGlobal = pGeom.getVerticesGlobal()

                                for vertIdx in range(len(verticesLocal)):
                                    positionLocal = Vec3_2_pyVec3(verticesLocal[vertIdx])
                                    position = Vec3_2_pyVec3(verticesGlobal[vertIdx])
                                    velocity = Vec3_2_pyVec3(node.body.GetLinVelocity(pyVec3_2_Vec3(positionLocal)))
                                    if notForce:
                                        penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, 0., True)
                                    else:
                                        penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, mus[0])

                                    if penentrated:
                                        bodyIDs.append(bodyID)
                                        positions.append(position)
                                        positionLocals.append(positionLocal)
                                        forces.append(force)
                                        velocities.append(velocity)

                            elif False:
                                # TODO:
                                # how to deal with SE3?
                                geomFrame = pGeom.GetGlobalFrame()
                                data = pGeom.GetSize()
                                for perm in itertools.product([1, -1], repeat=3):
                                    positionLocal = np.multiply(np.array((data[0], data[1], data[2])), np.array(perm))
                                    position = Vec3_2_pyVec3(geomFrame * pyVec3_2_Vec3(positionLocal))
                                    velocity = Vec3_2_pyVec3(node.body.GetLinVelocity(pyVec3_2_Vec3(positionLocal)))
                                    if notForce:
                                        penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, 0., True)
                                    else:
                                        penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, mus[0])

                                    if penentrated:
                                        print node.name
                                        bodyIDs.append(bodyID)
                                        positions.append(position)
                                        positionLocals.append(positionLocal)
                                        forces.append(force)
                                        velocities.append(velocity)


        if False:
            for bodyIdx in range(len(bodyIDsToCheck)):
                bodyID = bodyIDsToCheck[bodyIdx]
                pBody = self._world.GetBody(bodyIDsToCheck[bodyIdx])

                for geomIdx in range(pBody.GetNumGeometry()):
                    pGeom = pBody.GetGeometry(geomIdx)
                    geomType = pGeom.GetType()

                    if geomType == 'C':
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
                    elif False:
                        # TODO:
                        # how to deal with SE3?
                        geomFrame = pGeom.GetGlobalFrame()
                        data = pGeom.GetSize()
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
