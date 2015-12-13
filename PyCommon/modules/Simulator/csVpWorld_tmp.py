import numpy as np
from numpy import linalg as npl
import itertools
import VirtualPhysics.vpWorld as vpw
import VirtualPhysics.vpBody as vpb

class vpMaterial:
    def __init__(self):
        pass

    def GetDefaultMaterial(self):
        return self

    def SetRestitution(self, res):
        pass

    def SetDynamicFriction(self, mu):
        pass

    def SetStaticFriction(self, mu):
        pass


class vpGeom:
    def __init__(self):
        pass


class vpBox(vpGeom):
    def __init__(self, _size):
        pass


class vpJoint():
    def __init__(self):
        pass


class VpWorld:
    def __init__(self, config):
        self._world = vpw.vpWorld()
        self._world.SetGravity(config.gravity)
        self._ground = vpb.vpBody()
        self._timeStep = config.timeStep
        self._planeHeight = config.planeHeight
        self._lockingVel = config.lockingVel

        if config.useDefaultContactModel:
            vpMaterial.GetDefaultMaterial().SetRestitution(0.01)
            vpMaterial.GetDefaultMaterial().SetDynamicFriction(100)
            vpMaterial.GetDefaultMaterial().SetStaticFriction(100)
            self._ground.AddGeometry(vpBox(np.array([100., 0., 100.])))
            self._ground.SetFrame(np.array([0., self._planeHeight, 0.]))
            self._ground.SetGround()
            self._world.AddBody(self._ground)

    def step(self):
        self._world.StepAhead()

    def getBodyNum(self):
        return self._world.GetNumBody()

    def initialize(self):
        self._world.Initialize()
        self._world.SetIntegrator("IMPLICIT_EULER_FAST")

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
                geomType, data = pGeom.GetShape()

                if geomType == ord('C'):
                    verticesLocal = pGeom.getVerticesLocal()
                    verticesGlobal = pGeom.getVerticesGlobal()

                    for vertIdx in range(len(verticesLocal)):
                        positionLocal = verticesLocal[vertIdx]
                        position = verticesGlobal[vertIdx]
                        velocity = pBody.GetLinVelocity(positionLocal)
                        penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, mus[bodyIdx], notForce)

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
                        position = geomFrame * positionLocal
                        velocity = pBody.GetLinVelocity(positionLocal)
                        penentrated, force = self._calcPenaltyForce(position, velocity, Ks, Ds, mus[bodyIdx], notForce)

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
            pBody.ApplyGlobalForce(forces[bodyIdx], positionLocals[bodyIdx])

