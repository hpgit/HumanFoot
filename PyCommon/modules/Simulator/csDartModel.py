import sys
import math
import numpy as np
import numpy.linalg as npl
import itertools

sys.path.append('../../..')

# from PyCommon.modules.Simulator.csVpUtil import *
# from PyCommon.modules.Simulator.myGeom import *

from PyCommon.modules.Math import mmMath as mm
import PyCommon.modules.Simulator.ysPhysConfig as ypc
from PyCommon.modules.Motion import ysMotion as ym

from PyCommon.modules import pydart2 as pydart
from PyCommon.modules.dart.bvh2dartSkel import DartModelMaker


class DartModel:
    """
    :type world : pydart.World
    :type config : ypc.ModelConfig
    :type skeleton: pydart.Skeleton
    :type boneTs : list[np.ndarray]
    :type geomPoints : list[list[np.ndarray]]
    """

    def __init__(self, wcfg, posture, mcfg, isContainGround=True):
        """
        :type wcfg: ypc.WorldConfig
        :type posture: ym.JointPosture
        :type mcfg: ypc.ModelConfig
        """
        xmlstr, self._boneTs = DartModelMaker().posture2dartSkelXmlStr("dartModel", posture, mcfg, isContainGround)
        # print xmlstr

        self.world = pydart.World(wcfg.timeStep, xmlstr, True)

        self.config = mcfg

        self.planeHeight = wcfg.planeHeight
        self.lockingVel = wcfg.lockingVel

        if self.world.skeletons[0].name == "grount skeleton":
            self.hasGround = True
            self.skeleton = self.world.skeletons[1]
        else:
            self.hasGround = False
            self.skeleton = self.world.skeletons[0]

        self.update(posture)

        self.geomPoints = [None]*self.getBodyNum()
        self.initContactPoint()

    def step(self):
        self.world.step()

    def getBodyNum(self):
        return self.skeleton.num_bodynodes()

    def GetTimeStep(self):
        return self.world.time_step()

    def SetTimeStep(self, h):
        self.world.set_time_step(h)

    def SetGravity(self, g):
        self.world.set_gravity(g)

    def initContactPoint(self):
        # in sphere(ellipsoid) case, saves points around center of hemisphere in local body frame,
        #    so additional calculation is needed

        # in box case, saves points around in local body frame
        MULTIPLE_BOX = False
        MULTIPLE_CAPSULE = False

        for i in range(self.skeleton.num_bodynodes()):
            body = self.getBody(i)
            for shapeNode in body.shapenodes:
                if shapeNode.has_collision_aspect():
                    geomType = shapeNode.shape.shape_type_name()
                    # geomT = np.dot(body.world_transform(), shapeNode.relative_transform())
                    if MULTIPLE_CAPSULE and geomType == "ELLIPSOID":
                        rad, gap, row, col, height_ratio = .01, .01, 4, 4, .5
                        shape = shapeNode.shape # type: pydart.EllipsoidShape
                        lowestCenter = np.array([0., 0., 0.])

                        geomPoint = list()
                        geomPoint.extend(self.getContactSphere(lowestCenter, rad, row, col, height_ratio))
                        self.geomPoints[i] = geomPoint
                    elif geomType == "ELLIPSOID":
                        self.geomPoints[i] = [np.array([0., 0., 0.])]

                    elif MULTIPLE_BOX and geomType == "BOX":
                        rad, gap, row, col, height_ratio = .05, .01, 6, 6, .5
                        shape = shapeNode.shape # type: pydart.BoxShape
                        data = shape.size()/2. # type: np.ndarray

                        t, b, l, r, l0 = data[2], -data[2], -data[0], data[0], -data[1]
                        positionTmpCenters = [np.array((l+gap, l0+rad, t-gap)), np.array((l+gap, l0+rad, b+gap)),
                                              np.array((r-gap, l0+rad, t-gap)), np.array((r-gap, l0+rad, b+gap)),
                                            np.array((l+gap, l0+rad, 0.)), np.array((r-gap, l0+rad, 0.))]

                        geomPoint = list()
                        for posCenter in positionTmpCenters:
                            geomPoint.extend(self.getContactSphere(posCenter, rad, row, col, height_ratio))
                        self.geomPoints[i] = geomPoint

                    elif geomType == "BOX":
                        shape = shapeNode.shape # type: pydart.BoxShape
                        data = shape.size()/2. # type: np.ndarray
                        geomPoint = list()
                        for perm in itertools.product([1, -1], repeat=3):
                            positionLocal = np.multiply(np.array((data[0], data[1], data[2])), np.array(perm))
                            geomPoint.append(positionLocal)
                        self.geomPoints[i] = geomPoint

    @staticmethod
    def getContactSphere(center, rad, row, col, height_ratio):
        vertices = list()
        col_real = 0
        _row = int(row*height_ratio)

        for i in range(_row):
            t = float(i)/float(row)
            cp = math.cos(math.pi * t - math.pi/2.)
            sp = math.sin(math.pi * t - math.pi/2.)

            col_real = 1 if i == 0 else col
            for j in range(col_real):
                s = float(j)/float(col_real)
                ct = math.cos(2.*math.pi*s)
                st = math.sin(2.*math.pi*s)
                vertices.append(np.array((rad*cp*ct, rad*sp, -rad*cp*st)) + center)

        return vertices

    def applyPenaltyForce(self, bodyIDs, positionLocals, forces, localForce=True):
        for bodyIdx in range(len(bodyIDs)):
            bodyID = bodyIDs[bodyIdx]
            body = self.getBody(bodyID)
            body.add_ext_force(forces[bodyIdx], positionLocals[bodyIdx], False, localForce)

    def get_dart_contact_info(self):
        bodyIDs = [contact.bodynode2.id if contact.bodynode1.name == 'ground' else contact.bodynode1.id
                   for contact in self.world.collision_result.contacts]
        positions = [contact.point for contact in self.world.collision_result.contacts]
        positionLocals = [self.getBody(bodyIDs[i]).to_local(positions[i]) for i in range(len(bodyIDs))]
        forces = [(-contact.force if contact.bodynode1.name == 'ground' else contact.force)
                         for contact in self.world.collision_result.contacts]

        pop_index = []
        for i in range(len(bodyIDs)-1, -1, -1):
            if npl.norm(forces[i]) < 0.000001:
                pop_index.append(i)

        [bodyIDs.pop(i) for i in pop_index]
        [positions.pop(i) for i in pop_index]
        [positionLocals.pop(i) for i in pop_index]
        [forces.pop(i) for i in pop_index]

        return bodyIDs, positions, positionLocals, forces

    def getContactPoints(self, bodyIDsToCheck):
        bodyIDs = []
        positions = []
        positionLocals = []
        velocities = []
        forces = []
        for i in bodyIDsToCheck:
            body = self.getBody(i)
            for shapeNode in body.shapenodes:
                if shapeNode.has_collision_aspect():
                    geomType = shapeNode.shape.shape_type_name()
                    geomT = np.dot(body.world_transform(), shapeNode.relative_transform())
                    if geomType == "ELLIPSOID":
                        shape = shapeNode.shape # type: pydart.EllipsoidShape
                        lowestPoint = geomT[:3, 3]
                        lowestPoint[1] -= shape.size()[0]/2.
                        if lowestPoint[1] < 0.:
                            bodyIDs.append(i)
                            positions.append(lowestPoint)
                            positionLocals.append(body.to_local(lowestPoint))
                            #TODO:
                            # velocities.append(body.com_spatial_velocity())
                    elif geomType == "BOX":
                        shape = shapeNode.shape # type: pydart.BoxShape
                        data = shape.size()/2.
                        for perm in itertools.product([1, -1], repeat=3):
                            positionLocal = np.multiply(np.array((data[0], data[1], data[2])), np.array(perm))
                            position = np.dot(geomT[:3, :3], positionLocal) + geomT[:3, 3]
                            #TODO:
                            # velocity = node.body.GetLinVelocity(pyVec3_2_Vec3(positionLocal))

                            if position[1] < 0.:
                                bodyIDs.append(body.index_in_skeleton())
                                positions.append(position)
                                positionLocals.append(positionLocal)
                                # forces.append(force)
                                #TODO:
                                # velocities.append(velocity)

        return bodyIDs, positions, positionLocals, None
        # return self.calcPenaltyForce(bodyIDsToCheck, None, 0., 0., True)

    def calcPenaltyForce(self, bodyIDsToCheck, mus, Ks, Ds):
        def _calcPenaltyForce(pBody, position, velocity, mu, lockingVel):
            """
            :type pBody: pydart.BodyNode
            :type position: np.ndarray
            :type velocity: np.ndarray
            :type force: np.ndarray
            :type Ks: float
            :type Ds: float
            :type mu: float
            """
            if position[1] >= 0.:
                return False, np.zeros(3)
            else:
                vNormalRelVel = np.array((0., velocity[1], 0.))
                vTangentialRelVel = velocity - vNormalRelVel
                tangentialRelVel = npl.norm(vNormalRelVel)

                normalForce = max(0., -Ks*position[1] - Ds*velocity[1])
                vNormalForce = np.array((0., normalForce, 0.))
                frictionForce = mu * normalForce

                if tangentialRelVel < lockingVel:
                    frictionForce *= tangentialRelVel / lockingVel
                vFrictionForce = -frictionForce * (mm.normalize2(vTangentialRelVel))
                force = vNormalForce + vFrictionForce
                return True, force

        # rad, gap, row, col, height_ratio = .03, .01, 6, 6, .5

        bodyIDs, positions, positionLocals, velocities, forces = [], [], [], [], []
        MULTIPLE_POINT = False

        for i in range(len(bodyIDsToCheck)):
            body = self.getBody(bodyIDsToCheck[i])
            bodyIdx = body.index_in_skeleton()
            for shapeNode in body.shapenodes:
                if shapeNode.has_collision_aspect():
                    geomType = shapeNode.shape.shape_type_name()
                    geomT = np.dot(body.world_transform(), shapeNode.relative_transform())
                    if not MULTIPLE_POINT and geomType == "ELLIPSOID":
                        # single point
                        shape = shapeNode.shape # type: pydart.EllipsoidShape
                        lowestPoint = geomT[:3, 3]
                        lowestPoint[1] -= shape.size()[0]/2.
                        if lowestPoint[1] < 0.:
                            spatialVel = body.com_spatial_velocity() # type: np.ndarray
                            velocity = body.com_linear_velocity() + np.cross(spatialVel[:3], lowestPoint - body.com())
                            isPenetrated, force = _calcPenaltyForce(body, lowestPoint, velocity, mus[i], self.lockingVel)
                            bodyIDs.append(body.index_in_skeleton())
                            positions.append(lowestPoint)
                            positionLocals.append(body.to_local(lowestPoint))
                            velocities.append(velocity)
                            forces.append(force)

                    elif geomType == "ELLIPSOID":
                        # multiple point
                        shape = shapeNode.shape # type: pydart.EllipsoidShape
                        geomPoint = self.geomPoints[bodyIdx]

                        bodySpatialVel = body.com_spatial_velocity() # type: np.ndarray
                        bodyLinVel = body.com_linear_velocity() # type: np.ndarray
                        for posIdx in range(len(geomPoint)):
                            positionGlobal = np.dot(geomT[:3, :3], geomPoint[posIdx]) + geomT[:3, 3]
                            # positionGlobal[1] -= shape.size()[0]/2.
                            if positionGlobal[1] < 0.:
                                velocity = bodyLinVel + np.cross(bodySpatialVel[:3], positionGlobal - body.com())
                                isPenetrated, force = _calcPenaltyForce(body, positionGlobal, velocity, mus[i], self.lockingVel)
                                bodyIDs.append(body.index_in_skeleton())
                                positions.append(positionGlobal)
                                positionLocals.append(body.to_local(positionGlobal))
                                velocities.append(velocity)
                                forces.append(force)

                    elif True and geomType == "BOX":
                        # multiple point
                        shape = shapeNode.shape # type: pydart.BoxShape
                        geomPoint = self.geomPoints[bodyIdx]
                        # print self.getBody(bodyIdx).name, len(geomPoint)
                        # print geomPoint

                        bodySpatialVel = body.com_spatial_velocity() # type: np.ndarray
                        bodyLinVel = body.com_linear_velocity() # type: np.ndarray
                        # geomT = body.world_transform()
                        for posIdx in range(len(geomPoint)):
                            positionGlobal = np.dot(geomT[:3, :3], geomPoint[posIdx]) + geomT[:3, 3]
                            # print self.getBody(bodyIdx).name, positionGlobal
                            if positionGlobal[1] < 0.:
                                velocity = bodyLinVel + np.cross(bodySpatialVel[:3], positionGlobal - body.com())
                                isPenetrated, force = _calcPenaltyForce(body, positionGlobal, velocity, mus[i], self.lockingVel)
                                # print "positionGlobal", positionGlobal
                                bodyIDs.append(body.index_in_skeleton())
                                positions.append(positionGlobal)
                                positionLocals.append(body.to_local(positionGlobal))
                                velocities.append(velocity)
                                forces.append(force)

                    elif False and geomType == "BOX":
                        # single point
                        shape = shapeNode.shape  # type: pydart.BoxShape
                        data = shape.size() / 2.  # type: np.ndarray
                        for perm in itertools.product([1, -1], repeat=3):
                            positionLocal = np.multiply(np.array((data[0], data[1], data[2])), np.array(perm))
                            position = np.dot(geomT[:3, :3], positionLocal) + geomT[:3, 3]

                            if position[1] < 0.:
                                spatialVel = body.com_spatial_velocity()  # type: np.ndarray
                                velocity = body.com_linear_velocity() + np.cross(spatialVel[:3], position - body.com())
                                isPenetrated, force = _calcPenaltyForce(body, position, velocity, mus[i], self.lockingVel)
                                bodyIDs.append(body.index_in_skeleton())
                                positions.append(position)
                                positionLocals.append(positionLocal)
                                velocities.append(velocity)
                                forces.append(force)

        return bodyIDs, positions, positionLocals, forces

    def build_name2index(self):
        for i in range(len(self._nodes)):
            self._name2index[self._nodes[i].name] = i

    def index2name(self, index):
        return self._nodes[index]

    def index2id(self, index):
        return self._nodes[index].body.GetID()

    def name2index(self, name):
        try:
            return self._name2index[name]
        except:
            return -1

    def name2id(self, name):
        return self.index2id(self.name2index(name))

    def getBodyMasses(self):
        return [body.mass() for body in self.skeleton.bodynodes]

    def getTotalMass(self):
        return self.skeleton.mass()

    def getBody(self, key):
        """

        :type str|int
        :rtype: pydart.BodyNode
        """
        return self.skeleton.body(key)

    def getJoint(self, key):
        """

        :type key: str|int
        :rtype: pydart.Joint
        """
        if isinstance(key, str):
            return self.skeleton.joint("j_"+key)
        elif isinstance(key, int):
            return self.skeleton.joint(key)
        else:
            raise TypeError

    def setRenderColor(self, color):
        for body in self.skeleton.bodynodes:
            for shapenode in body.shapenodes:
                shapenode.set_visual_aspect_rgba(color)

    def getBodyShape(self, index):
        # pGeom = self._nodes[index].body.GetGeometry(0)
        geom = self._nodes[index].geoms[0]
        _type = geom.GetType()
        data = []

        #TODO:
        # class			 vpSphere;		// S
        if _type == 'B':
            _data = geom.GetSize()
            data.append(_data[0])
            data.append(_data[1])
            data.append(_data[2])

        # class			 vpBox;			// B
        # class			 vpCapsule;		// C
        if _type == 'C':
            data.append(geom.GetRadius())
            data.append(geom.GetHeight())
        # class			 vpPlane;		// P
        # class			 vpCylinder;	// L
        # class			 vpTorus;		// T

        return data

    def getBodyVerticesPositionGlobal(self, index):
        # pGeom = self._nodes[index].body.GetGeometry(0)
        pGeom = self._nodes[index].geoms[0]
        _type = pGeom.GetType()
        ls_point = []
        data = [0., 0., 0.]
        # TODO:
        # check if GetShape work well
        # there is a problem!!! data : scalar * .....
        # pGeom.GetShape(_type, data)
        geomFrame = pGeom.GetGlobalFrame()

        for i in range(8):
            # TODO:
            # point[0] =
            point = geomFrame * Vec3(point[0], point[1], point[2])

            ls_point.append(point)

        return ls_point

    def getBodyInertiaLocal(self, index):
        return self.getBody(index).inertia()

    def getBodyInertiaGlobal(self, index):
        bodyFrame = self.getBody(index).world_transform()[:3, :3]
        return np.dot(bodyFrame, np.dot(self.getBody(index).inertia(), bodyFrame.T))

    def getBodyInertiasLocal(self):
        return [self.getBodyInertiaLocal(i) for i in range(self.getBodyNum())]

    def getBodyInertiasGlobal(self):
        return [self.getBodyInertiaGlobal(i) for i in range(self.getBodyNum())]

    def getCOM(self):
        return self.skeleton.com()

    def getCOMJacobian(self):
        body_num = self.getBodyNum()
        dof_num = self.getTotalDOF()
        masses = self.getBodyMasses()
        total_mass_inv = 1./sum(masses)
        jacobian = np.zeros((3, dof_num))
        for i in range(body_num):
            jacobian += (masses[i] * total_mass_inv) * self.getBody(i).linear_jacobian()

        return jacobian

    def getBoneT(self, index):
        bone_T = mm.invertSE3(self.getJoint(index).transform_from_child_body_node())
        return mm.SE3_to_SO3_vec3(bone_T)

    def getInvBoneT(self, index):
        inv_bone_T = self.getJoint(index).transform_from_child_body_node()
        return mm.SE3_to_SO3_vec3(inv_bone_T)

    def getBodyGenVelLocal(self, index):
        return se3_2_pyVec6(self._nodes[index].body.GetGenVelocityLocal())

    def getBodyGenVelGlobal(self, index):
        return se3_2_pyVec6(self._nodes[index].body.GetGenVelocity())

    def getBodyGenAccLocal(self, index):
        return se3_2_pyVec6(self._nodex[index].body.GetGenAccelerationLocal())

    def getBodyGenAccGlobal(self, index):
        return se3_2_pyVec6(self._nodes[index].body.GetGenAccleration())

    def getBodyPositionGlobal(self, index, pPositionLocal=None):
        return self.skeleton.body(index).to_world()

    def getBodyOrientationGlobal(self, index):
        return self.getBody(index).transform()[:3, :3]

    def getBodyVelocityGlobal(self, index, positionLocal=None):
        return self.getBody(index).world_linear_velocity(positionLocal)

    def getBodyVelocitiesGlobal(self):
        return [self.getBodyVelocityGlobal(i) for i in range(self.getBodyNum())]

    def getBodyAngVelocityGlobal(self, index):
        return self.getBody(index).world_angular_velocity()

    def getBodyAngVelocitiesGlobal(self):
        return [self.getBodyAngVelocityGlobal(i) for i in range(self.getBodyNum())]

    def getBodyAccelerationGlobal(self, index, positionLocal=None):
        return self.getBody(index).world_linear_acceleration(positionLocal)

    def getBodyAccelerationsGlobal(self):
        return [self.getBodyAccelerationGlobal(i) for i in range(self.getBodyNum())]

    def setBodyPositionGlobal(self, index, position):
        # SE3 bodyFrame;
        bodyFrame = self._nodes[index].body.GetFrame()
        bodyFrame.SetPosition(pyVec3_2_Vec3(position))
        self._nodes[index].body.SetFrame(bodyFrame)

    def setBodyAccelerationGlobal(self, index, acc, pPositionLocal=None):
        # if pPositionLocal is not None:
        #     print "setBodyAccelerationGlobal: not implemented pPositionLocal yet."

        # se3 genAcc;
        genAcc = self._nodes[index].body.GetGenAcceleration()
        # genAcc[3] = acc[0]
        # genAcc[4] = acc[1]
        # genAcc[5] = acc[2]
        _genAcc = se3(genAcc[0], genAcc[1], genAcc[2], acc[0], acc[1], acc[2])

        self._nodes[index].body.SetGenAcceleration(_genAcc)

    def setBodyAngVelocityGlobal(self, index, angvel):
        genVel = self._nodes[index].body.GetGenVelocity()
        # genVel[0] = angvel[0]
        # genVel[1] = angvel[1]
        # genVel[2] = angvel[2]
        _genVel = se3(angvel[0], angvel[1], angvel[2], genVel[3], genVel[4], genVel[5])
        self._nodes[index].body.SetGenVelocity(_genVel)

    def setBodyAngAccelerationGlobal(self, index, angacc):
        genAcc = self._nodes[index].body.GetGenAcceleration()
        # genAcc[0] = angacc[0]
        # genAcc[1] = angacc[1]
        # genAcc[2] = angacc[2]
        _genAcc = se3(angacc[0], angacc[1], angacc[2], genAcc[3], genAcc[4], genAcc[5])
        self._nodes[index].body.SetGenAcceleration(_genAcc)

    def getBodyPositionsGlobal(self):
        return [self.getBodyPositionGlobal(i) for i in range(self.getBodyNum())]

    def getBodyAngAccelerationGlobal(self, index):
        pyV = np.zeros(3)

        genAcc = self._nodes[index].body.GetGenAcceleration()
        pyV[0] = genAcc[0]
        pyV[1] = genAcc[1]
        pyV[2] = genAcc[2]
        return pyV

    def getBodyAngAccelerationsGlobal(self):
        ls = []
        for i in range(len(self._nodes)):
            ls.append(self.getBodyAngAccelerationGlobal(i))
        return ls

    def translateByOffset(self, offset):
        q = self.skeleton.q
        q[3:6] += np.array(offset)
        self.skeleton.set_positions(q)

    def rotate(self, rotation):
        q = self.skeleton.q
        q[:3] = mm.logSO3(np.dot(rotation, mm.exp(self.skeleton.q[:3])))
        self.skeleton.set_positions(q)

    def ignoreCollisionWith(self, pBody):
        for i in range(len(self._nodes)):
            self._pWorld.IgnoreCollision(self._nodes[i].body, pBody )

    def addBody(self, flagControl):
        node = VpModel.Node("Left_Toes")
        density = 1000.
        width = .1
        height = .1
        length = .1
        node.body.AddGeometry(MyBox(Vec3(width, height, length)))
        node.body.SetInertia(BoxInertia(density, Vec3(width/2., height/2., length/2.)))

        newT = SE3()
        node.body.SetFrame(newT)
        self._nodes.append(node)
        self._boneTs.append(newT)
        if flagControl:
            self._pWorld.AddBody(node.body)

        invLocalT = SE3()
        parentIndex = len(self._nodes) - 1
        parentNode = self._nodes[parentIndex]
        parentNode.body.SetJoint(node.joint, Inv(self._boneTs[parentIndex])*Inv(invLocalT))
        node.body.SetJoint(node.joint, Inv(self._boneTs[len(self._nodes)]))
        node.use_joint = True

        raise NotImplementedError
        # return

    def recordVelByFiniteDiff(self, flag=True, motionTimeStep=1./30.):
        self._recordVelByFiniteDiff = flag
        self._inverseMotionTimeStep = 1./motionTimeStep

    def __str__(self):
        strstr = "<JOINTS, BODIES>\n"
        for i in range(len(self.skeleton.bodynodes)):
            strstr += "[" + str(i) + "]" + self.skeleton.joints[i].name + " ," + self.skeleton.bodynodes[i].name + "\n"
        strstr += "\n"
        strstr = "<BODY MASSES>\n"
        for i in range(len(self.skeleton.bodynodes)):
            strstr += "[" + str(i) + "]" + str(self.skeleton.bodynodes[i].mass) + ", \n"
        strstr += "\n"
        # strstr += "<INTERNAL JOINTS>\n"
        # for i in range(1, len(self._nodes)):
        #     strstr += "[" + str(i-1) + "]:" + self._nodes[i].name + ", " + str(self._nodes[i].dof) + "DOF\n"
        # strstr += "\n"

        return strstr

    def getInternalJointDOFs(self):
        ls = [self.getJoint(i).num_dofs() for i in range(1, len(self.skeleton.joints))]
        return ls

    def getTotalInternalJointDOF(self):
        # return 3*(len(self._nodes)-1)
        return sum(self.getInternalJointDOFs())

    def getDOFs(self):
        return [joint.num_dofs() for joint in self.skeleton.joints]

    def getTotalDOF(self):
        return self.skeleton.num_dofs()

    def get3dExtendTotalDOF(self):
        return 6 + 3*(len(self._nodes)-1)

    def get3dExtendDOFs(self):
        return [[6]] + [[3]]*(len(self._nodes)-1)

    def ignoreCollisionBtwnBodies(self):
        for i in range(len(self._nodes)):
            for j in range(i, len(self._nodes)):
                self._pWorld.IgnoreCollision(self._nodes[i].body, self._nodes[j].body)


    def update(self, posture):
        """

        :type posture: ym.JointPosture
        :return:
        """
        q = self.skeleton.q
        q[0:3] = mm.logSO3(posture.getJointOrientationGlobal(0))
        q[3:6] = posture.getJointPositionGlobal(0)
        self.skeleton.set_positions(q)

        for j in range(1, len(self.skeleton.joints)):
            joint = self.skeleton.joints[j]
            joint_q = mm.logSO3(posture.getJointOrientationLocal(posture.skeleton.getJointIndex(joint.name[2:])))
            for i in range(len(joint.dofs)):
                dof = joint.dofs[i] # type: pydart.Dof
                dof.set_position(joint_q[i])

        # joint = posture.skeleton.root
        # self._updateJoint(joint, posture)

    def initializeHybridDynamics(self, flotingBase=True):
        rootIndex = 0
        for i in range(self.skeleton.num_joints()):
            joint = self.getJoint(i)
            if i == rootIndex:
                if flotingBase:
                    joint.set_actuator_type(pydart.Joint.FORCE)
                else:
                    joint.set_actuator_type(pydart.Joint.ACCELERATION)
            else:
                joint.set_actuator_type(pydart.Joint.ACCELERATION)

    def initializeForwardDynamics(self):
        for i in range(self.skeleton.num_joints()):
            joint = self.getJoint(i).set_actuator_type(pydart.Joint.FORCE)

    def makeDOFFlatList(self):
        return [None] * self.getTotalDOF()

    def makeDOFNestedList(self):
        ls = []
        for joint in self.skeleton.joints:
            ls.append([None]*joint.num_dofs())
        return ls

    def makeExtendDOFFlatList(self):
        return [None] * self.get3dExtendTotalDOF()

    def makeExtendDOFNestedList(self):
        ls = []
        for i in range(len(self._nodes)):
            ls.append([None]*3)
        return ls

    def flattenDOF(self, DOFs):
        raise NotImplementedError
        # return

    def nestedDOF(self, dofs):
        raise NotImplementedError
        # return

    def solveHybridDynamics(self):
        self._nodes[0].body.GetSystem().HybridDynamics()

    def solveForwardDynamics(self):
        self._nodes[0].body.GetSystem().ForwardDynamics()

    def solveInverseDynamics(self):
        self._nodes[0].body.GetSystem().InverseDynamics()

    def get_q(self):
        return self.skeleton.q

    def get_dq(self):
        return self.skeleton.dq

    def get_state(self):
        return np.hstack((self.skeleton.q, self.skeleton.dq))

    # Get Joint Local State
    def getJointOrientationLocal(self, index):
        return self.getJoint(index).get_local_transform()[:3, :3]

    def getJointVelocityLocal(self, index):
        pospos = Inv(self._boneTs[index]).GetPosition()
        jointFrame = self._nodes[index].body.GetFrame() * Inv(self._boneTs[index])
        return Vec3_2_pyVec3(InvRotate(jointFrame, pyVec3_2_Vec3(self.getBodyVelocityGlobal(index, pospos))))

    def getJointAngVelocityLocal(self, index):
        if index == 0:
            genVelBodyLocal = self._nodes[index].body.GetGenVelocityLocal()
            genVelJointLocal = InvAd(Inv(self._boneTs[index]), genVelBodyLocal)
            # genVelJointLocal = Ad(self._boneTs[index], genVelBodyLocal)
            pyV = np.zeros(3)
            pyV[0] = genVelJointLocal[0]
            pyV[1] = genVelJointLocal[1]
            pyV[2] = genVelJointLocal[2]
            return pyV

        return Vec3_2_pyVec3(self._nodes[index].joint.GetVelocityLocal())

    def getJointAccelerationLocal(self, index):
        pospos = Inv(self._boneTs[index]).GetPosition()
        jointFrame = self._nodes[index].body.GetFrame() * Inv(self._boneTs[index])
        return Vec3_2_pyVec3(InvRotate(jointFrame, self.getBodyAccelerationGlobal(index, pospos)))

    def getJointAngAccelerationLocal(self, index):
        if index == 0:
            genAccBodyLocal = self._nodes[index].body.GetGenAccelerationLocal()
            genAccJointLocal = InvAd(Inv(self._boneTs[index]), genAccBodyLocal)
            pyV = np.zeros(3)
            pyV[0] = genAccJointLocal[0]
            pyV[1] = genAccJointLocal[1]
            pyV[2] = genAccJointLocal[2]
            return pyV
        return Vec3_2_pyVec3(self._nodes[index].joint.GetAccelerationLocal())

    # Get Joints Local State
    def getJointOrientationsLocal(self):
        return [self.getJointOrientationLocal(i) for i in range(len(self._nodes))]

    def getJointAngVelocitiesLocal(self):
        return [self.getJointAngVelocityLocal(i) for i in range(len(self._nodes))]

    def getJointAngAccelerationsLocal(self):
        return [self.getJointAngAccelerationLocal(i) for i in range(len(self._nodes))]

    def getInternalJointOrientationsLocal(self):
        return [self.getJointOrientationLocal(i) for i in range(1, len(self._nodes))]

    def getInternalJointAngVelocitiesLocal(self):
        return [self.getJointAngVelocityLocal(i) for i in range(1, len(self._nodes))]

    def getInternalJointAngAccelerationsLocal(self):
        return [self.getJointAngAccelerationLocal(i) for i in range(1, len(self._nodes))]

    def getInternalJointDOFSecondDerivesLocal(self):
        accs = []
        for i in range(1, len(self._nodes)):
            acc = []
            for j in range(self._nodes[i].joint.GetDOF()):
                acc.append(self._nodes[i].joint.GetSecondDeriv(j))
            accs.append(acc)
        return accs

    def getInternalJointDOFSecondDerivesLocalFlat(self):
        acc = []
        for i in range(1, len(self._nodes)):
            for j in range(self._nodes[i].joint.GetDOF()):
                acc.append(self._nodes[i].joint.GetSecondDeriv(j))
        return np.array(acc)

    def getInternalJointDOFFirstDerivesLocal(self):
        vels = []
        for i in range(1, len(self._nodes)):
            vel = []
            for j in range(self._nodes[i].joint.GetDOF()):
                vel.append(self._nodes[i].joint.GetFirstDeriv(j))
            vels.append(vel)
        return vels

    def getInternalJointDOFFirstDerivesLocalFlat(self):
        vel = []
        for i in range(1, len(self._nodes)):
            for j in range(self._nodes[i].joint.GetDOF()):
                vel.append(self._nodes[i].joint.GetFirstDeriv(j))
        return np.array(vel)

    def getJointTransform(self, key):
        joint = None
        if isinstance(key, str):
            joint = self.skeleton.joint("j_"+key)
        elif isinstance(key, int):
            joint = self.skeleton.joint(key)
        else:
            raise TypeError

        return joint.get_local_transform()

    # Get Joint Global State
    def getJointFrame(self, index):
        # return SE3_2_pySE3(self._nodes[index].body.GetFrame() * Inv(self._boneTs[index]))
        return None

    def getJointPositionGlobal(self, index):
        if index == 0:
            bodyFrame = self.skeleton.body(index).world_transform()
            return np.dot(bodyFrame, npl.inv(self._boneTs[index]))[:3, 3]
        else:
            return self.skeleton.joint(index).position_in_world_frame()

    def getJointVelocityGlobal(self, index):
        return self.getBodyVelocityGlobal(index, npl.inv(self._boneTs[index])[:3, 3])

    def getJointAccelerationGlobal(self, index):
        # pospos = Inv(self._boneTs[index]).GetPosition()
        # return Vec3_2_pyVec3(self.getBodyAccelerationGlobal(index, pospos))
        return None

    def getJointOrientationGlobal(self, index):
        return self.getJoint(index).orientation_in_world_frame()

    def getJointAngVelocityGlobal(self, index):
        # angVel = self._nodes[index].body.GetAngVelocity()
        # parentIndex = self.getParentIndex(index)
        # if parentIndex == -1:
        #     parentAngVel = Vec3(0., 0., 0.)
        # else:
        #     parentAngVel = self._nodes[parentIndex].body.GetAngVelocity()
        #     return Vec3_2_pyVec3(angVel - parentAngVel)
        return self.getBodyAngVelocityGlobal(index)

    def getJointAngAccelerationGlobal(self, index):
        return self.getBodyAngAccelerationGlobal(index)

    # Get Joints Global State
    def getJointPositionsGlobal(self):
        return [self.getJointPositionGlobal(i) for i in range(len(self._nodes))]

    def getJointVelocitiesGlobal(self):
        return [self.getJointVelocityGlobal(i) for i in range(len(self._nodes))]

    def getJointAcclerationsGlobal(self):
        return [self.getJointAccelerationGlobal(i) for i in range(len(self._nodes))]

    def getJointOrientationsGlobal(self):
        return [self.getJointOrientationGlobal(i) for i in range(len(self._nodes))]

    def getJointAngVelocitiesGlobal(self):
        return [self.getJointAngVelocityGlobal(i) for i in range(len(self._nodes))]

    def getJointAngAccelerationsGlobal(self):
        return [self.getJointAngAccelerationGlobal(i) for i in range(len(self._nodes))]

    def getInternalJointPositionsGlobal(self):
        return [self.getJointPositionGlobal(i) for i in range(1, len(self._nodes))]

    def getInternalJointOrientationsGlobal(self):
        return [self.getJointOrientationGlobal(i) for i in range(1, len(self._nodes))]

    # get DOF states
    def getDOFPositions(self):
        # ls = self.getInternalJointOrientationsLocal()
        # rootFrame = SE3_2_pySE3(self._nodes[0].body.GetFrame() * Inv(self._boneTs[0]))
        # ls.insert(0, rootFrame)
        # return ls
        ls = []
        jointFrame = self.skeleton.body(0).world_transform().dot(npl.inv(self._boneTs[0]))
        for i in range(1, len(self.skeleton.joints)):
            pass


        ls = self.getInternalJointOrientationsLocal()
        rootFrame = self._nodes[0].body.GetFrame() * Inv(self._boneTs[0])
        pyV = Vec3_2_pyVec3(rootFrame.GetPosition())
        pyR = SE3_2_pySO3(rootFrame)

        ls.insert(0, (pyV, pyR))

        return ls

    def getDOFVelocities(self):
        rootGenVel = np.zeros(6)
        rootGenVel[0:3] = self.getJointVelocityGlobal(0)
        # rootGenVel[3:6] = self.getJointAngVelocityGlobal(0)
        rootGenVel[3:6] = self.getJointAngVelocityLocal(0)

        ls = self.getInternalJointAngVelocitiesLocal()
        ls.insert(0, rootGenVel)

        return ls

    def getDOFAccelerations(self):
        rootGenAcc = np.zeros(6)
        rootGenAcc[0:3] = self.getJointAccelerationGlobal(0)
        # rootGenVel[3:6] = self.getJointAngAcclerationGlobal(0)
        rootGenAcc[3:6] = self.getJointAngAccelerationLocal(0)

        ls = self.getInternalJointAngAccelerationsLocal()
        ls.insert(0, rootGenAcc)

        return ls

    def getDOFAxeses(self):
        rootAxeses = np.vstack((np.eye(3), np.eye(3)))
        rootAxesTmp = self.getBodyOrientationGlobal(0)
        rootAxes = rootAxesTmp.T
        rootAxeses[3] = rootAxes[0]
        rootAxeses[4] = rootAxes[1]
        rootAxeses[5] = rootAxes[2]

        lsTmp = self.getInternalJointOrientationsGlobal()
        ls = [tmp.T for tmp in lsTmp]
        ls.insert(0, rootAxeses)
        return ls

    def getDOFPositionsLocal(self):
        # ls = self.getInternalJointOrientationsLocal()
        # rootFrame = SE3_2_pySE3(self._nodes[0].body.GetFrame() * Inv(self._boneTs[0]))
        # ls.insert(0, rootFrame)
        # return ls

        ls = self.getInternalJointOrientationsLocal()
        rootFrame = self._nodes[0].body.GetFrame() * Inv(self._boneTs[0])
        pyV = Vec3_2_pyVec3(-Inv(rootFrame).GetPosition())
        pyR = SE3_2_pySO3(rootFrame)
        ls.insert(0, (pyV, pyR))

    def getDOFVelocitiesLocal(self):
        rootGenVel = np.zeros(6)
        # rootGenVel[0:3] = self.getJointVelocityGlobal(0)
        # rootGenVel[3:6] = self.getJointAngVelocityGlobal(0)
        rootGenVel[0:3] = self.getJointVelocityLocal(0)
        rootGenVel[3:6] = self.getJointAngVelocityLocal(0)

        ls = self.getInternalJointAngVelocitiesLocal()

        ls.insert(0, rootGenVel)
        return ls

    def getDOFAccelerationsLocal(self):
        rootGenAcc = np.zeros(6)
        # rootGenAcc[0:3] = self.getJointAccelerationGlobal(0)
        # rootGenVel[3:6] = self.getJointAngAcclerationGlobal(0)
        rootGenAcc[0:3] = self.getJointAccelerationLocal(0)
        rootGenAcc[3:6] = self.getJointAngAccelerationLocal(0)

        ls = self.getInternalJointAngAccelerationsLocal()
        ls.insert(0, rootGenAcc)

        return ls

    def getDOFAxesesLocal(self):
        rootAxeses = np.vstack((np.eye(3), np.eye(3)))
        rootAxesTmp = self.getBodyOrientationGlobal(0)
        rootAxes = rootAxesTmp.T
        rootAxeses[0] = rootAxes[0]
        rootAxeses[1] = rootAxes[1]
        rootAxeses[2] = rootAxes[2]
        rootAxeses[3] = rootAxes[0]
        rootAxeses[4] = rootAxes[1]
        rootAxeses[5] = rootAxes[2]

        lsTmp = self.getInternalJointOrientationsLocal()
        ls = [tmp.T for tmp in lsTmp]
        ls.insert(0, rootAxeses)
        return ls

    def getBodyRootDOFVelocitiesLocal(self):
        rootGenVel = self.getBodyGenVelLocal(0)
        rootGenVel_swaped = np.hstack((rootGenVel[3:], rootGenVel[:3]))
        return [rootGenVel_swaped] + self.getInternalJointAngVelocitiesLocal()

    def getBodyRootDOFAccelerationsLocal(self):
        rootGenAcc = self.getBodyGenAccLocal(0)
        rootGenAcc_swaped = np.hstack((rootGenAcc[3:], rootGenAcc[:3]))
        return [rootGenAcc_swaped] + self.getInternalJointAngAccelerationsLocal()

    def getBodyRootDOFAxeses(self):
        rootAxeses = np.vstack((np.eye(3), np.eye(3)))
        rootAxesTmp = self.getBodyOrientationGlobal(0)
        rootAxes = rootAxesTmp.T
        rootAxeses[0] = rootAxes[0]
        rootAxeses[1] = rootAxes[1]
        rootAxeses[2] = rootAxes[2]
        rootAxeses[3] = rootAxes[0]
        rootAxeses[4] = rootAxes[1]
        rootAxeses[5] = rootAxes[2]

        internalJointOrientations = self.getInternalJointOrientationsGlobal()
        ls = [rootAxeses]
        for jointOrientation in internalJointOrientations:
            ls.append(jointOrientation.T)

        return ls

    # get dof function for jacobian and mass matrix
    def getBodyRootDOFFirstDerivs(self):
        rootGenVel = self.getBodyGenVelLocal(0)
        rootGenVel_swaped = np.hstack((rootGenVel[3:], rootGenVel[:3]))
        return [rootGenVel_swaped] + self.getInternalJointDOFFirstDerivesLocal()

    def getBodyRootJointAngJacobiansGlobal(self):
        rootAxes = self.getBodyOrientationGlobal(0)
        rootJacobian = mm.getLocalAngJacobianForAngleAxis(mm.logSO3(rootAxes))

        # rootAxeses = np.hstack((rootAxes, np.dot(rootAxes, rootJacobian))).T
        rootAxeses = np.hstack((rootAxes, rootAxes)).T

        ls = [rootAxeses] \
             +[np.dot(self.getJointOrientationGlobal(i), self.getJointLocalAngJacobian(i)).T for i in range(1, len(self._nodes))]
        return ls

    def getJointLocalAngJacobian(self, i):
        _joint = self._nodes[i].joint
        if self._nodes[i].dof == 1:
            return Vec3_2_pyVec3(_joint.GetAxis()).reshape([3, 1])
            # return Vec3_2_pyVec3(_joint.GetAxis())
        elif self._nodes[i].dof == 2:
            # TODO:
            # check whether GetAxis is appropriate or not
            axis0 = Vec3_2_pyVec3(_joint.GetAxis(0))
            # axis0 = Vec3_2_pyVec3(_joint.GetLocalAxis(0))
            axis1 = Vec3_2_pyVec3(_joint.GetAxis(1))
            return np.vstack((axis0, axis1)).T
            # return np.vstack((axis0, axis1))
        elif self._nodes[i].dof == 3:
            m_rQ = np.array([_joint.GetDisplacement(0), _joint.GetDisplacement(1), _joint.GetDisplacement(2)])
            return mm.getLocalAngJacobianForAngleAxis(m_rQ)

    # set Joints
    def setJointAngVelocityLocal(self, index, angvel):
        if index == 0:
            genVelBodyLocal = self._nodes[index].body.GetGenVelocityLocal()
            genVelJointLocal = InvAd(Inv(self._boneTs[index]), genVelBodyLocal)
            # genVelJointLocal[0] = angvel[0]
            # genVelJointLocal[1] = angvel[1]
            # genVelJointLocal[2] = angvel[2]

            _genVelJointLocal = se3(angvel[0], angvel[1], angvel[2], genVelJointLocal[3], genVelJointLocal[4], genVelJointLocal[5])

            genVelBodyLocal = Ad(Inv(self._boneTs[index]), _genVelJointLocal)
            self._nodes[index].body.SetGenVelocityLocal(genVelBodyLocal)
        else:
            self._nodes[index].joint.SetVelocity(pyVec3_2_Vec3(angvel))

    def setJointAngAccelerationLocal(self, index, angacc):
        if index == 0:
            genAccBodyLocal = self._nodes[index].body.GetGenAccelerationLocal()
            genAccJointLocal = InvAd(Inv(self._boneTs[index]), genAccBodyLocal)
            # genAccJointLocal[0] = angacc[0]
            # genAccJointLocal[1] = angacc[1]
            # genAccJointLocal[2] = angacc[2]

            _genAccJointLocal = se3(angacc[0], angacc[1], angacc[2], genAccJointLocal[3], genAccJointLocal[4], genAccJointLocal[5])

            genVelBodyLocal = Ad(Inv(self._boneTs[index]), genAccJointLocal)
            self._nodes[index].body.SetGenAccelerationLocal(genAccBodyLocal)
        else:
            self._nodes[index].joint.SetAcceleration(pyVec3_2_Vec3(angacc))

    def setJointAccelerationGlobal(self, index, acc):
        if index == 0:
            pospos = Inv(self._boneTs[index]).GetPosition()
            self.setBodyAccelerationGlobal(index, pyVec3_2_Vec3(acc), pospos)
        else:
            print "setJointAccelerationGlobal() : not completely implemented"

    def setJointAngAccelerationGlobal(self, index, angacc):
        self.setBodyAngAccelerationGlobal(index, angacc)

    def setJointAngAccelerationsLocal(self, angaccs):
        for i in range(len(self._nodes)):
            self.setJointAngAccelerationLocal(i, angaccs[i])

    def setInternalJointAngAccelerationsLocal(self, angaccs):
        for i in range(1, len(self._nodes)):
            # self._nodes[i].joint.SetAcceleration(pyVec3_2_Vec3(angaccs[i-1]))
            self._nodes[i].joint.SetAccelerationLocal(pyVec3_2_Vec3(angaccs[i-1]))

    def setDOFAccelerations(self, dofaccs):
        self.setJointAccelerationGlobal(0, dofaccs[0][0:3])
        # self.setJointAngAccelerationGlobal(0, dofaccs[0][3:6])
        self.setJointAngAccelerationLocal(0, dofaccs[0][3:6])
        self.setInternalJointAngAccelerationsLocal(dofaccs[1:])

    def setDOFAccelerationsFlatFromExtendDOF(self, dofaccs):
        self.setJointAccelerationGlobal(0, dofaccs[0:3])
        self.setJointAngAccelerationLocal(0, dofaccs[3:6])
        for i in range(1, len(self._nodes)):
            self._nodes[i].joint.SetAccelerationLocal(pyVec3_2_Vec3(dofaccs[3+3*i:6+3*i]))

    def setDOFGenAccelerationsFlat(self, dofaccs):
        # self.setJointAccelerationGlobal(0, dofaccs[0:3])
        # self.setJointAngAccelerationLocal(0, dofaccs[3:6])
        curIdx = 6
        for i in range(1, len(self._nodes)):
            node = self._nodes[i]
            for j in range(node.dof):
                node.joint.SetSecondDeriv(j, dofaccs[curIdx])
                curIdx += 1

    def setDOFTorques(self, dofTorque):
        for i in range(1, len(self._nodes)):
            self._nodes[i].joint.SetTorqueLocal(pyVec3_2_Vec3(dofTorque[i-1]))

    def setDOFTorquesFlatFromExtendDOF(self, dofTorque):
        # all joints are treated as 3dof
        for i in range(1, len(self._nodes)):
            self._nodes[i].joint.SetTorqueLocal(pyVec3_2_Vec3(dofTorque[3*(i-1):3*i]))

    def setDOFGenTorquesFlat(self, dofTorque):
        # compact dof representation
        curIdx = 0
        for i in range(1, len(self._nodes)):
            for j in range(self._nodes[i].dof):
                self._nodes[i].joint.SetGenTorque(j, dofTorque[curIdx])
                curIdx += 1
                # self._nodes[i].joint.SetTorqueLocal(pyVec3_2_Vec3(dofTorque[i-1]))

    def setDOFGenTorquesFlatFromExtendDOF(self, dofTorque):
        #
        curIdx = 0
        for i in range(1, len(self._nodes)):
            for j in range(self._nodes[i].dof):
                self._nodes[i].joint.SetGenTorque(j, dofTorque[curIdx])
                curIdx += 1
            curIdx += 3 - self._nodes[i].dof

    def setDOFAccelerationsFromExtendDOF(self, dofaccs):
        self.setJointAccelerationGlobal(0, dofaccs[0][0:3])
        # self.setJointAngAccelerationGlobal(0, dofaccs[0][3:6])
        self.setJointAngAccelerationLocal(0, dofaccs[0][3:6])
        curIdx = 0
        for i in range(1, len(self._nodes)):
            for j in range(self._nodes[i].dof):
                self._nodes[i].joint.SetGenTorque(j, dofaccs[curIdx])
                curIdx += 1
                # self._nodes[i].joint.SetTorqueLocal(pyVec3_2_Vec3(dofTorque[i-1]))

    def SetJointElasticity(self, index, Kx, Ky=None, Kz=None):
        assert (Kx < 0. and "Joint Elasticity must larger than 0")
        k = None
        if Ky is None:
            k = Inertia(Kx)
        else:
            k = Inertia(0., Kx, Ky, Kz)
        self._nodes[index].joint.SetElasticity(k)

    def SetJointsElasticity(self, Kx, Ky, Kz):
        for i in range(1, len(self._nodes)):
            self.SetJointElasticity(i, Kx, Ky, Kz)

    def SetJointDamping(self, index, Dx, Dy=None, Dz=None):
        assert (Dx >= 0. and "Joint Elasticity must larger than 0")
        if self._nodes[index].dof == 3:
            d = None
            if Dy is None:
                d = Inertia(Dx)
            else:
                d = Inertia(0., Dx, Dy, Dz)
            self._nodes[index].joint.SetDamping(d)
        elif self._nodes[index].dof == 2:
            self._nodes[index].joint.SetDamping(0, Dx)
            self._nodes[index].joint.SetDamping(1, Dy)
        elif self._nodes[index].dof == 1:
            self._nodes[index].joint.SetDamping(Dx)

    def SetJointsDamping(self, Dx, Dy=None, Dz=None):
        if Dy is None:
            for i in range(1, len(self._nodes)):
                self.SetJointDamping(i, Dx, Dx, Dx)
        else:
            for i in range(1, len(self._nodes)):
                self.SetJointDamping(i, Dx, Dy, Dz)

    def getJointTorqueLocal(self, index):
        if index == 0:
            return None
        return Vec3_2_pyVec3(self._nodes[index].joint.GetTorque())

    def getInternalJointTorquesLocal(self):
        return [self.getJointTorqueLocal(i) for i in range(1, len(self._nodes))]

    def setJointTorqueLocal(self, index, torque):
        self._nodes[index].joint.SetTorqueLocal(pyVec3_2_Vec3(torque))

    def setInternalJointTorquesLocal(self, torques):
        for i in range(1, len(self._nodes)):
            self._nodes[i].joint.SetTorqueLocal(pyVec3_2_Vec3(torques[i-1]))

    def applyBodyGenForceGlobal(self, index, torque, force, positionLocal=None):
        if positionLocal is None:
            self._nodes[index].body.ApplyGlobalForce(dse3(torque[0], torque[1], torque[2], force[0], force[1], force[2]), Vec3(0., 0., 0.))
        else:
            self._nodes[index].body.ApplyGlobalForce(dse3(torque[0], torque[1], torque[2], force[0], force[1], force[2]), pyVec3_2_Vec3(positionLocal))

    def applyBodyForceGlobal(self, index, force, positionLocal=None):
        if positionLocal is None:
            self._nodes[index].body.ApplyGlobalForce(dse3(0., 0., 0., force[0], force[1], force[2]), Vec3(0., 0., 0.))
        else:
            self._nodes[index].body.ApplyGlobalForce(dse3(0., 0., 0., force[0], force[1], force[2]), pyVec3_2_Vec3(positionLocal))

    def applyBodyTorqueGlobal(self, index, torque):
        self._nodes[index].body.ApplyGlobalForce(dse3(torque[0], torque[1], torque[2], 0., 0., 0.), Vec3(0., 0., 0.))

    def getBodyForceLocal(self, index):
        genForce = self._nodes[index].body.GetForce()
        pyV = np.zeros(3)
        pyV[0] = genForce[3]
        pyV[1] = genForce[4]
        pyV[2] = genForce[5]
        return pyV

    def getBodyNetForceLocal(self, index):
        genForce = self._nodes[index].body.GetNetForce()
        pyV = np.zeros(3)
        pyV[0] = genForce[3]
        pyV[1] = genForce[4]
        pyV[2] = genForce[5]
        return pyV

    def getBodyGravityForceLocal(self, index):
        genForce = self._nodes[index].body.GetGravityForce()
        pyV = np.zeros(3)
        pyV[0] = genForce[3]
        pyV[1] = genForce[4]
        pyV[2] = genForce[5]
        return pyV

    # Additional
    def id2index(self, idid):
        index = 0
        for i in range(self.getBodyNum()):
            if idid == self.index2id(i):
                index = i
                break
        return index

    def SetBodyColor(self, idid, r, g, b, a):
        index = self.id2index(idid)
        for i in range(len(self.skeleton.body(index).num_shapenodes())):
            if self.skeleton.body(index).shapenodes[i].has_visual_aspect():
                self.skeleton.body(index).shapenodes[i].set_visual_aspect_rgba([r, g, b, a])

    def setSpring(self, body1Idx, body2Idx, ela, damp, p1, p2, dist):
        spring = vpSpring()
        spring.Connect(self._nodes[body1Idx].body, self._nodes[body2Idx].body, pyVec3_2_Vec3(p1), pyVec3_2_Vec3(p2))
        spring.SetElasticity(ela)
        spring.SetDamping(damp)
        spring.SetInitialDistance(dist)
        self._springs.append(spring)

    # must be called at first:clear all torques and accelerations
    def getInverseEquationOfMotion(self):
        # M^-1 * tau - M^-1 * b = ddq
        # ddq^T = [rootjointLin^T rootjointAng^T joints^T]^T
        n = len(self._nodes)-1
        N = 6 + self.getTotalInternalJointDOF()

        invMb = np.zeros(N)
        invM = np.zeros((N, N))

        zero_dse3 = dse3(0.)
        zero_Vec3 = Vec3(0.)

        Hip = self._nodes[0].body
        HipFrame = Hip.GetFrame()
        HipGenDOF = Vec3_2_pyVec3(LogR(HipFrame))
        HipGenVel = se3_2_pyVec6(Hip.GetGenVelocityLocal())
        HipAngVel = HipGenVel[:3]
        HipLinVel = HipGenVel[3:]
        HipGenDOFVel = angvel_2_genangvel(HipAngVel, HipGenDOF)

        # save current ddq and tau
        accBackup = []
        torBackup = []
        for i in range(n):
            self._nodes[i+1].joint.BackupAccTau()

        hipAccBackup = Hip.ResetForce()
        for i in range(1, len(self._nodes)):
            self._nodes[i].body.ResetForce()
            self._nodes[i].joint.SetTorqueLocal(zero_Vec3)


        # get invMb
        Hip.ApplyLocalForce(zero_dse3, zero_Vec3)
        for i in range(n):
            self._nodes[i+1].joint.SetTorqueLocal(zero_Vec3)

        Hip.GetSystem().ForwardDynamics()
        hipAcc_tmp = se3_2_pyVec6(Hip.GetGenAccelerationLocal())   # represented in body frame

        invMb[:3] = -hipAcc_tmp[3:]
        invMb[3:6] = -hipAcc_tmp[:3]
        # invMb[3:6] = -angacc_2_genangacc(hipAcc_tmp[:3], HipGenDOF, HipGenDOFVel, HipAngVel)
        invMb[6:] = -self.getInternalJointDOFSecondDerivesLocalFlat()

        # get M
        for i in range(N):
            Hip.ResetForce()
            for j in range(1, len(self._nodes)):
                self._nodes[j].body.ResetForce()
                dof = self._nodes[j].dof
                for k in range(dof):
                    self._nodes[j].joint.SetGenTorque(k, 0.)
                    # self._nodes[j].joint.SetTorqueLocal(zero_Vec3)

            _torque = np.zeros(N)
            if i<3:
                _torque[i+3] = 1.
            elif i<6:
                _torque[i-3] = 1.
            else:
                _torque[i] = 1.

            genForceLocal = pyVec6_2_dse3(_torque[:6])

            curJointTorqueIdx = 6
            for j in range(n):
                dof = self._nodes[j+1].joint.GetDOF()
                if 0 <= i-curJointTorqueIdx < dof:
                    self._nodes[j+1].joint.SetGenTorque(i-curJointTorqueIdx, 1.)
                    break
                curJointTorqueIdx += dof

            Hip.ApplyLocalForce(genForceLocal, zero_Vec3)

            Hip.GetSystem().ForwardDynamics()

            hipAcc_tmp = se3_2_pyVec6(Hip.GetGenAccelerationLocal())
            invM[:3, i] = hipAcc_tmp[3:6] + invMb[:3]
            # invM[3:6, i] = angacc_2_genangacc(hipAcc_tmp[:3], HipGenDOF, HipGenDOFVel, HipAngVel) + invMb[3:6]
            invM[3:6, i] = hipAcc_tmp[:3] + invMb[3:6]
            # for j in range(3):
            #     invM[j, i] = hipAcc_tmp[j+3] + invMb[j]
            # for j in range(3, 6):
            #    # invM[j, i] = hipAcc_tmp[j-3] + invMb[j]
            #    invM[j, i] = angacc_2_genangacc(hipAcc_tmp[:3], HipGenDOF, angvel_2_genangvel(HipAngVel, HipGenDOF), HipAngVel) + invMb[j]

            invM[6:, i] = self.getInternalJointDOFSecondDerivesLocalFlat() + invMb[6:]

            # for j in range(n):
            #     joint = self._nodes[j+1].joint
            #     acc = joint.GetAcceleration()
            #     for k in range(3):
            #         invM[6+3*j+k, i] = acc[k] + invMb[6+3*j+k]

        # restore ddq and tau
        for i in range(n):
            self._nodes[i+1].joint.RestoreAccTau()
            self._nodes[i+1].joint.SetAccelerationLocal(zero_Vec3)
            self._nodes[i+1].joint.SetTorqueLocal(zero_Vec3)

        # Hip.SetGenAcceleration(hipAccBackup)

        Hip.ResetForce()
        # Hip.ApplyGlobalForce(hipTorBackup, zero_Vec3)

        return invM, invMb

    # must be called at first:clear all torques and accelerations
    def getInverseEquationOfMotion_deprecated(self):
        # M^-1 * tau - M^-1 * b = ddq
        # ddq^T = [rootjointLin^T rootjointAng^T joints^T]^T
        n = len(self._nodes)-1
        N = 6 + 3*n

        invMb = np.zeros(N)
        invM = np.zeros((N, N))

        zero_dse3 = dse3(0.)
        zero_Vec3 = Vec3(0.)

        Hip = self._nodes[0].body

        # save current ddq and tau
        accBackup = []
        torBackup = []
        for i in range(n):
            joint = self._nodes[i+1].joint
            accBackup.append(joint.GetAcceleration())
            torBackup.append(joint.GetTorque())

        # hipAccBackup = Hip.ResetForce()
        Hip.ResetForce()
        for i in range(len(self._nodes)):
            self._nodes[i].body.ResetForce()
            self._nodes[i].joint.SetTorque(zero_Vec3)

        # get invMb
        Hip.ApplyLocalForce(zero_dse3, zero_Vec3)
        for i in range(n):
            joint = self._nodes[i+1].joint
            joint.SetTorque(zero_Vec3)

        Hip.GetSystem().ForwardDynamics()
        hipAcc_tmp = Hip.GetGenAccelerationLocal()   # represented in body frame

        invMb[0] = -hipAcc_tmp[3]
        invMb[1] = -hipAcc_tmp[4]
        invMb[2] = -hipAcc_tmp[5]
        invMb[3] = -hipAcc_tmp[0]
        invMb[4] = -hipAcc_tmp[1]
        invMb[5] = -hipAcc_tmp[2]

        for i in range(n):
            joint = self._nodes[i+1].joint
            acc = joint.GetAcceleration()
            for j in range(3):
                invMb[6+3*i+j] = -acc[j]

        # get M
        for i in range(N):
            Hip.ResetForce()
            for j in range(len(self._nodes)):
                self._nodes[j].body.ResetForce()
                self._nodes[j].joint.SetTorque(zero_Vec3)

            _torque = np.zeros(N)
            if i < 3:
                _torque[i + 3] = 1.
            elif i < 6:
                _torque[i - 3] = 1.
            else:
                _torque[i] = 1.

            genForceLocal = pyVec6_2_dse3(_torque[:6])

            for j in range(n):
                torque = pyVec3_2_Vec3(_torque[6+3*j:9+3*j])
                self._nodes[j+1].joint.SetTorque(torque)

            Hip.ApplyLocalForce(genForceLocal, zero_Vec3)

            Hip.GetSystem().ForwardDynamics()

            hipAcc_tmp = Hip.GetGenAccelerationLocal()
            for j in range(3):
                invM[j, i] = hipAcc_tmp[j+3] + invMb[j]
            for j in range(3, 6):
                invM[j, i] = hipAcc_tmp[j-3] + invMb[j]
            for j in range(n):
                joint = self._nodes[j+1].joint
                acc = joint.GetAcceleration()
                for k in range(3):
                    invM[6+3*j+k, i] = acc[k] + invMb[6+3*j+k]

        # restore ddq and tau
        for i in range(n):
            joint = self._nodes[i+1].joint
            joint.SetAcceleration(accBackup[i])
            joint.SetTorque(torBackup[i])
            joint.SetAcceleration(zero_Vec3)
            joint.SetTorque(zero_Vec3)

        # Hip.SetGenAcceleration(hipAccBackup)

        Hip.ResetForce()
        # Hip.ApplyGlobalForce(hipTorBackup, zero_Vec3)

        return invM, invMb

    def stepKinematics(self, dt, accs):
        Hip = self._nodes[0].body
        hipacc = pyVec3_2_Vec3(accs[0][0:3])
        hipangacc = pyVec3_2_Vec3(accs[0][3:6])

        genAccBodyLocal = se3(hipangacc, hipacc)
        genVelBodyLocal = Hip.GetGenVelocityLocal() + genAccBodyLocal * dt
        Hip.SetGenVelocityLocal(genVelBodyLocal)
        rootFrame = Hip.GetFrame() * Exp(dt * genVelBodyLocal)
        Hip.SetFrame(rootFrame)

        for i in range(1, len(self._nodes)):
            ddq = pyVec3_2_Vec3(accs[i])
            joint = self._nodes[i].joint
            dq = joint.GetVelocity() + ddq * dt
            joint.SetVelocity(dq)
            q = joint.GetOrientation() * Exp(Axis(dq * dt))
            joint.SetOrientation(q)

        for i in range(len(self._nodes)):
            self._nodes[i].body.UpdateGeomFrame()

        Hip.ResetForce()

        '''
        zero_se3 = se3(0.)
        for i in range(len(self._nodes)):
            self._nodes[i].body.ResetForce()
            self._nodes[i].body.SetGenAcceleration(zero_se3)
        '''

