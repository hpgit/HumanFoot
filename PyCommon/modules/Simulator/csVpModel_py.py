#define MAX_X 1 // 0001
#define MAX_Y 2 // 0010
#define MAX_Z 4 // 0100

import sys
import math
import numpy as np

sys.path.append('../../..')

from PyCommon.modules.pyVirtualPhysics import *
from PyCommon.modules.Simulator.csVpUtil import *
from PyCommon.modules.Simulator.myGeom import *

from PyCommon.modules.Math import mmMath as mm
import PyCommon.modules.Simulator.ysPhysConfig as ypc
from PyCommon.modules.Motion import ysMotion as ym

QP = True


class VpModel:
    """
    :type _pWorld : vpWorld
    :type _config : ypc.ModelConfig
    :type _skeleton: ym.JointSkeleton
    :type _nodes : list[VpModel.Node]
    :type _boneTs : list[SE3]
    :type _name2index : dict[str, int]
    :type _id2index : dict[int, int]
    """
    class Node:
        """
        :type name : str
        :type body : vpBody
        :type material : vpMaterial
        :type joint : hpBJoint | hpRJoint | hpUJoint | None
        :type dof : int
        :type color : list[int]
        :type use_joint : bool
        :type geoms : list[MyBox | MyFoot3 | MyFoot4 | MyFoot5]
        """
        def __init__(self, _name):
            """

            :param _name: str
            """
            if _name is not None:
                self.name = _name
                self.body = vpBody()
                self.material = vpMaterial()
                # self.joint = hpBJoint()
                self.joint = None
                self.dof = 3
                self.color = [0., 0., 0., 255.]
                self.use_joint = False
                self.body.SetMaterial(self.material)
                self.geoms = []

        def setJointNearstOrientation(self, R):
            """
            :param R: SE3
            :return: None
            """
            if self.dof == 3:
                self.joint.SetOrientation(R)
            elif self.dof == 2:
                th0 = math.atan2(-R[7], R[8])
                th1 = math.asin(R[6])
                self.joint.SetAngle(0, th0)
                self.joint.SetAngle(1, th1)
            elif self.dof == 1:
                th0 = Inner(LogR(R), self.joint.GetAxis())
                self.joint.SetAngle(th0)


    def __init__(self, pWorld, createPosture, config):
        if pWorld is not None:
            self._pWorld = pWorld._world
            self._config = config
            self._skeleton = createPosture.skeleton

            num = createPosture.skeleton.getJointNum()

            self._nodes = [None] * num
            # self._nodes = [self.Node(None)]*num
            self._boneTs = [SE3()] * num

            self._name2index = dict()
            self._id2index = dict()

            self.createBodies(createPosture)
            self.build_name2index()

    def build_name2index(self):
        for i in range(len(self._nodes)):
            self._name2index[self._nodes[i].name] = i

    def getBodyNum(self):
        return len(self._nodes)

    def index2name(self, index):
        return self._nodes[index].name

    def index2id(self, index):
        return self._nodes[index].body.GetID()

    def name2index(self, name):
        try:
            return self._name2index[name]
        except:
            return -1

    def name2id(self, name):
        return self.index2id(self.name2index(name))

    def SetGround(self, index, flag):
        self._nodes[index].body.SetGround(flag)

    def createBodies(self, posture):
        """

        :param posture: ym.JointPosture
        :return:
        """
        joint = posture.skeleton.root
        rootPos = posture.rootPos
        T = SE3(Vec3(rootPos[0], rootPos[1], rootPos[2]))
        tpose = posture.getTPose()
        self._createBody(joint, T, tpose)

    def _createBody(self, joint, parentT, posture):
        """

        :param joint: ym.Joint
        :param parentT: SE3
        :param posture: ym.JointPosture
        :return:
        """
        len_joint_children = len(joint.children)
        if len_joint_children == 0:
            return

        T = parentT
        P = SE3(Vec3(joint.offset[0], joint.offset[1], joint.offset[2]))
        T = T * P

        joint_name = joint.name
        # joint_index = skeleton.getElementIndex(joint_name)
        # R = posture.getLocalR(joint_index)
        joint_index = posture.skeleton.getJointIndex(joint_name)
        R = posture.getJointOrientationLocal(joint_index)
        T = T * pySO3_2_SE3(R)

        if self._config.hasNode(joint_name):
            offset = Vec3(0.)
            for i in range(len_joint_children):
                offset += pyVec3_2_Vec3(joint.children[i].offset)

            # if joint.parent is None:
            offset *= 1./len_joint_children

            boneT = SE3(offset*.5)

            defaultBoneV = Vec3(0, 0, 1)
            boneR = getSE3FromVectors(defaultBoneV, offset)

            boneT = boneT * boneR

            pNode = self.Node(joint_name)
            self._nodes[joint_index] = pNode
            # pNode = self._nodes[joint_index]
            # pNode.name = joint_name
            cfgNode = self._config.getNode(joint_name)

            numGeom = len(cfgNode.geoms)
            if numGeom > 0:
                for i in range(numGeom):
                    geomType = cfgNode.geoms[i]
                    if geomType == "MyFoot3" or geomType == "MyFoot4" or geomType == "MyFoot5":
                        density = cfgNode.geomMaterial[i].density
                        radius = cfgNode.geomMaterial[i].radius
                        height = cfgNode.geomMaterial[i].height

                        if height <= 0.:
                            height = Norm(offset) + 2.*radius
                        
                        pNode.material.SetDensity(density)
                        pNode.body.SetMaterial(pNode.material)

                        geomT = SE3()
                        geomT.SetEye()
                        if cfgNode.geomTs[i] is not None:
                            geomT = pySO3_2_SE3(cfgNode.geomTs[i][1])
                            geomT.SetPosition(geomT*pyVec3_2_Vec3(cfgNode.geomTs[i][0]))
                        else:
                            print("there is no geom Ts!")

                        geom = None
                        #TODO:
                        if geomType == "MyFoot3":
                            geom = MyFoot3(radius, height)
                        elif geomType == "MyFoot4":
                            geom = MyFoot4(radius, height)
                        else:
                            geom = MyFoot5(radius, height)

                        pNode.geoms.append(geom)
                        pNode.body.AddGeometry(geom, geomT)
                    else:
                        width = cfgNode.geomMaterial[i].width
                        length = cfgNode.geomMaterial[i].length
                        height = cfgNode.geomMaterial[i].height

                        pNode.material.SetDensity(cfgNode.geomMaterial[i].density)
                        pNode.body.SetMaterial(pNode.material)

                        geomT = SE3()
                        geomT.SetEye()
                        if cfgNode.geomTs[i] is not None:
                            geomT = pySO3_2_SE3(cfgNode.geomTs[i][1])
                            geomT.SetPosition(geomT*pyVec3_2_Vec3(cfgNode.geomTs[i][0]))
                        geom = vpBox(Vec3(width, height, length))
                        pNode.geoms.append(geom)
                        pNode.body.AddGeometry(geom, geomT)

            else:
                geomType = cfgNode.geom
                if (geomType == "MyFoot3") or (geomType == "MyFoot4") or geomType == "MyFoot5":
                    radius = .05
                    if cfgNode.width is not None:
                        radius = cfgNode.width
                    length = Norm(offset) + 2*radius
                    density = cfgNode.density
                    mass = 1.
                    if cfgNode.mass is not None:
                        mass = cfgNode.mass
                        density = mass/(radius*radius*M_PI*length)
                    else:
                        mass = density * radius * radius * M_PI * length

                    # TODO:
                    geom = None
                    #TODO:
                    if geomType == "MyFoot3":
                        geom = MyFoot3(radius, length)
                    elif geomType == "MyFoot4":
                        geom = MyFoot4(radius, length)
                    else:
                        geom = MyFoot5(radius, length)

                    pNode.geoms.append(geom)
                    pNode.body.AddGeometry(geom)
                    pNode.body.SetInertia(CylinderInertia(density, radius, length))

                else:
                    length = 1.
                    if cfgNode.length is not None:
                        length = cfgNode.length * cfgNode.boneRatio
                    else:
                        length = Norm(offset) * cfgNode.boneRatio

                    density = cfgNode.density
                    width = .1
                    height = .1

                    if cfgNode.width is not None:
                        width = cfgNode.width
                        if cfgNode.mass is not None:
                            height = cfgNode.mass / (density*length*width)
                        else:
                            height = .1
                    else:
                        if cfgNode.mass is not None:
                            width = math.sqrt(cfgNode.mass/(density*length))
                        else:
                            width = .1
                        height = width

                    box = None
                    if geomType == "MyBox":
                        box = MyBox(Vec3(width, height, length))
                    else:
                        box = vpBox(Vec3(width, height, length))
                    pNode.geoms.append(box)
                    pNode.body.AddGeometry(box)
                    pNode.body.SetInertia(BoxInertia(density, Vec3(width/2., height/2., length/2.)))

            boneT = boneT * SE3(pyVec3_2_Vec3(cfgNode.offset))
            self._boneTs[joint_index] = boneT
            newT = T * boneT

            pNode.body.SetFrame(newT)
            self._id2index[pNode.body.GetID()] = joint_index

        for i in range(len_joint_children):
            self._createBody(joint.children[i], T, posture)

    def __str__(self):
        '''
        //	ss << "<NODES>" << endl;
        //	for(int i=0; i<_nodes.size(); ++i)
        //	{
              //		ss << "[" << i << "]:";
        ////		if(_nodes[i]==NULL)
                ////			ss << "NULL, ";
        ////		else
        //			ss << _nodes[i]->name << ", ";
        //	}
        //	ss << endl;
        //
        //	ss << "<BODIES INDEX:(NODE INDEX) NODE NAME>\n";
        //	for(int i=0; i<_bodyElementIndexes.size(); ++i)
        //		ss << "[" << i << "]:(" << _bodyElementIndexes[i] << ") " << _nodes[_bodyElementIndexes[i]]->name << ", ";
        //	ss << endl;

        '''
        strstr = "<BODIES (,JOINTS)>\n"
        for i in range(len(self._nodes)):
            strstr += "[" + str(i) + "]" + self._nodes[i].name + ", \n"
        strstr += "\n"
        strstr = "<BODY MASSES>\n"
        for i in range(len(self._nodes)):
            strstr += "[" + str(i) + "]" + str(self._nodes[i].body.GetInertia().GetMass()) + ", \n"
        strstr += "\n"
        '''
        //	ss << "<BODY INERTIAS>" << endl;
        //	ss << "I11 I22 I33 I12 I13 I23 offset.x offset.y offset.z mass" << endl;
        //	for(int i=0; i<_nodes.size(); ++i)
        //		if(_nodes[i])
            //		{
                      //			ss << "[" << i << "]:";
        //			for(int j=0; j<10; ++j)
        //				ss << _nodes[i]->body.GetInertia()[j] << " ";
        //			ss << endl;
        //		}
        //	ss << endl;
        '''
        return strstr

    def getBodyMasses(self):
        ls = []
        for i in range(len(self._nodes)):
            ls.append(self._nodes[i].body.GetInertia().GetMass())
        return ls

    def getTotalMass(self):
        mass = 0.
        for i in range(len(self._nodes)):
            mass += self._nodes[i].body.GetInertia().GetMass()
        return mass

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
        iner = self._nodes[index].body.GetInertia()
        # Tin = SE3()
        Tin = np.zeros((3, 3))

        Tin[0, 0] = iner[0]
        Tin[1, 1] = iner[1]
        Tin[2, 2] = iner[2]
        Tin[0, 1] = Tin[1, 0] = iner[3]
        Tin[0, 2] = Tin[2, 0] = iner[4]
        Tin[1, 2] = Tin[2, 1] = iner[5]

        return Tin

    def getBodyInertiaGlobal(self, index):
        Tin = SE3()
        iner = self._nodes[index].body.GetInertia()
        Tin[0] = iner[0]
        Tin[4] = iner[1]
        Tin[8] = iner[2]
        Tin[3] = Tin[1] = iner[3]
        Tin[6] = Tin[2] = iner[4]
        Tin[7] = Tin[5] = iner[5]
        bodyFrame = self._nodes.body.GetFrame()
        return bodyFrame * Tin * Inv(bodyFrame)

    def getBodyInertiasLocal(self):
        ls = []
        for i in range(len(self._nodes)):
            ls.append(self.getBodyInertiaLocal(i))

        return ls

    def getBodyInertiasGlobal(self):
        ls = []
        for i in range(len(self._nodes)):
            ls.append(self.getBodyInertiaGlobal(i))

        return ls

    def getCOM(self):
        com = Vec3(0., 0., 0.)
        for i in range(len(self._nodes)):
            com += self._nodes[i].body.GetInertia().GetMass() * self._nodes[i].body.GetFrame().GetPosition()
        com *= 1./self.getTotalMass()

        return com
    
    def getBoneT(self, index):
        return [SE3_2_pySO3(self._boneTs[index]), Vec3_2_pyVec3(self._boneTs[index].GetPosition())]

    def getInvBoneT(self, index):
        return [SE3_2_pySO3(Inv(self._boneTs[index])), Vec3_2_pyVec3(Inv(self._boneTs[index]).GetPosition())]

    def getBodyGenVelLocal(self, index):
        return se3_2_pyVec6(self._nodes[index].body.GetGenVelocityLocal())
    
    def getBodyGenVelGlobal(self, index):
        return se3_2_pyVec6(self._nodes[index].body.GetGenVelocity())

    def getBodyGenAccLocal(self, index):
        return se3_2_pyVec6(self._nodex[index].body.GetGenAccelerationLocal())

    def getBodyGenAccGlobal(self, index):
        return se3_2_pyVec6(self._nodes[index].body.GetGenAccleration())
    
    def getBodyPositionGlobal(self, index, pPositionLocal=None):
        bodyFrame = self._nodes[index].body.GetFrame()
        if pPositionLocal is None:
            return Vec3_2_pyVec3(bodyFrame.GetPosition())
        return Vec3_2_pyVec3(bodyFrame * pPositionLocal)

    def getBodyOrientationGlobal(self, index):
        bodyFrame = self._nodes[index].body.GetFrame()

        return SE3_2_pySO3(bodyFrame)

    def getBodyVelocityGlobal(self, index, positionLocal=None):
        if positionLocal is None:
            return self._nodes[index].body.GetLinVelocity(Vec3(0., 0., 0.))
        return Vec3_2_pyVec3(self._nodes[index].body.GetLinVelocity(pyVec3_2_Vec3(positionLocal)))

        '''
        //	static se3 genAccLocal, genAccGlobal;
        //	genAccLocal = _nodes[index]->body.GetGenVelocityLocal();
        //	genAccLocal = MinusLinearAd(positionLocal, genAccLocal);
        //	genAccGlobal = Rotate(_nodes[index]->body.GetFrame(), genAccLocal);
        //	return Vec3(genAccGlobal[3], genAccGlobal[4], genAccGlobal[5]);
        '''

    def getBodyVelocitiesGlobal(self):
        ls = []
        for i in range(len(self._nodes)):
            ls.append(self.getBodyVelocityGlobal(i))
        return ls

    def getBodyAngVelocityGlobal(self, index):
        # se3 genVel;
        genVel = self._nodes[index].body.GetGenVelocity()
        return se3_2_pyVec6(genVel)[:3]

    def getBodyAngVelocitiesGlobal(self):
        ls = []
        for i in range(len(self._nodes)):
            ls.append(self.getBodyVelocityGlobal(i))
        return ls

    def getBodyAccelerationGlobal(self, index, pPositionLocal=None):
        # se3 genAccLocal, genAccGlobal
        genAccLocal = self._nodes[index].body.GetGenAccelerationLocal()
        if pPositionLocal is not None:
            genAccLocal = MinusLinearAd(pPositionLocal, genAccLocal)

        genAccGlobal = Rotate(self._nodes[index].body.GetFrame(), genAccLocal)

        return Vec3(genAccGlobal[3], genAccGlobal[4], genAccGlobal[5])

    def getBodyAccelerationsGlobal(self):
        ls = []
        for i in range(len(self._nodes)):
            ls.append(self.getBodyAccelerationGlobal(i))
        return ls

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
        self._nodes[index].body.SetGenAcceleration(genAcc)

    def getBodyPositionsGlobal(self):
        return [self.getBodyPositionGlobal(i) for i in range(len(self._nodes))]

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
        for i in range(len(self._nodes)):
            self.setBodyPositionGlobal(i, self.getBodyPositionGlobal(i) + offset)

    def rotate(self, rotation):
        R = pySO3_2_SE3(rotation)

        bodyFrame = self._nodes[0].body.GetFrame()
        self._nodes[0].body.SetFrame(bodyFrame * R)

        self._pWorld.UpdateFrame()

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


class VpMotionModel(VpModel):
    def __init__(self, pWorld, createPosture, config):
        VpModel.__init__(self, pWorld, createPosture, config)
        if pWorld is not None:
            self._recordVelByFiniteDiff = False
            self._inverseMotionTimeStep = 30.

            self.update(createPosture)
            # self.addBody(False)

    def recordVelByFiniteDiff(self, flag=True, motionTimeStep=1./30.):
        self._recordVelByFiniteDiff = flag
        self._inverseMotionTimeStep = 1./motionTimeStep

    def update(self, posture):
        """

        :param posture: ym.JointPosture
        :return:
        """
        joint = posture.skeleton.root
        rootPos = posture.rootPos
        T = SE3(pyVec3_2_Vec3(rootPos))
        self._updateBody(joint, T, posture)

    def _updateBody(self, joint, parentT, posture):
        """

        :param joint: ym.Joint
        :param parentT: SE3
        :param posture: ym.JointPosture
        :return:
        """

        len_joint_children = len(joint.children)
        if len_joint_children == 0:
            return

        P = SE3(pyVec3_2_Vec3(joint.offset))
        T = parentT * P

        joint_name = joint.name
        # joint_index = posture.skeleton.getElementIndex(joint_name)
        # R = pySO3_2_SE3(posture.getLocalR(joint_index))
        joint_index = posture.skeleton.getJointIndex(joint_name)
        R = pySO3_2_SE3(posture.getJointOrientationLocal(joint_index))

        T = T * R
        # if (len_joint_children > 0 ) and self._config.hasNode(joint_name):
        if self._config.hasNode(joint_name):
            boneT = self._boneTs[joint_index]
            newT = T * boneT

            pNode = self._nodes[joint_index]

            if self._recordVelByFiniteDiff:
                oldT = pNode.body.GetFame()
                diffT = newT * Inv(oldT)

                p = newT.GetPosition() - oldT.GetPosition()
                diffT.SetPosition(p)

                pNode.body.SetGenVelocity(Log(diffT) * self._inverseMotionTimeStep)

            pNode.body.SetFrame(newT)

        for i in range(len_joint_children):
            self._updateBody(joint.children[i], T, posture)


class VpControlModel(VpModel):
    """
    :type _springs : vpSpring
    """
    def __init__(self, pWorld, createPosture, config):
        VpModel.__init__(self, pWorld, createPosture, config)
        if pWorld is not None:
            self.addBodiesToWorld(createPosture)
            self.ignoreCollisionBtwnBodies()
            pWorld.addVpModel(self)

            tpose = createPosture.getTPose()
            self.createJoints(tpose)

            self.update(createPosture)
            # self.addBody(True)

            self._springs = []

    def __str__(self):
        strstr = VpModel.__str__(self)
        strstr += "<INTERNAL JOINTS>\n"
        for i in range(1, len(self._nodes)):
            strstr += "[" + str(i-1) + "]:" + self._nodes[i].name + ", " + str(self._nodes[i].dof) + "DOF\n"
        strstr += "\n"

        return strstr

    def getInternalJointDOFs(self):
        # ls = [3] * (len(self._nodes)-1)
        # for i in range(1, len(self._nodes)):
        #     ls.append(3)
        ls = [self._nodes[i].dof for i in range(1, len(self._nodes))]
        return ls

    def getTotalInternalJointDOF(self):
        # return 3*(len(self._nodes)-1)
        return sum(self.getInternalJointDOFs())

    def getDOFs(self):
        ls = [6]
        ls.extend(self.getInternalJointDOFs())
        return ls

    def getTotalDOF(self):
        return 6 + self.getTotalInternalJointDOF()

    def get3dExtendTotalDOF(self):
        return 6 + 3*(len(self._nodes)-1)

    def get3dExtendDOFs(self):
        return [[6]] + [[3]]*(len(self._nodes)-1)

    def createJoints(self, posture):
        """

        :param posture: ym.JointPosture
        :return:
        """
        joint = posture.skeleton.root
        self._createJoint(joint, posture)

    def _createJoint(self, joint, posture):
        """

        :param joint: ym.Joint
        :param posture: ym.JointPosture
        :return:
        """
        len_joint_children = len(joint.children)
        if len_joint_children == 0:
            return

        offset = joint.offset
        P = SE3(pyVec3_2_Vec3(joint.offset))
        joint_name = joint.name
        # joint_index = posture.skeleton.getElementIndex(joint_name)
        # R = pySO3_2_SE3(posture.getLocalR(joint_index))
        joint_index = posture.skeleton.getJointIndex(joint_name)
        R = pySO3_2_SE3(posture.getJointOrientationLocal(joint_index))

        invLocalT = Inv(R) * Inv(P)

        temp_joint = joint
        nodeExistParentJoint = None
        while True:
            if temp_joint.parent is None:
                nodeExistParentJoint = None
                break
            else:
                temp_parent_name = temp_joint.parent.name
                # temp_parent_index = posture.skeleton.getElementIndex(temp_parent_name)
                temp_parent_index = posture.skeleton.getJointIndex(temp_parent_name)

                if self._nodes[temp_parent_index] is not None:
                    nodeExistParentJoint = temp_joint.parent
                    break
                else:
                    temp_joint = temp_joint.parent

                    offset = temp_joint.offset
                    P = SE3(pyVec3_2_Vec3(offset))

                    joint_name = temp_joint.name
                    localSO3 = posture.localRs[joint_index]
                    R = pySO3_2_SE3(localSO3)

                    invLocalT = invLocalT * Inv(R) * Inv(P)

        # len_joint_children = len(joint.children)

        # if (nodeExistParentJoint is not None) \
        #        and (len_joint_children > 0) \
        #        and self._config.hasNode(joint_name):
        if nodeExistParentJoint is None:
            self._nodes[joint_index].dof = 6

        elif self._config.hasNode(joint_name):
            pNode = self._nodes[joint_index]
            cfgNode = self._config.getNode(joint_name)

            parent_name = nodeExistParentJoint.name
            # parent_index = posutre.skeleton.getElementIndex(parent_name)
            parent_index = posture.skeleton.getJointIndex(parent_name)
            pParentNode = self._nodes[parent_index]
            parentCfgNode = self._config.getNode(parent_name)

            # offset = cfgNode.offset
            # offsetT = SE3(pyVec3_2_Vec3(offset))

            # parentOffset = parentCfgNode.offset
            # parentOffsetT = SE3(pyVec3_2_Vec3(parentOffset))

            if cfgNode.jointType == "R":
                pNode.dof = 1
                pNode.joint = hpRJoint()
            elif cfgNode.jointType == "U":
                pNode.dof = 2
                pNode.joint = hpUJoint()
                pNode.joint.SetAxis(1, Vec3(0., 0., 1.))
            elif cfgNode.jointType == "B":
                pNode.dof = 3
                pNode.joint = hpBJoint()
            else:
                pNode.dof = 3
                pNode.joint = hpBJoint()

            pParentNode.body.SetJoint(pNode.joint, Inv(self._boneTs[parent_index]) * Inv(invLocalT))
            pNode.body.SetJoint(pNode.joint, Inv(self._boneTs[joint_index]))

            kt = 16.
            dt = 8.
            el = Inertia(kt)
            dam = Inertia(dt)
            # pNode.joint.SetElasticity(el)
            # pNode.joint.SetDamping(dam)
            pNode.use_joint = True

        for i in range(len_joint_children):
            self._createJoint(joint.children[i], posture)

    def ignoreCollisionBtwnBodies(self):
        for i in range(len(self._nodes)):
            for j in range(i, len(self._nodes)):
                self._pWorld.IgnoreCollision(self._nodes[i].body, self._nodes[j].body)

    def addBodiesToWorld(self, createPosture):
        self._pWorld.AddBody(self._nodes[0].body)

    def update(self, posture):
        """

        :param posture: ym.JointPosture
        :return:
        """
        joint = posture.skeleton.root
        self._updateJoint(joint, posture)

    def _updateJoint(self, joint, posture):
        """

        :param joint: ym.Joint
        :param posture: ym.JointPosture
        :return:
        """
        len_joint_children = len(joint.children)
        if len_joint_children == 0:
            return

        P = SE3(pyVec3_2_Vec3(joint.offset))
        joint_name = joint.name
        # joint_index = posture.skeleton.getElementIndex(joint_name)
        # R = pySO3_2_SE3(posture.getLocalR(joint_index))
        joint_index = posture.skeleton.getJointIndex(joint_name)
        R = pySO3_2_SE3(posture.getJointOrientationLocal(joint_index))

        invLocalT = Inv(R) * Inv(P)

        temp_joint = joint
        nodeExistParentJoint = None
        while True:
            if temp_joint.parent is None:
                nodeExistParentJoint = None
                break
            else:
                temp_parent_name = temp_joint.parent.name
                # temp_parent_index = posture.skeleton.getElementIndex(temp_parent_name)
                temp_parent_index = posture.skeleton.getJointIndex(temp_parent_name)

                if self._nodes[temp_parent_index] is not None:
                    nodeExistParentJoint = temp_joint.parent
                    break
                else:
                    temp_joint = temp_joint.parent

                    offset = temp_joint.offset
                    P = SE3(pyVec3_2_Vec3(offset))

                    joint_name = temp_joint.name
                    localSO3 = posture.localRs[joint_index]
                    R = pySO3_2_SE3(localSO3)

                    invLocalT = invLocalT * Inv(R) * Inv(P)

        # len_joint_children = len(joint.children)

        # if (len_joint_children > 0) and self._config.hasNode(joint_name):
        if self._config.hasNode(joint_name):
            pNode = self._nodes[joint_index]
            if nodeExistParentJoint is not None:
                # pNode.joint.SetOrientation(R)
                pNode.setJointNearstOrientation(R)
            else:
                pNode.body.SetFrame(SE3(pyVec3_2_Vec3(posture.rootPos)) * P * R * self._boneTs[joint_index])

        for i in range(len_joint_children):
            self._updateJoint(joint.children[i], posture)

    def fixBody(self, index):
        self._nodes[index].body.SetGround()

    def initializeHybridDynamics(self, flotingBase=True):
        rootIndex = 0
        for i in range(len(self._nodes)):
            if i == rootIndex:
                if flotingBase:
                    self._nodes[i].body.SetHybridDynamicsType(DYNAMIC)
                else:
                    self._nodes[i].body.SetHybridDynamicsType(KINEMATIC)
            else:
                self._nodes[i].joint.SetHybridDynamicsType(KINEMATIC)

    def makeDOFFlatList(self):
        return [None] * self.getTotalDOF()

    def makeDOFNestedList(self):
        ls = []
        for node in self._nodes:
            ls.append([None]*node.dof)
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

    def initializeForwardDynamics(self):
        for i in range(len(self._nodes)):
            self._nodes[i].body.SetHybridDynamicsType(DYNAMIC)

    def solveHybridDynamics(self):
        self._nodes[0].body.GetSystem().HybridDynamics()

    def solveForwardDynamics(self):
        self._nodes[0].body.GetSystem().ForwardDynamics()

    def solveInverseDynamics(self):
        self._nodes[0].body.GetSystem().InverseDynamics()

    # Get Joint Local State
    def getJointOrientationLocal(self, index):
        if index == 0:
            return self.getJointOrientationGlobal(index)
        return SE3_2_pySO3(self._nodes[index].joint.GetOrientation())

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

    # Get Joint Global State
    def getJointFrame(self, index):
        return SE3_2_pySE3(self._nodes[index].body.GetFrame() * Inv(self._boneTs[index]))

    def getJointPositionGlobal(self, index):
        if self._nodes[index] is not None:
            bodyFrame = self._nodes[index].body.GetFrame()
            return Vec3_2_pyVec3((bodyFrame * Inv(self._boneTs[index])).GetPosition())
        '''
        parent = self._skeleton.getParentJointIndex(index)
        parentJointFrame = self._nodes[parent].body.GetFrame() * Inv(self._boneTs[parent])
        offset = pyVec3_2_Vec3(self._skeleton.getOffset(index))
        return Vec3_2_pyVec3(parentJointFrame * offset)
        '''

    def getJointVelocityGlobal(self, index):
        return Vec3_2_pyVec3(self.getBodyVelocityGlobal(index, Inv(self._boneTs[index]).GetPosition()))

    def getJointAccelerationGlobal(self, index):
        pospos = Inv(self._boneTs[index]).GetPosition()
        return Vec3_2_pyVec3(self.getBodyAccelerationGlobal(index, pospos))

    def getJointOrientationGlobal(self, index):
        bodyFrame = self._nodes[index].body.GetFrame()
        return SE3_2_pySO3(bodyFrame * Inv(self._boneTs[index]))

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

        ls = [rootAxeses]\
             +[np.dot(self.getJointOrientationGlobal(i), self.getJointLocalAngJacobian(i)).T for i in range(1, len(self._nodes))]
        return ls

    def getJointLocalAngJacobian(self, i):
        _joint = self._nodes[i].joint
        if self._nodes[i].dof == 1:
            return Vec3_2_pyVec3(_joint.GetAxis()).reshape([3, 1])
            # return Vec3_2_pyVec3(_joint.GetAxis())
        elif self._nodes[i].dof == 2:
            axis0 = Vec3_2_pyVec3(_joint.GetAxis(0))
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
        pNode = self._nodes[index]
        pNode.color[:] = [r, g, b, a]

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
        HipAngVel = Vec3_2_pyVec3(Hip.GetAngVelocity())

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
            joint = self._nodes[i+1].joint
            joint.SetTorqueLocal(zero_Vec3)

        Hip.GetSystem().ForwardDynamics()
        hipAcc_tmp = se3_2_pyVec6(Hip.GetGenAccelerationLocal())   # represented in body frame

        invMb[:3] = -hipAcc_tmp[3:]
        # invMb[3:6] = -hipAcc_tmp[:3]
        invMb[3:6] = -angacc_2_genangacc(hipAcc_tmp[:3], HipGenDOF, angvel_2_genangvel(HipAngVel, HipGenDOF), HipAngVel)
        invMb[6:] = -self.getInternalJointDOFSecondDerivesLocalFlat()

        # get M
        for i in range(N):
            Hip.ResetForce()
            for j in range(1, len(self._nodes)):
                self._nodes[j].body.ResetForce()
                self._nodes[j].joint.SetTorqueLocal(zero_Vec3)

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
            invM[3:6, i] = angacc_2_genangacc(hipAcc_tmp[:3], HipGenDOF, angvel_2_genangvel(HipAngVel, HipGenDOF), HipAngVel) + invMb[3:6]
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
            joint.SetAccelerationLocal(zero_Vec3)
            joint.SetTorqueLocal(zero_Vec3)

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
