import math

from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.VirtualPhysics.LieGroup import *
from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Resource import ysMotionLoader as yf

from PyCommon.modules.Simulator import ysPhysConfig as ypc

import xml.etree.ElementTree as et
from xml.dom import minidom

import copy


def prettifyXML(elem):
    """
    Return a pretty-printed XML string for the Element.
    :param elem: et.Element
    :return: str
    """
    rough_string = et.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="    ")


def SE32vlogSO3(T):
    v = np.append(T[0:3, 3].flatten(), mm.logSO3(T[0:3, 0:3]))
    return v


def SE32vlogSO3str(T):
    text = ""
    v = SE32vlogSO3(T)
    for j in range(6):
        text += str(v[j]) + " "
    return text


def SE32veulerXYZ(T):
    v = np.append(T[0:3, 3].flatten(), mm.R2XYZ(T[0:3, 0:3]))
    return v


def SE32veulerXYZstr(T):
    text = ""
    v = SE32veulerXYZ(T)
    for j in range(6):
        text += str(v[j]) + " "
    return text


class DsModelMaker:
    """
    :type config : ypc.ModelConfig
    """
    def __int__(self):
        """

        :param config: ypc.ModelConfig
        :return:
        """
        self.config = None
        self.bvh = None
        self.skelname = None

    def createBodies(self, posture):
        """

        :param posture: ym.JointPosture
        :return:
        """
        joint = posture.skeleton.root
        # rootPos = SE3(posture.rootPos)
        rootPos = SE3()
        tpose = posture.getTPose()
        return self._createBody(joint, rootPos, tpose)

    def _createBody(self, joint, parentT, posture):
        """

        :param joint: ym.Joint
        :param parentT: SE3
        :param posture: ym.JointPosture
        :return: list[str], list[SE3], list[Vec3], list[SE3]
        """
        Ts = []
        names = []
        offsets = []
        boneTs = []
        len_joint_children = len(joint.children)
        if len_joint_children == 0:
            return names, Ts, offsets, boneTs

        P = SE3(joint.offset)
        T = parentT * P

        joint_name = joint.name
        names.append(joint_name)
        joint_index = posture.skeleton.getJointIndex(joint_name)
        R = posture.getJointOrientationLocal(joint_index)
        T = T * SE3(R)

        offset = Vec3(0.)
        for i in range(len_joint_children):
            offset += Vec3(joint.children[i].offset)

        offset *= 1./len_joint_children

        boneT = SE3(offset * .5)

        defaultBoneV = Vec3(0., 0., 1.)
        boneR = SE3(mm.getSO3FromVectors(defaultBoneV, offset))

        boneT = boneT * boneR

        if self.config is not None:
            if self.config.hasNode(joint_name):
                boneT = boneT * SE3(Vec3(self.config.getNode(joint_name).offset))
        boneTs.append(boneT)
        offsets.append(offset)

        newT = T * boneT

        Ts.append(newT)

        for i in range(len_joint_children):
            childNames, childTs, childOffsets, childBoneTs = self._createBody(joint.children[i], T, posture)
            names.extend(childNames)
            Ts.extend(childTs)
            offsets.extend(childOffsets)
            boneTs.extend(childBoneTs)

        return names, Ts, offsets, boneTs


    def createJoints(self, posture, boneTs):
        """

        :param posture: ym.JointPosture
        :param boneTs: SE3
        :return:
        """
        joint = posture.skeleton.root
        return self._createJoint(joint, posture, boneTs)

        # TODO:
        # return body_to_joint list, [parent, child] list


    def _createJoint(self, joint, posture, boneTs):
        jointPairs = []
        bodyToJointTs = []
        jointTypes = []

        len_joint_children = len(joint.children)
        if len_joint_children == 0:
            return jointPairs, bodyToJointTs, jointTypes

        def Inv(T):
            return SE3(np.linalg.inv(T))

        # offset = joint.offset
        # P = SE3(joint.offset)
        joint_name = joint.name
        joint_index = posture.skeleton.getJointIndex(joint_name)
        # R = SE3(posture.getJointOrientationLocal(joint_index))

        # invLocalT = Inv(R) * Inv(P)

        temp_joint = joint
        nodeExistParentJoint = None

        if temp_joint.parent is None:
            nodeExistParentJoint = None
        else:
            nodeExistParentJoint = temp_joint.parent

        if nodeExistParentJoint is not None:
            parent_name = nodeExistParentJoint.name
            parent_index = posture.skeleton.getJointIndex(parent_name)

            # parentbodyToJointT = Inv(boneTs[parent_index]) * Inv(invLocalT)
            bodyToJointT = Inv(boneTs[joint_index])
            jointType = "ball"
            if self.config is not None:
                if self.config.hasNode(joint_name):
                    joint_type = self.config.getNode(joint_name).jointType
                    if joint_type == "B":
                        jointType = "gspherical"
                    elif joint_type == "U":
                        jointType = "universal"
                    elif joint_type == "R":
                        jointType = "revolute"

            jointPair = (parent_index, joint_index)
            jointPairs.append(jointPair)
            bodyToJointTs.append(bodyToJointT)
            jointTypes.append(jointType)
        else:
            jointPair = (-1, joint_index)
            jointPairs.append(jointPair)
            bodyToJointTs.append(Inv(boneTs[joint_index]))
            jointTypes.append("gfreemotion")


        for i in range(len_joint_children):
            childJointPairs, childbodyToJointTs, childJointTypes = self._createJoint(joint.children[i], posture, boneTs)
            jointPairs.extend(childJointPairs)
            bodyToJointTs.extend(childbodyToJointTs)
            jointTypes.extend(childJointTypes)

        return jointPairs, bodyToJointTs, jointTypes


    def AddDartShapeNode(self, T, size, geom):
        geomtext = ""
        if geom == "box":
            geomtext = "box"
        elif geom == "cylinder":
            geomtext = "cylinder"
        elif geom == "ellipsoid":
            geomtext = "ellipsoid"
        elif geom == "capsule":
            geomtext = "capsule"
        else:
            raise "geom type Error"

        etGeom = et.Element(
            'geom_info',
            {'type':'0'}
        )
        et.SubElement(
            etGeom, 'color',
            {'value':'0.2 0.7 0.7', 'type':'4', 'size':'1'}
        )
        if (geom == "box") or (geom == "ellipsoid"):
            et.SubElement(
                etGeom, 'size',
                {'value':' '.join(map(str, size)), 'type':'4', 'size':'1'}
            )
            # et.SubElement(etGeom, "size").text = " ".join(map(str, size))
        else:
            et.SubElement(etGeom, "radius").text = str(size[0])
            et.SubElement(etGeom, "height").text = str(size[1])

        et.SubElement(
            etGeom, 'type',
            {'value':geomtext, 'type':'4', 'size':'1'}
        )

        return etGeom


    def AddGeomCapsule(self, idx, density, height, radius):
        s = ''
        s += 'box['+str(idx)+'].getInertia()->setInertiaBox('+','.join(map(str, [density, height, radius]))+');\n'
        s += 'sml::rose::GeomCapsule *geom = new sml::rose::GeomCapsule();\n'
        s += 'geom->setLength('+str(height)+');\n'
        s += 'geom->setRadius('+str(radius)+');\n'
        s += 'geom->setColor(0.2, 0.7, 0.7, 1.0);\n'
        return s

    def AddGeomBox(self, idx, density, width, height, length):
        s = ''
        s += 'const sm::real geomSize[3] = {'+','.join(map(str, [width, height, length]))+'};\n'
        s += 'box['+str(idx)+'].getInertia()->setInertiaBox('+str(density)+',geomSize);\n'
        s += 'sml::rose::GeomBox *geom = new sml::rose::GeomBox();\n'
        s += 'geom->setSize(geomSize);\n'
        s += 'geom->setColor(0.2, 0.7, 0.7, 1.0);\n'
        return s


    def AddBody(self, idx, parentIdx, name, T, offset, inertia):
        """

        :param name: str
        :param T: SE3
        :param offset: Vec3
        :param inertia:
        :return:
        """
        s = '{\n'
        s += 'box['+str(idx)+'].setName("'+name+'");\n'
        s += 'const sm::real pos[3] = {'+','.join(map(str, T[:3, 3]))+'};\n'
        s += 'const sm::real rot[9] = {'+','.join(map(str, T[:3, :3].flatten()))+'};\n'
        s += 'box['+str(idx)+'].setInitPosition(pos);\n'
        s += 'box['+str(idx)+'].setInitRotation(rot);\n'

        cylLen_2 = np.linalg.norm(offset)/2.
        if self.config.hasNode(name):
            cfgNode = self.config.getNode(name)

            numGeom = len(cfgNode.geoms)
            if numGeom > 0:
                for i in range(numGeom):
                    geomType = cfgNode.geoms[i]
                    visualShapeCount = 0
                    collisionShapeCount = 0

                    if geomType == "MyFoot3" or geomType == "MyFoot4" or geomType == "MyFoot5":
                        # capsule case
                        density = cfgNode.geomMaterial[i].density
                        radius = cfgNode.geomMaterial[i].radius
                        height = cfgNode.geomMaterial[i].height

                        if height <= 0.:
                            height = np.linalg.norm(offset) + 2.*radius
                            cfgNode.geomMaterial[i].height = height

                        geomT = SE3()
                        if cfgNode.geomTs[i] is not None:
                            geomT = SE3(cfgNode.geomTs[i][1])
                            geomT.SetPosition(geomT*Vec3(cfgNode.geomTs[i][0]))
                        else:
                            print("there is no geom Ts!")

                        s += self.AddGeomCapsule(idx, density, height, radius)

                    else:
                        width = cfgNode.geomMaterial[i].width
                        length = cfgNode.geomMaterial[i].length
                        height = cfgNode.geomMaterial[i].height

                        geomT = SE3()
                        if cfgNode.geomTs[i] is not None:
                            geomT = SE3(cfgNode.geomTs[i][1])
                            geomT.SetPosition(geomT*Vec3(cfgNode.geomTs[i][0]))

                        s += self.AddGeomBox(idx, cfgNode.geomMaterial[i].density, width, height, length)

            else:
                geomType = cfgNode.geom
                visualShapeCount = 0
                collisionShapeCount = 0
                if (geomType == "MyFoot3") or (geomType == "MyFoot4") or geomType == "MyFoot5":
                    radius = .05
                    if cfgNode.width is not None:
                        radius = cfgNode.width
                    length = np.linalg.norm(offset) + 2.*radius
                    density = cfgNode.density
                    mass = 1.
                    if cfgNode.mass is not None:
                        mass = cfgNode.mass
                        density = mass/(radius*radius*math.pi*length)
                    else:
                        mass = density * radius * radius * math.pi * length

                    s += self.AddGeomCapsule(idx, density, length, radius)
                else:
                    length = 1.
                    if cfgNode.length is not None:
                        length = cfgNode.length * cfgNode.boneRatio
                    else:
                        length = np.linalg.norm(offset) * cfgNode.boneRatio

                    density = cfgNode.density
                    width = .1
                    height = .1
                    mass = density * width * height

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

                    s += self.AddGeomBox(idx, density, width, height, length)
        else:
            s += self.AddGeomBox(idx, 1000., .1, .1, 1.8*cylLen_2)


        s += 'box['+str(idx)+'].setGeomObject(geom);\n'
        s += '}\n'

        return s

    def setMainBvh(self, filename):
        self.bvh = yf.readBvhFileAsBvh(filename)
        self.skelname = filename.split("/")[-1].split(".")[0]

    def attachPartBvh(self, partFilePath, attachPart, scale=1.0, mirror=None):
        partBvh = yf.readBvhFileAsBvh(partFilePath)
        if mirror is not None:
            partBvh.mirror(mirror)
        self.bvh.replaceJointFromBvh(attachPart, partBvh, scale)

    def posture2DsSkel(self, posture, config=None, offsetR=np.eye(3)):
        self.config = config
        skeleton = posture.skeleton

        offsetT = mm.SO3ToSE3(offsetR)

        names, Ts, offsets, boneTs = self.createBodies(posture)
        jointPairs, bodyToJointTs, jointTypes = self.createJoints(posture, boneTs)

        s = \
            '#include <sml.h>\n' \
            '#include <map>\n' \
            '\n' \
            'void buildCharacter(sml::rose::RigidBodySystem *ch)\n' \
            '{\n' \
            'const int num_boxes = '+str(len(names))+';\n' \
            'sm::MemoryManager < sml::rose::RigidBody > mmrb;\n' \
            'sm::MemoryManager < sml::rose::GSphericalJoint > mmrj;\n' \
            'sm::MemoryManager < sml::rose::GeomBox > mmgb;\n' \
            'sml::rose::RigidBody * box = mmrb.malloc_arr(num_boxes);\n' \
            'sml::rose::GFreeMotionJoint        fjoint;\n' \
            'sml::rose::GSphericalJoint * rjoint = mmrj.malloc_arr(num_boxes);\n' \
            'std::map < int, int > bvhIdx2JointIdxMap;\n' \
            'int joint_count = 0;\n\n'


        # add Body
        for i in range(len(names)):
            # for i in range(1):
            # append body
            parentIdx = -1
            for jpair in jointPairs:
                if jpair[1] == i:
                    parentIdx = jpair[0]
            # s += self.AddBody(i, parentIdx, names[i], Ts[i], offsets[i], None)
            s += self.AddBody(i, parentIdx, names[i], np.dot(offsetT, Ts[i]), np.dot(offsetR, offsets[i]), None)

        # add Joint
        jointqidx = 0
        jointrqidx = 0
        # s += 'fjoint.setInitPosition'
        for i in range(len(jointPairs)):
            s += '{\n'
            jointPair = jointPairs[i]
            jointT = np.dot(Ts[jointPair[1]], bodyToJointTs[i])
            # jointT = np.dot(offsetT, np.dot(Ts[jointPair[1]], bodyToJointTs[i]))
            s += 'joint_count++;\n'
            s += 'const sm::real joint_pos[3] = {'+','.join(map(str, np.dot(offsetR, jointT[:3, 3])))+'};\n'
            s += 'const sm::real joint_rot[9] = {'+','.join(map(str, jointT[:3, :3].flatten()))+'};\n'
            if jointPair[0] == -1:
                s += 'fjoint.setInitPosition(joint_pos);\n'
                s += 'fjoint.setInitRotation(joint_rot);\n'
                s += 'box[0].setJoint(&fjoint);\n'
                s += 'fjoint.setName(box[0].getName());\n'
            else:
                s += 'rjoint['+str(jointPair[1])+'].setInitPosition(joint_pos);\n'
                s += 'rjoint['+str(jointPair[1])+'].setInitRotation(joint_rot);\n'
                s += 'box['+str(jointPair[1])+'].setJoint(&rjoint['+str(jointPair[1])+']);\n'
                s += 'box['+str(jointPair[1])+'].setParent(&box['+str(jointPair[0])+']);\n'
                s += 'rjoint['+str(jointPair[1])+'].setName("'+skeleton.getJointName(jointPair[1])+'");\n'

            jointdof = 3
            jointrqdim = 4

            if jointTypes[i] == "universal":
                etAxis1 = et.SubElement(etJoint, "axis")
                et.SubElement(etAxis1, "xyz").text = "1 0 0"
                etAxis2 = et.SubElement(etJoint, "axis2")
                et.SubElement(etAxis2, "xyz").text = "0 0 1"

                jointdof = 2
                jointrqdim = 0
            elif jointTypes[i] == "revolute":
                etAxis = et.SubElement(etJoint, "axis")
                et.SubElement(etAxis, "xyz").text = "1 0 0"
                jointdof = 1
                jointrqdim = 0
            elif jointTypes[i] == 'gfreemotion':
                jointdof = 6
                jointrqdim = 4
            elif jointTypes[i] == 'gspherical':
                jointdof = 3
                jointrqdim = 4


            # etJoint.append(et.Element('index', {'value':str(jointPair[1]), 'type':'4', 'size':'1'}))
            # etJoint.append(et.Element('name', {'value':skeleton.getJointName(jointPair[1]), 'type':'4', 'size':'1'}))
            # etJoint.append(et.Element('pos', {'value':' '.join([str(t) for t in jointT[:3, 3].flatten()]), 'type':'4', 'size':'1'}))
            # etJoint.append(et.Element('qdof', {'value':str(jointdof), 'type':'4', 'size':'1'}))
            # etJoint.append(et.Element('qidx', {'value':str(jointqidx), 'type':'4', 'size':'1'}))
            # etJoint.append(et.Element('rot', {'value':' '.join([str(r) for r in jointT[:3, :3].flatten()]), 'type':'4', 'size':'1'}))
            # etJoint.append(et.Element('rqdim', {'value':str(jointrqdim), 'type':'4', 'size':'1'}))
            # etJoint.append(et.Element('rqidx', {'value':str(jointrqidx), 'type':'4', 'size':'1'}))
            # etJoint.append(et.Element('rqtype', {'value':'quaternion', 'type':'4', 'size':'1'}))
            # etJoint.append(et.Element('type', {'value':jointTypes[i], 'type':'4', 'size':'1'}))

            jointqidx += jointdof
            jointrqidx += jointrqdim

            s += '}\n'
        s += 'ch->constructSystem(&box[0]);\n'
        s += 'ch->setGravAcc(0., 0., -9.81);\n'

        for i in range(len(jointPairs)):
            jointPair = jointPairs[i]
            s += 'for(int i=0; i<joint_count; i++)\n'
            s += '{\n'
            s += 'if (!strcmp("'+skeleton.getJointName(jointPair[1])+'", ch->getJoint(i)->getName()))\n'
            s += '{\n'
            s += 'bvhIdx2JointIdxMap['+str(jointPair[1])+'] = i;\n'
            s += 'break;\n'
            s += '}\n'
            s += '}\n'

        if True:
            s += 'sm::real *q = ch->getData(sml::rose::ROSE_RBS_DATA_q);\n'
            s += 'sm::real *rq = ch->getData(sml::rose::ROSE_RBS_DATA_rq);\n'
            for i in range(len(jointPairs)):
                s += '{\n'
                jointPair = jointPairs[i]
                # rotation = posture.getJointOrientationLocal(jointPair[1])
                rotation = np.dot(offsetR, np.dot(posture.getJointOrientationLocal(jointPair[1]), offsetR.T))
                quat = mm.R2Quat(rotation)
                s += 'sm::real quat[4] = {'+','.join(map(str, quat))+'};\n'
                s += 'QCOPY(&rq[ch->rqidx(bvhIdx2JointIdxMap['+str(jointPair[1])+'])], quat);\n'
                s += '}\n'

            globalRot = np.dot(offsetR, np.dot(posture.getJointOrientationGlobal(0), offsetR.T))
            globalPos = np.dot(offsetR, posture.getJointPositionGlobal(0))
            globalPos_in_body_coord = np.dot(globalRot.T, globalPos)

            s += 'q[4] = '+str(globalPos_in_body_coord[0])+';\n'
            s += 'q[5] = '+str(globalPos_in_body_coord[1])+';\n'
            s += 'q[6] = '+str(globalPos_in_body_coord[2])+';\n'
            s += 'ch->refreshState();\n'
        s += '}\n'


        # return etRootTree, boneTs
        return s

    def toDsSkelFile(self, skelfile, config):
        output = open(skelfile, "w")
        output.write(self.toDsSkelXmlStr(config))
        output.close()

    def toDsSkelXmlStr(self, config):
        jointMotion = self.bvh.toJointMotion(1., False)
        # jointMotion.rotateByOffset(mm.rotX(math.pi*0.5))
        return self.posture2DsSkel(jointMotion[0], config, mm.rotX(math.pi*.5))
        # print(prettifyXML(tree.getroot()))

    def posture2dartSkelFile(self, name, posture, skelfile, config):
        self.skelname = name
        tree, boneTs = self.posture2DsSkel(posture, config)
        output = open(skelfile, "w")
        output.write(prettifyXML(tree.getroot()))
        output.close()

    def posture2dartSkelXmlStr(self, name, posture, config):
        self.skelname = name
        tree, boneTs = self.posture2DsSkel(posture, config)
        # return prettifyXML(tree.getroot())
        # print prettifyXML(tree.getroot())
        return et.tostring(tree.getroot(), 'ascii'), boneTs

if __name__ == '__main__':
    def buildMassMap():
        massMap = {}
        massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips',
                                    'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector',
                                    'LeftLeg', 'LeftShoulder1', 'LeftUpLeg',
                                    'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector',
                                    'RightLeg', 'RightShoulder', 'RightUpLeg',
                                    'Spine', 'Spine1',
                                    'RightFoot_foot_0_0', 'RightFoot_foot_0_1', 'RightFoot_foot_0_1_Effector',
                                    'RightFoot_foot_1_0', 'RightFoot_foot_1_1', 'RightFoot_foot_1_1_Effector',
                                    'RightFoot_foot_2_0', 'RightFoot_foot_2_1', 'RightFoot_foot_2_1_Effector',
                                    'LeftFoot_foot_0_0', 'LeftFoot_foot_0_1', 'LeftFoot_foot_0_1_Effector',
                                    'LeftFoot_foot_1_0', 'LeftFoot_foot_1_1', 'LeftFoot_foot_1_1_Effector',
                                    'LeftFoot_foot_2_0', 'LeftFoot_foot_2_1', 'LeftFoot_foot_2_1_Effector',
                                    ], 0.)

        # torso : 10
        massMap['Hips'] += 2.
        massMap['Spine'] += 8.

        # head : 3
        massMap['Spine1'] += 3.

        # right upper arm : 2
        massMap['RightArm'] += 2.

        # left upper arm : 2
        massMap['LeftArm'] += 2.

        # right lower arm : 1
        massMap['RightForeArm'] = 1.
        #    massMap['RightForeArm'] = 2.

        # left lower arm : 1
        massMap['LeftForeArm'] = 1.
        #    massMap['LeftForeArm'] = 2.

        # right thigh : 7
        massMap['Hips'] += 2.
        massMap['RightUpLeg'] += 5.

        # left thigh : 7
        massMap['Hips'] += 2.
        massMap['LeftUpLeg'] += 5.

        # right shin : 5
        massMap['RightLeg'] += 5.

        # left shin : 5
        massMap['LeftLeg'] += 5.

        # right foot : 4
        massMap['RightFoot'] += 2.
        # massMap['RightFoot'] += .4

        # left foot : 4
        massMap['LeftFoot'] += 2.
        # massMap['LeftFoot'] += .4
        '''
        massMap['RightFoot_foot_0_0'] = .3
        massMap['RightFoot_foot_0_1'] = .3
        massMap['RightFoot_foot_1_0'] = .3
        massMap['RightFoot_foot_1_1'] = .3
        massMap['RightFoot_foot_2_0'] = .3
        massMap['RightFoot_foot_2_1'] = .3
        massMap['LeftFoot_foot_0_0'] = .3
        massMap['LeftFoot_foot_0_1'] = .3
        massMap['LeftFoot_foot_1_0'] = .3
        massMap['LeftFoot_foot_1_1'] = .3
        massMap['LeftFoot_foot_2_0'] = .3
        massMap['LeftFoot_foot_2_1'] = .3
        #'''

        massMap['RightFoot_foot_0_0'] = .3
        massMap['RightFoot_foot_0_1'] = .3
        massMap['RightFoot_foot_0_0_0'] = .3
        massMap['RightFoot_foot_0_1_0'] = .3
        massMap['RightFoot_foot_1_0'] = .3
        massMap['RightFoot_foot_1_1'] = .3
        massMap['RightFoot_foot_1_2'] = .3
        massMap['LeftFoot_foot_0_0'] = .3
        massMap['LeftFoot_foot_0_1'] = .3
        massMap['LeftFoot_foot_0_0_0'] = .3
        massMap['LeftFoot_foot_0_1_0'] = .3
        massMap['LeftFoot_foot_1_0'] = .3
        massMap['LeftFoot_foot_1_1'] = .3
        massMap['LeftFoot_foot_1_2'] = .3

        return massMap

    def buildMcfg():
        massMap = buildMassMap()
        mcfg = ypc.ModelConfig()
        mcfg.defaultDensity = 1000.
        mcfg.defaultBoneRatio = .9

        totalMass = 0.
        for name in massMap:
            node = mcfg.addNode(name)
            node.mass = massMap[name]
            # totalMass += node.mass

        node = mcfg.getNode('Hips')
        node.length = .2
        node.width = .25

        node = mcfg.getNode('Spine1')
        node.length = .2
        node.offset = (0,0,0.1)

        node = mcfg.getNode('Spine')
        node.width = .22

        node = mcfg.getNode('RightFoot')
        node.length = .25
        #    node.length = .27
        #    node.offset = (0,0,0.01)
        node.width = .1
        node.geom = 'MyFoot1'

        node = mcfg.getNode('LeftFoot')
        node.length = .25
        #    node.length = .27
        #    node.offset = (0,0,0.01)
        node.width = .1
        node.geom = 'MyFoot1'

        def capsulize(node_name):
            node = mcfg.getNode(node_name)
            node.geom = 'MyFoot4'
            node.width = 0.01
            # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., math.pi/4., 0.])], ypc.CapsuleMaterial(1000., .02, .2))
            # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., math.pi/4., 0.])], ypc.CapsuleMaterial(1000., .02, .1))
            # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., 0., 0.])], ypc.CapsuleMaterial(1000., .01, -1))
            # node.addGeom('MyFoot4', None, ypc.CapsuleMaterial(1000., .02, .1))

        # capsulize('RightFoot')
        # capsulize('LeftFoot')

        '''
        node = mcfg.getNode('RightFoot')
        node.density = 200.
        node.geom = 'MyFoot5'
        node.width = 0.01
        # node.jointType = 'U'

        node = mcfg.getNode('LeftFoot')
        node.density = 200.
        node.geom = 'MyFoot5'
        node.width = 0.01
        # node.jointType = 'U'
        '''

        # bird foot
        # capsulize('RightFoot_foot_0_0')
        # capsulize('RightFoot_foot_0_1')
        # capsulize('RightFoot_foot_1_0')
        # capsulize('RightFoot_foot_1_1')
        # capsulize('RightFoot_foot_2_0')
        # capsulize('RightFoot_foot_2_1')
        # capsulize('LeftFoot_foot_0_0')
        # capsulize('LeftFoot_foot_0_1')
        # capsulize('LeftFoot_foot_1_0')
        # capsulize('LeftFoot_foot_1_1')
        # capsulize('LeftFoot_foot_2_0')
        # capsulize('LeftFoot_foot_2_1')


        # human foot
        capsulize('RightFoot_foot_0_0')
        node = mcfg.getNode('RightFoot_foot_0_0')
        node.addGeom('MyFoot3', [0.02*np.array([-0.3, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])], ypc.CapsuleMaterial(400., .01, 0.02*2.5+0.02))
        node.addGeom('MyFoot3', [0.02*np.array([-0.3-1.2, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])], ypc.CapsuleMaterial(400., .01, 0.02*2.5+0.02))
        # node.addGeom('MyFoot4', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0., 0., 0.])], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = 'R'

        capsulize('RightFoot_foot_0_0_0')
        node = mcfg.getNode('RightFoot_foot_0_0_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.addGeom('MyFoot4', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.jointType = 'R'

        capsulize('RightFoot_foot_0_1')
        node = mcfg.getNode('RightFoot_foot_0_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.addGeom('MyFoot3', [0.02*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(400.,.01, -1))
        node.jointType = 'R'

        capsulize('RightFoot_foot_0_1_0')
        node = mcfg.getNode('RightFoot_foot_0_1_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.addGeom('MyFoot4', [0.02*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.jointType = 'R'

        capsulize('RightFoot_foot_1_0')
        node = mcfg.getNode('RightFoot_foot_1_0')
        node.addGeom('MyFoot3', [0.02*np.array([0., 0., .7]), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, 0.02*2.0+0.02))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = 'R'

        capsulize('RightFoot_foot_1_1')
        node = mcfg.getNode('RightFoot_foot_1_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.jointType = 'R'

        capsulize('RightFoot_foot_1_2')
        node = mcfg.getNode('RightFoot_foot_1_2')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.jointType = 'R'


        capsulize('LeftFoot_foot_0_0')
        node = mcfg.getNode('LeftFoot_foot_0_0')
        node.addGeom('MyFoot3', [0.02*np.array([0.3, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])], ypc.CapsuleMaterial(400., .01, 0.02*2.5+0.02))
        node.addGeom('MyFoot3', [0.02*np.array([0.3+1.2, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])], ypc.CapsuleMaterial(400., .01, 0.02*2.5+0.02))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = 'R'

        capsulize('LeftFoot_foot_0_0_0')
        node = mcfg.getNode('LeftFoot_foot_0_0_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.addGeom('MyFoot4', [0.02*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.jointType = 'R'

        capsulize('LeftFoot_foot_0_1')
        node = mcfg.getNode('LeftFoot_foot_0_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.addGeom('MyFoot3', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.jointType = 'R'

        capsulize('LeftFoot_foot_0_1_0')
        node = mcfg.getNode('LeftFoot_foot_0_1_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.addGeom('MyFoot4', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.jointType = 'R'

        capsulize('LeftFoot_foot_1_0')
        node = mcfg.getNode('LeftFoot_foot_1_0')
        node.addGeom('MyFoot3', [0.02*np.array([0., 0., .7]), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, 0.02*2.0+0.02))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = 'R'

        capsulize('LeftFoot_foot_1_1')
        node = mcfg.getNode('LeftFoot_foot_1_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.jointType = 'R'

        capsulize('LeftFoot_foot_1_2')
        node = mcfg.getNode('LeftFoot_foot_1_2')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(400., .01, -1))
        node.jointType = 'R'


        return mcfg

    # fname = "SimpleJump_long.bvh"
    fname = 'wd2_WalkForwardNormal00.bvh'
    dmm = DsModelMaker()
    dmm.setMainBvh(fname)
    dmm.toDsSkelFile('test.cpp', buildMcfg())
    # dmm.toDartSkelFile('test.cpp', buildMcfg())
