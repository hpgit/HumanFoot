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


class DartModelMaker:
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
                        jointType = "ball"
                    elif joint_type == "U":
                        jointType = "universal"
                    elif joint_type == "R":
                        jointType = "revolute"

            jointPair = (parent_name, joint_name)
            jointPairs.append(jointPair)
            bodyToJointTs.append(bodyToJointT)
            jointTypes.append(jointType)
        else:
            jointPair = ("world", joint_name)
            jointPairs.append(jointPair)
            bodyToJointTs.append(Inv(boneTs[joint_index]))
            jointTypes.append("free")


        for i in range(len_joint_children):
            childJointPairs, childbodyToJointTs, childJointTypes = self._createJoint(joint.children[i], posture, boneTs)
            jointPairs.extend(childJointPairs)
            bodyToJointTs.extend(childbodyToJointTs)
            jointTypes.extend(childJointTypes)

        return jointPairs, bodyToJointTs, jointTypes


    def AddDartShapeNode(self, T, size, geom, shapeType='visual'):
        geomtext = ""
        if geom == "box":
            geomtext = "box"
        elif geom == "cylinder":
            geomtext = "cylinder"
        elif geom == "ellipsoid":
            geomtext = "ellipsoid"
        else:
            raise "geom type Error"

        typetext = "visualization_shape"
        if shapeType == 'collision':
            typetext = "collision_shape"

        etShape = et.Element(typetext)
        # et.SubElement(etShape, "transformation").text = SE32veulerXYZstr(T)
        et.SubElement(etShape, "transformation").text = SE32vlogSO3str(T)
        etGeom = et.SubElement(et.SubElement(etShape, "geometry"), geomtext)
        if (geom == "box") or (geom == "ellipsoid"):
            et.SubElement(etGeom, "size").text = " ".join(map(str, size))
        else:
            et.SubElement(etGeom, "radius").text = str(size[0])
            et.SubElement(etGeom, "height").text = str(size[1])

        return etShape


    def AddInertia(self, geomMaterialOrMass):
        # TODO:
        etInertia = et.Element("inertia")
        if isinstance(geomMaterialOrMass, ypc.Material):
            # et.SubElement(etInertia, "mass").text = ".5"
            et.SubElement(etInertia, "mass").text = str(geomMaterialOrMass.getMass())
        elif isinstance(geomMaterialOrMass, float):
            et.SubElement(etInertia, "mass").text = str(geomMaterialOrMass)

        # et.SubElement(etInertia, "mass").text = ".5"
        et.SubElement(etInertia, "offset").text = "0.0 0 0.0"

        return etInertia


    def AddBody(self, name, T, offset, inertia):
        """

        :param name: str
        :param T: SE3
        :param offset: Vec3
        :param inertia:
        :return:
        """
        etBody = et.Element("body")
        etBody.attrib["name"] = name
        et.SubElement(etBody, "transformation").text = SE32veulerXYZstr(T)

        # etBody.append(self.AddInertia(.5))

        cylLen_2 = np.linalg.norm(offset)/2.
        # if False and("foot" in name or "Foot" in name):
        #     rad = .05
        #
        #     etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., cylLen_2)), [rad]*3, "ellipsoid"))
        #     if name == 'root':
        #         etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., -cylLen_2)), [rad]*3, "ellipsoid"))
        #
        #     etBody.append(self.AddDartShapeNode(SE3(), [rad/2., 2.*cylLen_2], "cylinder"))
        #
        #     etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., cylLen_2)), [rad]*3, "ellipsoid", "collision"))
        #     if name=='root':
        #         etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., -cylLen_2)), [rad]*3, "ellipsoid", "collision"))
        if self.config.hasNode(name):
        # if self.config is not None and self.config.hasNode(name):
            # pNode = self.Node(joint_name)
            # self._nodes[joint_index] = pNode
            # pNode = self._nodes[joint_index]
            # pNode.name = joint_name
            cfgNode = self.config.getNode(name)

            numGeom = len(cfgNode.geoms)
            if numGeom > 0:
                for i in range(numGeom):
                    geomType = cfgNode.geoms[i]

                    if geomType == "MyFoot3" or geomType == "MyFoot4" or geomType == "MyFoot5":
                        # capsule case
                        density = cfgNode.geomMaterial[i].density
                        radius = cfgNode.geomMaterial[i].radius
                        height = cfgNode.geomMaterial[i].height

                        if height <= 0.:
                            height = offset.Norm() + 2.*radius
                            cfgNode.geomMaterial[i].height = height

                        etBody.append(self.AddInertia(cfgNode.geomMaterial[i]))

                        geomT = SE3()
                        if cfgNode.geomTs[i] is not None:
                            geomT = SE3(cfgNode.geomTs[i][1])
                            geomT.SetPosition(geomT*Vec3(cfgNode.geomTs[i][0]))
                        else:
                            print("there is no geom Ts!")

                        etBody.append(self.AddDartShapeNode(geomT, [radius, height-2.*radius], "cylinder"))

                        geomSphereT = copy.deepcopy(geomT)
                        geomSphereT.SetPosition(geomT*Vec3(0., 0., -(height/2. - radius)))
                        if geomType == "MyFoot3" or geomType =="MyFoot4":
                            etBody.append(self.AddDartShapeNode(geomSphereT, [radius*2.]*3, "ellipsoid"))
                        if geomType == "MyFoot3":
                            etBody.append(self.AddDartShapeNode(geomSphereT, [radius*2.]*3, "ellipsoid", "collision"))

                        geomSphereT.SetPosition(geomT*Vec3(0., 0., (height/2. - radius)))
                        etBody.append(self.AddDartShapeNode(geomSphereT, [radius*2.]*3, "ellipsoid"))
                        if geomType != "MyFoot5":
                            etBody.append(self.AddDartShapeNode(geomSphereT, [radius*2.]*3, "ellipsoid", "collision"))

                        # etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., cylLen_2)), [radius]*3, "ellipsoid"))
                        # etBody.append(self.AddDartShapeNode(SE3(), [radius/2., 2.*cylLen_2], "cylinder"))
                        # etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., cylLen_2)), [radius]*3, "ellipsoid", "collision"))
                    else:
                        width = cfgNode.geomMaterial[i].width
                        length = cfgNode.geomMaterial[i].length
                        height = cfgNode.geomMaterial[i].height
                        etBody.append(self.AddInertia(cfgNode.geomMaterial[i]))

                        geomT = SE3()
                        if cfgNode.geomTs[i] is not None:
                            geomT = SE3(cfgNode.geomTs[i][1])
                            geomT.SetPosition(geomT*Vec3(cfgNode.geomTs[i][0]))

                        etBody.append(self.AddDartShapeNode(geomT, [width, height, length], "box"))
                        etBody.append(self.AddDartShapeNode(geomT, [width, height, length], "box", "collision"))

            else:
                geomType = cfgNode.geom
                if (geomType == "MyFoot3") or (geomType == "MyFoot4") or geomType == "MyFoot5":
                    radius = .05
                    if cfgNode.width is not None:
                        radius = cfgNode.width
                    length = offset.Norm() + 2.*radius
                    density = cfgNode.density
                    mass = 1.
                    if cfgNode.mass is not None:
                        mass = cfgNode.mass
                        density = mass/(radius*radius*math.pi*length)
                    else:
                        mass = density * radius * radius * math.pi * length

                    etBody.append(self.AddInertia(mass))

                    etBody.append(self.AddDartShapeNode(SE3(), [radius, length-2.*radius], "cylinder"))
                    if geomType == "MyFoot3" or geomType =="MyFoot4":
                        etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., -(length/2.-radius))), [radius*2.]*3, "ellipsoid"))
                    if geomType == "MyFoot3":
                        etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., -(length/2.-radius))), [radius*2.]*3, "ellipsoid", "collision"))
                    etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., length/2.-radius)), [radius*2.]*3, "ellipsoid"))
                    if geomType != "MyFoot5":
                        etBody.append(self.AddDartShapeNode(SE3(Vec3(0., 0., length/2.-radius)), [radius*2.]*3, "ellipsoid", "collision"))
                else:
                    length = 1.
                    if cfgNode.length is not None:
                        length = cfgNode.length * cfgNode.boneRatio
                    else:
                        length = offset.Norm() * cfgNode.boneRatio

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

                    etBody.append(self.AddInertia(cfgNode.mass))

                    etBody.append(self.AddDartShapeNode(SE3(), [width, height, length], "box"))
                    etBody.append(self.AddDartShapeNode(SE3(), [width, height, length], "box", "collision"))
        else:
            etBody.append(self.AddDartShapeNode(SE3(), [.1, .1, 1.8*cylLen_2], "box"))
            etBody.append(self.AddDartShapeNode(SE3(), [.1, .1, 1.8*cylLen_2], "box", "collision"))

        return etBody

    def setMainBvh(self, filename):
        self.bvh = yf.readBvhFileAsBvh(filename)
        self.skelname = filename.split("/")[-1].split(".")[0]

    def attachPartBvh(self, partFilePath, attachPart, scale=1.0, mirror=None):
        partBvh = yf.readBvhFileAsBvh(partFilePath)
        if mirror is not None:
            partBvh.mirror(mirror)
        self.bvh.replaceJointFromBvh(attachPart, partBvh, scale)

    def posture2dartSkel(self, posture, config=None):
        self.config = config

        names, Ts, offsets, boneTs = self.createBodies(posture)
        jointPairs, bodyToJointTs, jointTypes = self.createJoints(posture, boneTs)

        etSkel = et.Element("skel", {"version": "1.0"})
        etSkelTree = et.ElementTree(etSkel)

        etWorld = et.SubElement(etSkel, "world", {"name": "world 1"})

        # set physics constant
        etPhysics = et.SubElement(etWorld, "physics")
        et.SubElement(etPhysics, "time_step").text = "0.001"
        et.SubElement(etPhysics, "gravity").text = "0 -9.81 0"
        et.SubElement(etPhysics, "collision_detector").text = "fcl_mesh"


        if True:
            # add ground body and joint
            etGroundSkeleton = et.SubElement(etWorld, "skeleton", {"name": "grount skeleton"})
            et.SubElement(etGroundSkeleton, "mobile").text = "false"
            etGroundBody = et.SubElement(etGroundSkeleton, "body", {"name": "ground"})
            et.SubElement(etGroundBody, "transformation").text = "0 -0.025 0 0 0 0"
            # etGroundBody.append(AddDartShapeNode(SE3(), [5.0, 0.05, 5.0], "box"))
            etGroundBody.append(self.AddDartShapeNode(SE3(), [10000.0, 0.05, 10000.0], "box", "collision"))

            etGroundJoint = et.SubElement(etGroundSkeleton, "joint", {"type": "free", "name": "joint 1"})
            et.SubElement(etGroundJoint, "parent").text = "world"
            et.SubElement(etGroundJoint, "child").text = "ground"

        etSkeleton = et.SubElement(etWorld, "skeleton")
        etSkeleton.attrib["name"] = self.skelname

        # add Body
        for i in range(len(names)):
        # for i in range(1):
            # append body
            etSkeleton.append(self.AddBody(names[i], Ts[i], offsets[i], None))

        # TODO:
        # add Joint
        # etJoint = et.SubElement(etSkeleton, "joint", {"type": "free", "name": names[0]})
        # et.SubElement(etJoint, "parent").text = "world"
        # et.SubElement(etJoint, "child").text = names[0]
        # et.SubElement(etJoint, "init_pos").text = "0 0 0 0 0 0"
        # et.SubElement(etJoint, "init_vel").text = "0 0 0 0 0 0"

        for i in range(len(jointPairs)):
            jointPair = jointPairs[i]
            etJoint = et.SubElement(etSkeleton, "joint", {"type": jointTypes[i], "name": "j_"+jointPair[1]})
            if jointTypes[i] == "universal":
                etAxis1 = et.SubElement(etJoint, "axis")
                et.SubElement(etAxis1, "xyz").text = "1 0 0"
                etAxis2 = et.SubElement(etJoint, "axis2")
                et.SubElement(etAxis2, "xyz").text = "0 0 1"
            elif jointTypes[i] == "revolute":
                etAxis = et.SubElement(etJoint, "axis")
                et.SubElement(etAxis, "xyz").text = "1 0 0"

            et.SubElement(etJoint, "transformation").text = SE32vlogSO3str(bodyToJointTs[i])

            # if jointPair[0] == "world":
            #     etJoint = et.SubElement(etSkeleton, "joint", {"type": "free", "name": "j_"+jointPair[1]})
            #     # etJoint = et.SubElement(etSkeleton, "joint", {"type": "free", "name": jointPair[1]})
            #     et.SubElement(etJoint, "transformation").text = SE32vlogSO3str(bodyToJointTs[i])
            # else:
            #     etJoint = et.SubElement(etSkeleton, "joint", {"type": "ball", "name": "j_"+jointPair[1]})
            #     # etJoint = et.SubElement(etSkeleton, "joint", {"type": "ball", "name": jointPair[1]})
            #     et.SubElement(etJoint, "transformation").text = SE32vlogSO3str(bodyToJointTs[i])
            #     # et.SubElement(etJoint, "axis_order").text = "xyz"
            et.SubElement(etJoint, "parent").text = jointPair[0]
            et.SubElement(etJoint, "child").text = jointPair[1]

        return etSkelTree, boneTs

    def toDartSkelFile(self, skelfile, config):
        tree, boneTs = self.posture2dartSkel(self.bvh.toJointMotion(1., False)[0], config)
        output = open(skelfile, "w")
        output.write(prettifyXML(tree.getroot()))
        output.close()

    def posture2dartSkelFile(self, name, posture, skelfile, config):
        self.skelname = name
        tree, boneTs = self.posture2dartSkel(posture, config)
        output = open(skelfile, "w")
        output.write(prettifyXML(tree.getroot()))
        output.close()

    def posture2dartSkelXmlStr(self, name, posture, config):
        self.skelname = name
        tree, boneTs = self.posture2dartSkel(posture, config)
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
        # massMap['RightFoot'] += 2.
        massMap['RightFoot'] += .4

        # left foot : 4
        # massMap['LeftFoot'] += 2.
        massMap['LeftFoot'] += .4
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
    tree = DartModelMaker().bvh2dartSkelFile(fname, "test.xml", buildMcfg())
