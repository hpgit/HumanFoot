import sys
sys.path.append("..")

sys.path.append("../PyCommon/modules")

from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.VirtualPhysics.LieGroup import *
from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Resource import ysMotionLoader as yf

import xml.etree.ElementTree as et
from xml.dom import minidom


def prettifyXML(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = et.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="    ")


def list2str(ls):
    strstr = ""
    for i in range(len(ls)):
        strstr += str(ls[i]) + " "
    return strstr


def SE32veulerXYZ(T):
    v = np.append(T[0:3, 3].flatten(), mm.R2XYZ(T[0:3, 0:3]))
    return v


def SE32veulerXYZstr(T):
    text = ""
    v = SE32veulerXYZ(T)
    for j in range(6):
        text += str(v[j]) + " "
    return text


def createBodies(posture):
    joint = posture.skeleton.root
    rootPos = SE3(posture.rootPos)
    tpose = posture.getTPose()
    return _createBody(joint, rootPos, tpose)


def _createBody(joint, parentT, posture):
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

    if True:
        offset = Vec3(0.)
        for i in range(len_joint_children):
            offset += Vec3(joint.children[i].offset)

        if True:
            offset *= 1./len_joint_children

        boneT = SE3(offset * .5)
        # if joint_name == "Hips":
        if joint.parent is None:
            boneT = SE3()

        defaultBoneV = Vec3(0., 0., 1.)
        boneR = SE3(mm.getSO3FromVectors(defaultBoneV, offset))

        # if joint_name != "Hips":
        if joint.parent is not None:
            boneT = boneT * boneR

        boneTs.append(boneT)
        offsets.append(offset)

        newT = T * boneT

        Ts.append(newT)

    for i in range(len_joint_children):
        childNames, childTs, childOffsets, childBoneTs = _createBody(joint.children[i], T, posture)
        names.extend(childNames)
        Ts.extend(childTs)
        offsets.extend(childOffsets)
        boneTs.extend(childBoneTs)

    return names, Ts, offsets, boneTs


def createJoints(posture, boneTs):
    joint = posture.skeleton.root
    return _createJoint(joint, posture, boneTs)

    # TODO:
    # return body_to_joint list, [parent, child] list


def _createJoint(joint, posture, boneTs):
    jointPairs = []
    bodyToJointTs = []

    len_joint_children = len(joint.children)
    if len_joint_children == 0:
        return jointPairs, bodyToJointTs

    def Inv(T):
        return SE3(np.linalg.inv(T))

    offset = joint.offset
    P = SE3(joint.offset)
    joint_name = joint.name
    joint_index = posture.skeleton.getJointIndex(joint_name)
    R = SE3(posture.getJointOrientationLocal(joint_index))

    invLocalT = Inv(R) * Inv(P)

    temp_joint = joint
    nodeExistParentJoint = None

    if temp_joint.parent is None:
        nodeExistParentJoint = None
    else:
        nodeExistParentJoint = temp_joint.parent

    if nodeExistParentJoint is not None:
        parent_name = nodeExistParentJoint.name
        parent_index = posture.skeleton.getJointIndex(parent_name)

        parentbodyToJointT = Inv(boneTs[parent_index]) * Inv(invLocalT)
        bodyToJointT = Inv(boneTs[joint_index])

        jointPair = (parent_name, joint_name)
        jointPairs.append(jointPair)
        bodyToJointTs.append(bodyToJointT)
    else:
        jointPair = ("world", joint_name)
        jointPairs.append(jointPair)
        bodyToJointTs.append(SE3())

    for i in range(len_joint_children):
        childJointPairs, childbodyToJointTs = _createJoint(joint.children[i], posture, boneTs)
        jointPairs.extend(childJointPairs)
        bodyToJointTs.extend(childbodyToJointTs)

    return jointPairs, bodyToJointTs


def AddDartShapeNode(T, size, geom, shapeType='visual'):
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
    et.SubElement(etShape, "transformation").text = SE32veulerXYZstr(T)
    etGeom = et.SubElement(et.SubElement(etShape, "geometry"), geomtext)
    if (geom == "box") or (geom == "ellipsoid"):
        et.SubElement(etGeom, "size").text = list2str(size)
    else:
        et.SubElement(etGeom, "radius").text = str(size[0])
        et.SubElement(etGeom, "height").text = str(size[1])

    return etShape


def AddInertia():
    # TODO:
    etInertia = et.Element("inertia")
    et.SubElement(etInertia, "mass").text = ".5"
    et.SubElement(etInertia, "offset").text = "0.0 0 0.0"

    return etInertia


def AddBody(name, T, offset, inertia):
    etBody = et.Element("body")
    etBody.attrib["name"] = name
    et.SubElement(etBody, "transformation").text = SE32veulerXYZstr(T)

    etBody.append(AddInertia())

    cylLen_2 = np.linalg.norm(offset)/2.

    etBody.append(AddDartShapeNode(SE3(Vec3(0., 0., cylLen_2)), [.1, .1, .1], "ellipsoid"))
    # etBody.append(AddDartShapeNode(SE3(Vec3(0., 0., -cylLen_2)), [.1, .1, .1], "ellipsoid"))
    # etBody.append(AddDartShapeNode(SE3(), [.05, 2.*cylLen_2], "cylinder"))
    etBody.append(AddDartShapeNode(SE3(Vec3(0., 0., cylLen_2)), [.1, .1, .1], "ellipsoid", "collision"))
    # etBody.append(AddDartShapeNode(SE3(Vec3(0., 0., -cylLen_2)), [.1, .1, .1], "ellipsoid", "collision"))

    return etBody


def bvh2dartSkel(filename):
    motion = yf.readBvhFile(filename, .05)
    # bvh = yf.readBvhFileAsBvh(filename)
    # skel = bvh.toJointSkeleton(1.0, False)
    # sk = ym.JointSkeleton()

    names, Ts, offsets, boneTs = createBodies(motion[0])
    jointPairs, bodyToJointTs = createJoints(motion[0], boneTs)

    etSkel = et.Element("skel", {"version": "1.0"})
    etSkelTree = et.ElementTree(etSkel)

    etWorld = et.SubElement(etSkel, "world", {"name": "world 1"})

    etPhysics = et.SubElement(etWorld, "physics")
    et.SubElement(etPhysics, "time_step").text = "0.001"
    et.SubElement(etPhysics, "gravity").text = "0 -9.81 0"
    et.SubElement(etPhysics, "collision_detector").text = "fcl_mesh"

    etGroundSkeleton = et.SubElement(etWorld, "skeleton", {"name": "grount skeleton"})
    et.SubElement(etGroundSkeleton, "mobile").text = "false"
    etGroundBody = et.SubElement(etGroundSkeleton, "body", {"name": "ground"})
    et.SubElement(etGroundBody, "transformation").text = "0 -0.92 0 0 0 0"
    etGroundBody.append(AddDartShapeNode(SE3(), [5.0, 0.05, 5.0], "box"))
    etGroundBody.append(AddDartShapeNode(SE3(), [10.0, 0.05, 10.0], "box", "collision"))

    etGroundJoint = et.SubElement(etGroundSkeleton, "joint", {"type": "free", "name": "joint 1"})
    et.SubElement(etGroundJoint, "parent").text = "world"
    et.SubElement(etGroundJoint, "child").text = "ground"

    etSkeleton = et.SubElement(etWorld, "skeleton")
    etSkeleton.attrib["name"] = filename.split("/")[-1].split(".")[0]

    # add Body
    for i in range(len(names)):
    # for i in range(1):
        # append body
        etSkeleton.append(AddBody(names[i], Ts[i], offsets[i], None))

    # TODO:
    # add Joint
    etJoint = et.SubElement(etSkeleton, "joint", {"type": "free", "name": names[0]})
    et.SubElement(etJoint, "parent").text = "world"
    et.SubElement(etJoint, "child").text = names[0]
    # et.SubElement(etJoint, "init_pos").text = "0 0 0 0 0 0"
    # et.SubElement(etJoint, "init_vel").text = "0 0 0 0 0 0"

    for i in range(len(jointPairs)):
        jointPair = jointPairs[i]

        if jointPair[0] == "world":
            etJoint = et.SubElement(etSkeleton, "joint", {"type": "free", "name": "j_"+jointPair[1]})
        else:
            etJoint = et.SubElement(etSkeleton, "joint", {"type": "ball", "name": "j_"+jointPair[1]})
            et.SubElement(etJoint, "transformation").text = SE32veulerXYZstr(bodyToJointTs[i])
            # et.SubElement(etJoint, "axis_order").text = "xyz"
        et.SubElement(etJoint, "parent").text = jointPair[0]
        et.SubElement(etJoint, "child").text = jointPair[1]

    return etSkelTree


if __name__ == '__main__':
    fname = "../FootProject/SimpleJump.bvh"
    tree = bvh2dartSkel(fname)
    output = open("test.skel", "w")
    output.write(prettifyXML(tree.getroot()))
    output.close()

    # tree.write("test.skel",xml_declaration=)





