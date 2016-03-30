import sys
sys.path.append("..")

sys.path.append("../PyCommon/modules")

from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.VirtualPhysics.LieGroup import *
from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Resource import ysMotionLoader as yf

import xml.etree.ElementTree as et


def createBodies(posture):
    joint = posture.skeleton.root
    rootPos = SE3(posture.rootPos)
    tpose = posture.getTPose()
    return _createBody(joint, rootPos, tpose)


def _createBody(joint, parentT, posture):
    ls = []
    len_joint_children = len(joint.children)
    if len_joint_children == 0:
        return []

    P = SE3(joint.offset)
    T = parentT * P

    joint_name = joint.name
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
        if joint_name == "Hips":
            boneT = SE3()

        defaultBoneV = Vec3(0., 0., 1.)
        boneR = SE3(mm.getSO3FromVectors(defaultBoneV, offset))

        if joint_name != "Hips":
            boneT = boneT * boneR

        newT = T * boneT

        ls.append(newT)

    for i in range(len_joint_children):
        ls.extend(_createBody(joint.children[i], T, posture))

    return ls



def bvh2dartSkel(filename):
    motion = yf.readBvhFile(filename, .05)
    # bvh = yf.readBvhFileAsBvh(filename)
    # skel = bvh.toJointSkeleton(1.0, False)
    # sk = ym.JointSkeleton()
    note = et.Element("note")
    body = et.Element("body")
    body.text = "1.0 0.0 0.0 0.0 0.0 0.0"
    note.append(body)

    et.dump(note)

    return motion


if __name__ == '__main__':
    # bvh2dartSkel("hehe")
    # bvh = yf.readBvhFileAsBvh(filename)
    motion = bvh2dartSkel("../FootProject/SimpleJump.bvh")
    print createBodies(motion[0])
