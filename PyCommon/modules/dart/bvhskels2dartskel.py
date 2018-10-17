import math
import numpy as np

from PyCommon.modules.Motion import ysMotion as ym

from PyCommon.modules.Math import mmMath as mm
# from PyCommon.modules.VirtualPhysics.LieGroup import *
from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Resource import ysMotionLoader as yf

from PyCommon.modules.Simulator import ysPhysConfig as ypc

import xml.etree.ElementTree as et
from xml.dom import minidom



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


class BvhSkelToDartSkel(object):
    def __init__(self, skeleton, mass_map, shape_map=None, name='skel', has_ground=True):
        """

        :type skeleton: ym.JointSkeleton
        """
        self.skeleton = skeleton
        self.name = name
        self.has_ground = has_ground
        self.mass_map = mass_map

        self.tree = None

        self.body_name = []
        self.body_pos = []
        self.body_mass = []
        self.body_geom_size = []

        self.processs_body_geom()

        self.generate()

    def processs_body_geom(self):
        skel = self.skeleton
        for i in range(skel.getJointNum()):
            joint = skel.getJoint(i)  # type: ym.Joint
            self.body_name.append(joint.name)
            # print(joint.children)
            print(joint.name)
            avg_offset = sum([child.offset for child in joint.children]) / len(joint.children)
            length = mm.length(avg_offset)
            length_vector = mm.seq2Vec3(avg_offset)/length
            print(length, length_vector)
            mass = self.mass_map[joint.name]
            width = math.sqrt(mass/1000./length)
            height = width
            self.body_mass.append(mass)
            self.body_geom_size.append([length, width, height])

    def add_bodies(self, et_skeleton):
        """

        :type et_skeleton: et.Element
        :return:
        """
        for i in range(len(self.body_name)):
            et_body = et.SubElement(et_skeleton, 'body', {'name': self.body_name[i]})

            et_inertia = et.SubElement(et_body, 'inertia')
            et.SubElement(et_inertia, 'mass').text = str(self.body_mass[i])
            et.SubElement(et_inertia, 'offset').text = '0.0 0.0 0.0'

            self.add_dart_shape_node(et_body, mm.I_SE3(), self.body_geom_size[i], 'box', 'visualization')
            self.add_dart_shape_node(et_body, mm.I_SE3(), self.body_geom_size[i], 'box', 'collision')

    def add_joints(self, et_skeleton):
        pass

    def add_dart_shape_node(self, et_body, transf, full_size, shape, shape_type):
        et_shape = et.SubElement(et_body, shape_type+'_shape')
        et.SubElement(et_shape, 'transformation').text = SE32veulerXYZstr(transf)
        et_geom = et.SubElement(et_shape, shape)
        et.SubElement(et_geom, 'size').text = str(full_size[0]) + ' ' + str(full_size[1]) + ' ' + str(full_size[2])

    def generate(self):
        skel = self.skeleton
        posture = ym.JointPosture(skel)
        posture.initLocalRs()
        posture.updateGlobalT()

        joint_name = [skel.getJointName(i) for i in range(skel.getJointNum())]
        joint_pos = [posture.getJointPositionGlobal(i) for i in range(skel.getJointNum())]
        # print(list(zip(joint_name, joint_pos)))

        et_skel = et.Element("skel", {"version": "1.0"})
        et_skel_tree = et.ElementTree(et_skel)

        et_world = et.SubElement(et_skel, "world", {"name": "world 1"})

        # set physics constant
        et_physics = et.SubElement(et_world, "physics")
        et.SubElement(et_physics, "time_step").text = "0.001"
        et.SubElement(et_physics, "gravity").text = "0 -9.81 0"
        et.SubElement(et_physics, "collision_detector").text = "fcl_mesh"

        if self.has_ground:
            # add ground body and joint
            etGroundSkeleton = et.SubElement(et_world, "skeleton", {"name": "grount skeleton"})
            et.SubElement(etGroundSkeleton, "mobile").text = "false"
            etGroundBody = et.SubElement(etGroundSkeleton, "body", {"name": "ground"})
            et.SubElement(etGroundBody, "transformation").text = "0 -0.025 0 0 0 0"
            self.add_dart_shape_node(etGroundBody, mm.I_SE3(), [1000., 0.05, 1000.], "box", 'visualization')
            self.add_dart_shape_node(etGroundBody, mm.I_SE3(), [1000., 0.05, 1000.], "box", "collision")

            etGroundJoint = et.SubElement(etGroundSkeleton, "joint", {"type": "free", "name": "joint 1"})
            et.SubElement(etGroundJoint, "parent").text = "world"
            et.SubElement(etGroundJoint, "child").text = "ground"

        et_skeleton = et.SubElement(et_world, "skeleton")
        et_skeleton.attrib["name"] = self.name

        # add Body
        self.add_bodies(et_skeleton)

        '''
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
        '''

        # return etSkelTree, boneTs
        self.tree = et_skel_tree

    def save(self, filename):
        with open(filename, 'w') as f:
            f.write(prettifyXML(self.tree.getroot()))
            pass


if __name__ == '__main__':
    import PyCommon.modules.Motion.ysHierarchyEdit as yme
    motion = yf.readBvhFile('test.bvh', 0.01)
    yme.removeJoint(motion, 'LHipJoint', False)
    yme.removeJoint(motion, 'RHipJoint', False)
    yme.removeJoint(motion, 'LowerBack', False)
    yme.removeJoint(motion, 'LeftToeBase', False)
    yme.removeJoint(motion, 'RightToeBase', False)
    yme.removeJoint(motion, 'Neck', False)
    yme.removeJoint(motion, 'Head', False)
    yme.removeJoint(motion, 'LeftHandIndex1', False)
    yme.removeJoint(motion, 'RightHandIndex1', False)
    yme.removeJoint(motion, 'LeftFingerBase', False)
    yme.removeJoint(motion, 'RightFingerBase', False)
    yme.removeJoint(motion, 'LThumb_Effector', False)
    yme.removeJoint(motion, 'RThumb_Effector', False)
    yme.removeJoint(motion, 'LThumb', False)
    yme.removeJoint(motion, 'RThumb', False)
    motion.updateGlobalT()

    massMap = {}
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips',
                                'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector',
                                'LeftLeg', 'LeftShoulder', 'LeftToeBase', 'LeftToeBase_Effector', 'LeftUpLeg',
                                'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector',
                                'RightLeg', 'RightShoulder', 'RightToeBase', 'RightToeBase_Effector', 'RightUpLeg',
                                'Spine', 'Spine1', 'Neck1'], 0.)

    # torso : 10
    massMap['Hips'] += 2.
    massMap['Spine'] += 8.

    # head : 3
    massMap['Spine1'] += 3.

    # right upper arm : 2
    massMap['RightArm'] += 2.

    # left upper arm : 2
    massMap['LeftArm'] += 2.

    # right shoulder: 2
    massMap['RightShoulder'] += 1.

    # left shoulder: 2
    massMap['LeftShoulder'] += 1.

    # right lower arm : 1
    #massMap['RightForeArm'] = 1.
    massMap['RightForeArm'] = 2.

    # left lower arm : 1
    #massMap['LeftForeArm'] = 1.
    massMap['LeftForeArm'] = 2.

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

    # right foot : 2
    massMap['RightFoot'] += 2.

    # left foot : 2
    massMap['LeftFoot'] += 2.

    # right Toe : 1
    massMap['RightToeBase'] += 1.

    # left Toe : 1
    massMap['LeftToeBase'] += 1.

    # right hand : .5
    massMap['RightHand'] += .5

    # left hand : .5
    massMap['LeftHand'] += .5

    # head
    massMap['Neck1'] += 5.

    BvhSkelToDartSkel(motion[0].skeleton, massMap).save('test.xml')

