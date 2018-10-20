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
        :type mass_map: dict
        :type shape_map: dict
        """
        self.skeleton = skeleton
        self.name = name
        self.has_ground = has_ground
        self.mass_map = mass_map
        self.shape_map = shape_map
        if shape_map is None:
            self.shape_map = dict()
            self.shape_map.fromkeys(self.mass_map.keys(), None)

        self.tree = None

        self.body_name = []
        self.body_transf = []
        self.joint_to_body_transf = []
        self.body_mass = []
        self.body_geom_size = []
        self.body_geom_type = []

        skel = self.skeleton
        posture = ym.JointPosture(skel)
        posture.initLocalRs()
        posture.updateGlobalT()

        self.joint_name = [skel.getJointName(i) for i in range(skel.getJointNum())]
        self.joint_transf = [mm.TransVToSE3(posture.getJointPositionGlobal(i)) for i in range(skel.getJointNum())]

        self.processs_body_geom()

        self.generate()

    def processs_body_geom(self):
        skel = self.skeleton
        for i in range(skel.getJointNum()):
            joint = skel.getJoint(i)  # type: ym.Joint
            self.body_name.append(joint.name)
            # print(joint.children)
            avg_offset = mm.seq2Vec3(sum([child.offset for child in joint.children]) / len(joint.children))
            length = mm.length(avg_offset)
            length *= 0.9
            mass = self.mass_map[joint.name]
            width = math.sqrt(mass/1000./length*0.9)
            height = width
            geom_type = 'box' if self.shape_map[joint.name] is None else self.shape_map[joint.name][0]
            geom_size = [width, height, length] if self.shape_map[joint.name] is None else self.shape_map[joint.name][1]

            offset_T = mm.getSE3ByTransV(.5 * avg_offset)

            defaultBoneV = mm.unitZ()
            boneR = mm.SO3ToSE3(mm.getSO3FromVectors(defaultBoneV, avg_offset))

            boneT = np.dot(offset_T, boneR)
            # boneT = offset_T
            self.joint_to_body_transf.append(boneT)

            self.body_transf.append(np.dot(self.joint_transf[i], boneT))
            self.body_mass.append(mass)
            self.body_geom_type.append(geom_type)
            self.body_geom_size.append(geom_size)

    def add_bodies(self, et_skeleton):
        """

        :type et_skeleton: et.Element
        :return:
        """
        for i in range(len(self.body_name)):
            et_body = et.SubElement(et_skeleton, 'body', {'name': self.body_name[i]})
            et.SubElement(et_body, 'transformation').text = SE32veulerXYZstr(self.body_transf[i])

            et_inertia = et.SubElement(et_body, 'inertia')
            et.SubElement(et_inertia, 'mass').text = str(self.body_mass[i])
            et.SubElement(et_inertia, 'offset').text = '0.0 0.0 0.0'

            self.add_dart_shape_node(et_body, self.body_name[i], mm.I_SE3(), self.body_geom_size[i], self.body_geom_type[i], 'visualization')
            self.add_dart_shape_node(et_body, self.body_name[i], mm.I_SE3(), self.body_geom_size[i], self.body_geom_type[i], 'collision')

    def add_joints(self, et_skeleton):
        skel = self.skeleton
        for i in range(len(self.joint_name)):
            joint = skel.getJoint(i)  # type: ym.Joint
            etJoint = et.SubElement(et_skeleton, "joint", {"type": "free" if i == 0 else "ball", "name": self.joint_name[i]})
            # parent_joint_idx = self.joint_name.index(joint.parent.name)
            # parent_body_transf = self.body_transf[parent_joint_idx]
            # et.SubElement(etJoint, 'transformation').text = SE32veulerXYZstr(np.dot(mm.invertSE3(parent_body_transf), self.joint_transf[i]))
            # et.SubElement(etJoint, 'transformation').text = SE32veulerXYZstr(np.dot(mm.invertSE3(self.joint_transf[i]), self.body_transf[i]))
            et.SubElement(etJoint, 'transformation').text = SE32veulerXYZstr(mm.invertSE3(self.joint_to_body_transf[i]))
            et.SubElement(etJoint, "parent").text = 'world' if i == 0 else joint.parent.name
            et.SubElement(etJoint, "child").text = self.joint_name[i]

    def add_dart_shape_node(self, et_body, name, transf, full_size, shape, shape_type):
        et_shape = et.SubElement(et_body, shape_type+'_shape', {'name': name+'_'+shape_type+'_shape'})
        et.SubElement(et_shape, 'transformation').text = SE32veulerXYZstr(transf)
        et_geom_shape = et.SubElement(et.SubElement(et_shape, 'geometry'), shape)
        if shape == 'box':
            et.SubElement(et_geom_shape, 'size').text = str(full_size[0]) + ' ' + str(full_size[1]) + ' ' + str(full_size[2])
        elif shape == 'sphere':
            et.SubElement(et_geom_shape, 'radius').text = str(full_size)

    def generate(self):
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
            self.add_dart_shape_node(etGroundBody, 'ground', mm.I_SE3(), [1000., 0.05, 1000.], "box", 'visualization')
            self.add_dart_shape_node(etGroundBody, 'ground', mm.I_SE3(), [1000., 0.05, 1000.], "box", "collision")

            etGroundJoint = et.SubElement(etGroundSkeleton, "joint", {"type": "free", "name": "joint 1"})
            et.SubElement(etGroundJoint, "parent").text = "world"
            et.SubElement(etGroundJoint, "child").text = "ground"

        et_skeleton = et.SubElement(et_world, "skeleton")
        et_skeleton.attrib["name"] = self.name

        # add Body
        self.add_bodies(et_skeleton)

        # add Joint
        self.add_joints(et_skeleton)

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
    yme.rotateJointLocal(motion, 'LeftFoot', mm.exp(mm.v3(.8,-0.0,-0.4), -.5), False)
    yme.rotateJointLocal(motion, 'RightFoot', mm.exp(mm.v3(.8,0.0,0.4), -.5), False)
    motion.updateGlobalT()
    motion.translateByOffset((0, -0.08, 0))

    massMap = {}
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips',
                                'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector',
                                'LeftLeg', 'LeftShoulder', 'LeftToeBase', 'LeftToeBase_Effector', 'LeftUpLeg',
                                'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector',
                                'RightLeg', 'RightShoulder', 'RightToeBase', 'RightToeBase_Effector', 'RightUpLeg',
                                'Spine', 'Spine1', 'Neck1'], 0.)

    # torso : 10
    massMap['Hips'] += 2.
    massMap['Spine'] += 5.
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
    massMap['RightForeArm'] = 1.
    # massMap['RightForeArm'] = 2.

    # left lower arm : 1
    massMap['LeftForeArm'] = 1.
    # massMap['LeftForeArm'] = 2.

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

    # head : 3
    massMap['Neck1'] += 3.

    shape_map = {}
    shape_map = shape_map.fromkeys(massMap.keys(), None)
    shape_map['Hips'] = ('sphere', 0.12)
    shape_map['Spine'] = ('sphere', 0.12)
    shape_map['Spine1'] = ('sphere', 0.05)
    shape_map['Neck1'] = ('sphere', 0.1)
    shape_map['LeftFoot'] = ('box', [0.1, 0.09, 0.225])
    shape_map['RightFoot'] = ('box', [0.1, 0.09, 0.225])
    shape_map['LeftHand'] = ('sphere', 0.05)
    shape_map['RightHand'] = ('sphere', 0.05)

    BvhSkelToDartSkel(motion[0].skeleton, massMap, shape_map).save('test.xml')

    from fltk import *
    from PyCommon.modules.GUI.hpSimpleViewer import hpSimpleViewer as SimpleViewer
    from PyCommon.modules.Renderer import ysRenderer as yr

    import pydart2 as pydart

    pydart.init()
    world = pydart.World(1./1200., "test.xml")
    # model = world.skeletons[1]

    posture = ym.JointPosture(motion[0].skeleton)
    posture.initLocalRs()
    posture.translateByOffset([0.02642508, 1.02154927, 1.09827205])
    posture.updateGlobalT()

    viewer = SimpleViewer(rect=(0, 0, 1200, 800), viewForceWnd=False)
    viewer.setMaxFrame(100)

    rd_target_position = [None]

    points = [None]
    viewer.doc.addRenderer('motion', yr.JointMotionRenderer(motion))
    viewer.doc.addRenderer('MotionModel', yr.DartRenderer(world, (150,150,255), yr.POLYGON_FILL, save_state=True))
    viewer.doc.addRenderer('joints', yr.PointsRenderer(points, (0,255,0), save_state=True))

    def simulateCallback(frame):
        # del points[:]
        # for j in range(world.skeletons[1].num_joints()):
        #     points.append(world.skeletons[1].joint(j).get_world_frame_after_transform()[:3, 3])
        # for i in range(40):
        #     world.step()
        world.skeletons[1].set_positions(motion[frame].get_q())
        motion.frame = frame

    # viewer.setPreFrameCallback_Always(preFrameCallback)
    viewer.setSimulateCallback(simulateCallback)

    viewer.startTimer(1. / 30.)

    viewer.show()
    Fl.run()

