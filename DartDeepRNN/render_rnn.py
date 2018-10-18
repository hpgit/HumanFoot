from DartDeepRNN.rnn.RNNController import RNNController
from DartDeepRNN.util.Pose2d import Pose2d
from DartDeepRNN.util.Util import *

from OpenGL.GL import *
from OpenGL.GLU import *
from fltk import *
from PyCommon.modules.GUI.hpSimpleViewer import hpSimpleViewer as SimpleViewer
from PyCommon.modules.Renderer import ysRenderer as yr
from PyCommon.modules.Math import mmMath as mm
from PyCommon.modules.Motion import ysMotion as ym
from PyCommon.modules.Resource import ysMotionLoader as yf
from PyCommon.modules.dart.dart_ik import DartIk

import pydart2 as pydart
import numpy as np


MOTION_SCALE = .01

joint_point_list = [None, "Head_End", "LeftHand", "LeftFoot", "LeftToeBase", "RightHand", "RightFoot", "RightToeBase", "LeftArm",
"RightArm", "LeftForeArm", "LeftLeg", "RightForeArm", "RightLeg", "Spine", "LeftHandIndex1", "RightHandIndex1", "Neck1", "LeftUpLeg", "RightUpLeg"]

joint_list = ["Head", "Hips", "LHipJoint", "LeftArm", "LeftFoot", "LeftForeArm", "LeftHand",
 "LeftLeg", "LeftShoulder", "LeftToeBase", "LeftUpLeg", "LowerBack", "Neck", "Neck1",
 "RHipJoint", "RightArm", "RightFoot","RightForeArm","RightHand", "RightLeg","RightShoulder","RightToeBase","RightUpLeg",
 "Spine","Spine1"]


class ModelViewer(object):
    def __init__(self, folder):
        pydart.init()
        self.world = pydart.World(1./1200., "data/cmu_with_ground.xml")
        self.model = self.world.skeletons[1]
        self.ik = DartIk(self.model)

        self.controller = RNNController(folder)

        self.all_angles = [[] for i in range(93)]

        viewer = SimpleViewer(viewForceWnd=False)
        self.viewer = viewer
        viewer.record(False)
        viewer.setMaxFrame(100)
        self.isFirst = True
        self.lines = None
        viewer.motionViewWnd.glWindow.set_mouse_pick(True)

        def callback_btn(ptr):
            self.controller.reset()
        viewer.objectInfoWnd.addBtn('reset', callback_btn)

        self.rc = yr.RenderContext()

        self.rd_target_position = [None]
        self.rd_frames = [None]

        viewer.doc.addRenderer('contact', yr.PointsRenderer(self.rd_target_position, (0, 255, 0), save_state=False))
        viewer.doc.addRenderer('MotionModel', yr.DartRenderer(self.world, (150,150,255), yr.POLYGON_FILL, save_state=False))
        viewer.doc.addRenderer('rd_frames', yr.FramesRenderer(self.rd_frames))

        def extraDrawCallback():
            self.rd_target_position[0] = self.viewer.motionViewWnd.glWindow.pickPoint
            self.step_model()
            del self.rd_frames[:]
            self.rd_frames.append(self.model.body(0).world_transform())
            # for i in range(3):
            #     print(self.model.body(0).world_transform()[:3, i])

        viewer.setExtraDrawCallback(extraDrawCallback)

        viewer.startTimer(1. / 30.)

        viewer.show()
        viewer.play()
        Fl.run()

    def get_target(self):
        p = self.viewer.motionViewWnd.glWindow.pickPoint

        target = Pose2d([p[0]/MOTION_SCALE, -p[2]/MOTION_SCALE])
        target = self.controller.pose.relativePose(target)
        target = target.p
        t_len = v_len(target)
        if t_len > 80:
            ratio = 80/t_len
            target[0] *= ratio
            target[1] *= ratio
        return target

    def step_model(self):
        contacts, points, angles, orientations, root_orientation = self.controller.step(self.get_target())

        # pairs = [[0,11,3,4],
        #          [0,8,10,2],
        #          [0,13,6,7],
        #          [0,9,12,5],
        #          [0,1]]
        pairs = [[0,18,11,3,4],
                 [0,14,8,10,2],
                 [0,19,13,6,7],
                 [0,14,9,12,5],
                 [0,14,17,1]]
        self.lines = []
        for pair in pairs:
            for i in range(len(pair)-1):
                self.lines.append([points[pair[i]], points[pair[i+1]]])
        # print(len(orientations))
        for i in range(len(angles)):
            self.all_angles[i].append(angles[i])

        for j in range(len(self.model.joints)):
            if j == 0:
                joint = self.model.joints[j]  # type: pydart.FreeJoint
                joint_idx = joint_list.index(joint.name)
                hip_angles = mm.logSO3(np.dot(root_orientation, orientations[joint_idx]))
                # hip_angles = mm.logSO3(root_orientation)
                joint.set_position(np.array([hip_angles[0], hip_angles[1], hip_angles[2], points[0][0], points[0][1], points[0][2]]))
                continue
            joint = self.model.joints[j]  # type: pydart.BallJoint
            joint_idx = joint_list.index(joint.name)
            joint.set_position(angles[joint_idx*3:joint_idx*3+3])

        self.ik.clean_constraints()
        self.ik.add_joint_pos_const('LeftForeArm', np.asarray(points[10]))
        self.ik.add_joint_pos_const('LeftHand', np.asarray(points[2]))
        self.ik.add_joint_pos_const('LeftLeg', np.asarray(points[11]))
        self.ik.add_joint_pos_const('LeftFoot', np.asarray(points[3]))
        if contacts[0] > 0.8 and False:
            body_transform = self.model.body('LeftFoot').transform()[:3, :3]
            angle = math.acos(body_transform[1, 1])
            body_ori = np.dot(body_transform, mm.rotX(-angle))
            self.ik.add_orientation_const('LeftFoot', body_ori)

        self.ik.add_joint_pos_const('RightForeArm', np.asarray(points[12]))
        self.ik.add_joint_pos_const('RightHand', np.asarray(points[5]))
        self.ik.add_joint_pos_const('RightLeg', np.asarray(points[13]))
        self.ik.add_joint_pos_const('RightFoot', np.asarray(points[6]))
        self.ik.solve()

        foot_joint_ori = mm.exp(self.model.joint('LeftFoot').position())
        self.model.joint('LeftFoot').set_position(mm.logSO3(np.dot(foot_joint_ori, np.dot(mm.rotX(-.6), mm.rotZ(.4)))))
        foot_joint_ori = mm.exp(self.model.joint('RightFoot').position())
        self.model.joint('RightFoot').set_position(mm.logSO3(np.dot(foot_joint_ori, np.dot(mm.rotX(-.6), mm.rotZ(-.4)))))

    def draw_motion(self, lines):
        glPushMatrix()
        for pair in lines:
            boneThickness = 3
            self.draw_line(pair[0], pair[1], boneThickness)
            glPushMatrix()
            glTranslated(pair[0][0], pair[0][1], pair[0][2])
            self.rc.drawSphere(boneThickness + 0.05)
            glPopMatrix()
            glPushMatrix()
            glTranslated(pair[1][0], pair[1][1], pair[1][2])
            self.rc.drawSphere(boneThickness + 0.05)
            glPopMatrix()
        glPopMatrix()

    def draw_line(self, p0, p1, radius):
        glPushMatrix()
        v = v_sub(p1, p0)
        l = v_len(v)
        base = [0, 0, 1]
        cross = v_cross(base, v)

        angle = v_angle(base, v)
        glTranslated(p0[0], p0[1], p0[2])
        glRotated(math.degrees(angle), cross[0], cross[1], cross[2])
        self.rc.drawCylinder(radius, l)
        glPopMatrix()

    def draw_target(self, ):
        glPointSize(10.0)
        glPushMatrix()
        _point = self.viewer.motionViewWnd.glWindow.pickPoint
        glTranslated(_point[0], _point[1], _point[2])
        glBegin(GL_POINTS)
        glVertex3f(0., 0., 0.)
        glEnd()
        glPopMatrix()


if __name__ == '__main__':
    ModelViewer("walk")
