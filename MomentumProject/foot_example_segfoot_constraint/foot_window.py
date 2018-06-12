import os
from time import gmtime, strftime
from PyCommon.modules.GUI.csDump import dump_png
import numpy as np

from fltk import *
from OpenGL.GL import *
from PyCommon.modules.Renderer import ysRenderer as yr
from PyCommon.modules.Math import mmMath as mm


class PressureFrameInfo:
    def __init__(self):
        self.contact_body_idx = []
        self.contact_seg_idx = []
        self.contact_seg_position_local = []
        self.contact_seg_position_geom_local = []
        self.contact_seg_forces = []


class FootPressureGlWindow(Fl_Gl_Window):
    def __init__(self, x, y, w, h, model):
        Fl_Gl_Window.__init__(self, x, y, w, h)

        self.initGL()
        self.rc = yr.RenderContext()
        self.model = model

        self.foot_index = []
        self.left_foot_index = []
        self.right_foot_index = []
        self.foot_seg_index = []
        self.left_seg_index = []
        self.right_seg_index = []

        self.geom_ids = []
        self.geom_names = []
        self.geom_types = []
        self.geom_trans = []
        self.geom_sizes = []
        self.body_trans = []

        self.pressure_info = {}  # type: dict[int, PressureFrameInfo]

        self.frame = -1

        self.init_model()

    def init_model(self):
        for joint_idx in range(self.model.getJointNum()):
            joint_name = self.model.index2name(joint_idx)
            if 'Foot' in joint_name:
                self.foot_index.append(joint_idx)
                if 'Left' in joint_name:
                    self.left_foot_index.append(joint_idx)
                elif 'Right' in joint_name:
                    self.right_foot_index.append(joint_idx)

            if 'foot' in joint_name:
                self.foot_seg_index.append(joint_idx)
                if 'Left' in joint_name:
                    self.left_seg_index.append(joint_idx)
                elif 'Right' in joint_name:
                    self.right_seg_index.append(joint_idx)

        q = self.model.get_q()
        q_zero = np.zeros_like(q)
        self.model.set_q(q_zero)
        for seg_idx in self.foot_seg_index:
            for i in range(self.model.getBodyGeomNum(seg_idx)):
                self.geom_names.append(self.model.index2name(seg_idx))
                self.geom_ids.append(seg_idx)
                self.body_trans.append(self.model.getBodyTransformGlobal(seg_idx))
            self.geom_types.extend(self.model.getBodyGeomsType(seg_idx))
            self.geom_sizes.extend(self.model.getBodyGeomsSize(seg_idx))
            self.geom_trans.extend(self.model.getBodyGeomsGlobalFrame(seg_idx))

        self.model.set_q(q)

    def refresh_foot_contact_info(self, frame, world, bodyIDsToCheck, mus, Ks, Ds):
        if frame not in self.pressure_info:
            self.pressure_info[frame] = PressureFrameInfo()
            bodyIDs, contactPositions, contactPositionLocals, contactForces = world.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
            self.pressure_info[frame].contact_seg_idx.extend(bodyIDs)
            self.pressure_info[frame].contact_seg_position_local.extend(contactPositionLocals)
            self.pressure_info[frame].contact_seg_forces.extend(contactForces)
        self.redraw()

    def goToFrame(self, frame):
        self.frame = frame

    def initGL(self):
        glClearColor(1., 1., 1., 1.)
        self.projectOrtho(3)

    def draw(self):
        frame = self.frame
        glClear(GL_COLOR_BUFFER_BIT)
        # self.rc.drawCircle(1.)
        # self.rc.drawCapsule2D(.2, .2)
        force_max = None
        if self.pressure_info:
            force_max = max([mm.length(force) for force in self.pressure_info[frame].contact_seg_forces]) if self.pressure_info[frame].contact_seg_forces else 1000.
        for i in range(len(self.geom_types)):
            geom_seg_idx = self.geom_ids[i]
            geom_name = self.geom_names[i]
            geom_type = self.geom_types[i]
            geom_size = self.geom_sizes[i]
            geom_tran = self.geom_trans[i].copy()
            geom_body_tran = self.body_trans[i].copy()
            geom_tran[0, 3], geom_tran[1, 3], geom_tran[2, 3] = -geom_tran[0, 3], geom_tran[2, 3], 0.
            # geom_tran[1, 3] = geom_tran[2, 3]
            # geom_tran[2, 3] = 0
            geom_body_tran[0, 3], geom_body_tran[1, 3], geom_body_tran[2, 3] = -geom_body_tran[0, 3], geom_body_tran[2, 3], 0.
            # geom_body_tran[1, 3] = geom_body_tran[2, 3]
            # geom_body_tran[2, 3] = 0

            if False and geom_type is 'ELLIPSOID':
                # print(geom_tran)
                glPushMatrix()
                glMultMatrixf(geom_tran.T)
                glScalef(geom_size[0], geom_size[1], geom_size[2])
                self.rc.drawCircle(1.)
                glPopMatrix()
            elif geom_type in ('B', 'BOX'):
                pass

            if geom_type in ('C', 'D', 'E'):
                # print(geom_seg_idx, geom_name)
                # print(geom_tran)
                # print(geom_body_tran)
                glPushMatrix()
                if 'Left' in geom_name:
                    glTranslatef(-0.3, -0.3, 0.)
                else:
                    glTranslatef(0.3, -0.3, 0.)
                # glRotatef(180., 0., 1., 0.)
                glScalef(4., 4., 4.)
                glPushMatrix()
                glMultMatrixf(geom_tran.T)
                glColor3f(1., 1., 1.)
                self.rc.drawCapsule2D(geom_size[0], geom_size[1] - 2.*geom_size[0])
                glPopMatrix()

                # draw distribution of forces
                glMultMatrixf(geom_body_tran.T)

                if self.pressure_info:
                    for contact_idx in np.where(np.array(self.pressure_info[frame].contact_seg_idx) == geom_seg_idx)[0]:
                        glPushMatrix()
                        trans = self.pressure_info[frame].contact_seg_position_local[contact_idx]
                        # print(geom_seg_idx, geom_name, trans, geom_size[0])
                        # print(mm.length(self.contact_seg_forces[contact_idx]))
                        normalized_force = mm.length(self.pressure_info[frame].contact_seg_forces[contact_idx])/force_max
                        glTranslatef(trans[0], trans[1], trans[2])
                        if normalized_force < 0.5:
                            glColor3f(0., 2.*normalized_force, 1. - 2.*normalized_force)
                        else:
                            glColor3f(2.*(normalized_force-0.5), 1. - 2.*(normalized_force-0.5), 0.)
                        # glColor3f(1., 0., 0.)
                        self.rc.drawSphere(geom_size[0])
                        glPopMatrix()
                glPopMatrix()
            elif geom_type is 'CYLINDER':
                glPushMatrix()
                if 'Left' in geom_name:
                    glTranslatef(-0.3, -0.3, 0.)
                else:
                    glTranslatef(0.3, -0.3, 0.)
                glRotatef(180., 0., 1., 0.)
                glScalef(4., 4., 4.)
                glMultMatrixf(geom_tran.T)
                if geom_seg_idx in self.pressure_info[frame].contact_seg_idx:
                    glColor3f(1., 0., 0.)
                else:
                    glColor3f(1., 1., 1.)
                self.rc.drawCapsule2D(geom_size[1], geom_size[0] - 2.*geom_size[1])
                glPopMatrix()

    def projectOrtho(self, distance):
        glViewport(0, 0, self.w(), self.h())
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        x = float(self.w())/float(self.h()) * distance
        y = 1. * distance
        glOrtho(-.5*x, .5*x, -.5*y, .5*y, -1000., 1000.)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()


class FootContactGlWindow(Fl_Gl_Window):
    def __init__(self, x, y, w, h, model):
        Fl_Gl_Window.__init__(self, x, y, w, h)

        self.initGL()
        self.rc = yr.RenderContext()
        self.model = model
        self.idDic = dict()
        for i in range(self.model.getJointNum()):
            self.idDic[i] = self.model.index2name(i)

        self.foot_index = []
        self.left_foot_index = []
        self.right_foot_index = []
        self.foot_seg_index = []
        self.left_seg_index = []
        self.right_seg_index = []

        self.geom_ids = []
        self.geom_names = []
        self.geom_types = []
        self.geom_trans = []
        self.geom_sizes = []
        self.body_trans = []

        self.pressure_info = {}  # type: dict[int, PressureFrameInfo]

        self.frame = -1

    def refresh_foot_contact_info(self, frame, world, bodyIDsToCheck, mus, Ks, Ds):
        if frame not in self.pressure_info.keys():
            self.pressure_info[frame] = PressureFrameInfo()
            bodyIDs, geomIDs, contactPositionGeomLocals = world.getContactInfoForcePlate(bodyIDsToCheck)
            self.pressure_info[frame].contact_body_idx.extend(bodyIDs)
            self.pressure_info[frame].contact_seg_idx.extend(geomIDs)
            self.pressure_info[frame].contact_seg_position_geom_local.extend(contactPositionGeomLocals)
        self.redraw()

    def draw_foot(self, side):
        sign = 1
        if side == 'Right':
            sign = -1

        # inside phalanges
        glPushMatrix()
        glTranslatef(sign*-0.1823, 1.0397, 0.)
        # glTranslatef(0., 0.18, 0.)
        glTranslatef(0., 0.1*6.63*0.5, 0.)
        self.rc.drawCapsule2D(0.08, 0.1*6.63)
        glTranslatef(sign*0.18, 0., 0.)
        self.rc.drawCapsule2D(0.08, 0.1*6.63)
        glTranslatef(sign*0.18, 0., 0.)
        self.rc.drawCapsule2D(0.08, 0.1*6.63)
        glPopMatrix()

        # outside metatarsal
        glPushMatrix()
        glTranslatef(sign*(-0.2784-0.0773), 0.452, 0.)
        glTranslatef(0., 0.1*5.877*0.5, 0.)
        self.rc.drawCapsule2D(0.08, 0.1*5.877)
        glTranslatef(sign*-0.18, 0., 0.)
        self.rc.drawCapsule2D(0.08, 0.1*5.877)
        glPopMatrix()

        # outside phalanges
        glPushMatrix()
        glTranslatef(sign*-0.2784, 0.452, 0.)
        glTranslatef(sign*-0.0773, 0.5877, 0.)
        # glTranslatef(0., 0.18, 0.)
        glTranslatef(0., 0.1*5.25*0.5, 0.)
        self.rc.drawCapsule2D(0.08, 0.1 * 5.25)
        glTranslatef(sign*-0.18, 0., 0.)
        self.rc.drawCapsule2D(0.08, 0.1 * 5.25)
        glPopMatrix()

        # heel
        glPushMatrix()
        glTranslatef(0., -0.1, 0.)
        glTranslatef(0., -0.1*3.6*0.5, 0.)
        glTranslatef(sign*0.09, 0., 0.)
        self.rc.drawCapsule2D(0.08, 0.1 * 3.6)
        glTranslatef(sign*-0.18, 0., 0.)
        self.rc.drawCapsule2D(0.08, 0.1 * 3.6)
        glPopMatrix()

    def draw_contact(self, side, frame):
        if frame >= 0:
            sign = 1
            if side == 'Right':
                sign = -1
            for i in range(len(self.pressure_info[frame].contact_body_idx)):
                contact_body_idx = self.pressure_info[frame].contact_body_idx[i]
                if side not in self.idDic[contact_body_idx]:
                    continue
                contact_seg_idx = self.pressure_info[frame].contact_seg_idx[i]
                contact_pos = self.pressure_info[frame].contact_seg_position_geom_local[i][2]
                if '0_1_0' in self.idDic[contact_body_idx]:
                    # inside phalanges
                    glPushMatrix()
                    glTranslatef(sign*-0.1823, 1.0397, 0.)
                    if contact_seg_idx == 1:
                        glTranslatef(sign*0.18, 0., 0.)
                    if contact_seg_idx == 2:
                        glTranslatef(sign*0.36, 0., 0.)
                    glTranslatef(0., 0.1*6.63*0.5, 0.)
                    if contact_pos/6.63 > 0.005:
                        contact_pos = 6.63*0.005
                    elif contact_pos/6.63 < -0.005:
                        contact_pos = -6.63*0.005
                    glTranslatef(0., 10.*contact_pos, 0.)
                    self.rc.drawCircleFilled(0.08)
                    glPopMatrix()

                elif '0_0_0' in self.idDic[contact_body_idx]:
                    # outside phalanges
                    glPushMatrix()
                    glTranslatef(sign*-0.2784, 0.452, 0.)
                    glTranslatef(sign*-0.0773, 0.5877, 0.)
                    glTranslatef(0., 0.1*5.25*0.5, 0.)
                    if contact_seg_idx == 1:
                        glTranslatef(sign*-0.18, 0., 0.)
                    if contact_pos/5.25 > 0.005:
                        contact_pos = 5.25*0.005
                    elif contact_pos/5.25 < -0.005:
                        contact_pos = -5.25*0.005
                    glTranslatef(0., 10.*contact_pos, 0.)
                    self.rc.drawCircleFilled(0.08)
                    glPopMatrix()

                elif '0_0' in self.idDic[contact_body_idx]:
                    # outside metatarsal
                    glPushMatrix()
                    glTranslatef(sign*(-0.2784-0.0773), 0.452, 0.)
                    glTranslatef(0., 0.1*5.877*0.5, 0.)
                    if contact_seg_idx == 0:
                        if contact_pos/6.236 > 0.005:
                            contact_pos = 6.236*0.005
                        elif contact_pos/6.236 < -0.005:
                            contact_pos = -6.236*0.005
                        glTranslatef(0., 10.*contact_pos/6.236*5.877, 0.)
                    elif contact_seg_idx == 1:
                        glTranslatef(sign*-0.18, 0., 0.)
                        if contact_pos/5.928 > 0.005:
                            contact_pos = 5.928*0.005
                        if contact_pos/5.928 < -0.005:
                            contact_pos = -5.928*0.005
                        glTranslatef(0., 10.*contact_pos/5.928*5.877, 0.)
                    self.rc.drawCircleFilled(0.08)
                    glPopMatrix()

                elif '1_0' in self.idDic[contact_body_idx]:
                    # heel
                    glPushMatrix()
                    glTranslatef(0., -0.1, 0.)
                    glTranslatef(0., -0.1*3.6*0.5, 0.)
                    if contact_seg_idx == 0:
                        glTranslatef(sign*0.09, 0., 0.)
                    elif contact_seg_idx == 1:
                        glTranslatef(sign*-0.09, 0., 0.)
                    if contact_pos/3.6 > 0.005:
                        contact_pos = 3.6*0.005
                    elif contact_pos/3.6 < -0.005:
                        contact_pos = -3.6*0.005
                    glTranslatef(0., -10.*contact_pos, 0.)
                    self.rc.drawCircleFilled(0.08)
                    glPopMatrix()

    def draw(self):
        frame = self.frame
        glClearColor(1., 1., 1., 1.)
        glClear(GL_COLOR_BUFFER_BIT)

        glPushMatrix()
        glScalef(0.5, 0.5, 0.5)

        glPushMatrix()
        # glColor3f(200, 200, 200)
        glColor3ub(50, 50, 50)
        glTranslatef(-0.7, -0.5, 0.)
        self.draw_foot('Left')
        glTranslatef(1.4, 0., 0.)
        self.draw_foot('Right')
        glPopMatrix()

        glPushMatrix()
        glColor3ub(200, 0, 0)
        glTranslatef(-0.7, -0.5, 0.01)
        self.draw_contact('Left', frame)
        glTranslatef(1.4, 0., 0.)
        self.draw_contact('Right', frame)
        glPopMatrix()

        glPopMatrix()

    def goToFrame(self, frame):
        self.frame = frame
        self.redraw()

    def initGL(self):
        # glClearColor(0., 0., 0., 1.)
        glClearColor(1., 1., 1., 1.)
        self.projectOrtho(3)

    def projectOrtho(self, distance):
        glViewport(0, 0, self.w(), self.h())
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        x = float(self.w())/float(self.h()) * distance
        y = 1. * distance
        glOrtho(-.5*x, .5*x, -.5*y, .5*y, -1000., 1000.)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def test(self):
        FramebufferName = 0
        glGenFramebuffers(1, FramebufferName)
        glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName)
        glGenTextures(1, renderedTexture)

        glBindTexture(GL_TEXTURE_2D, renderedTexture)

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 768, 0, GL_RGB, GL_UNSIGNED_BYTE, 0)

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

        glGenRenderbuffers(1, depthrenderbuffer)
        glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer)
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, 1024, 768)
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer)

        glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);

        DrawBuffers[1] = {GL_COLOR_ATTACHMENT0}
        glDrawBuffers(1, DrawBuffers)

        glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
        glViewport(0, 0, 1024, 768);

    def dump(self, ptr, parent):
        session_name = parent.dump_name.value()
        if len(session_name) == 0:
            session_name = '_movtmp'
        dumping_session = session_name + strftime("%Y%m%d%H%M")
        dumping_start_frame = int(parent.dump_start_frame.value())
        dumping_end_frame = int(parent.dump_end_frame.value())

        if not os.path.exists('dump/'+dumping_session):
            os.makedirs('dump/'+dumping_session)

        for frame in range(dumping_start_frame, dumping_end_frame):
            self.goToFrame(frame)
            dump_png('dump/' + dumping_session + '/' + '{:04d}'.format(frame-dumping_start_frame) + ".png", self.w(), self.h())

        os.system('ffmpeg -loglevel 0 -framerate 30 -i dump/'+dumping_session+'/%04d.png -vcodec libx264 -crf 20 -pix_fmt yuv420p dump/' + dumping_session+'.mp4')
        os.system('ffplay -loglevel 0 dump/'+dumping_session+'.mp4 &')

    def dump_single(self):
        dump_png('dump_force_plate/'+'{:04d}'.format(self.frame) + ".png", self.w(), self.h())



class FootWindow(Fl_Window):
    def __init__(self, x, y, w, h, title, model):
        Fl_Window.__init__(self, x, y, w, h, title)
        y_padding = 20

        self.ENABLE_INSIDE_METATARSAL = False

        self.model = model

        self.begin()

        self.check_op_l = Fl_Check_Button(10, 10, 30, 30, 'OP')
        self.check_ip_l = Fl_Check_Button(50, 10, 30, 30, 'IP')
        self.check_om_l = Fl_Check_Button(10, 50, 30, 30, 'OM')
        self.check_im_l = None
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_l = Fl_Check_Button(50, 50, 30, 30, 'IM')
        self.check_h_l = Fl_Check_Button(30, 90, 30, 30, 'H')

        self.check_op_r = Fl_Check_Button(150, 10, 30, 30, 'OP')
        self.check_ip_r = Fl_Check_Button(110, 10, 30, 30, 'IP')
        self.check_om_r = Fl_Check_Button(150, 50, 30, 30, 'OM')
        self.check_im_r = None
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_r = Fl_Check_Button(110, 50, 30, 30, 'IM')
        self.check_h_r = Fl_Check_Button(130, 90, 30, 30, 'H')

        # self.foot_pressure_gl_window = FootPressureGlWindow(50, 150, 200, 200, model)
        # self.foot_pressure_gl_window = FootContactGlWindow(50, 150, 200, 200, model)
        self.foot_pressure_gl_window = FootContactGlWindow(50, 150, self.h()-300, self.h()-300, model)

        self.dump_btn = Fl_Button(250, 10, 30, 20, 'dump')
        self.dump_btn.callback(self.foot_pressure_gl_window.dump, self)
        self.dump_start_frame = Fl_Value_Input(250, 40, 40, 20, 's')
        self.dump_start_frame.value(1)
        self.dump_end_frame = Fl_Value_Input(250, 70, 40, 20, 'e')
        self.dump_end_frame.value(300)
        self.dump_name = Fl_Input(250, 100, 100, 20, 'name')
        self.dump_name.value('_movtmp')

        self.end()

    def check_left_seg(self):
        self.check_op_l.value(True)
        self.check_ip_l.value(True)
        self.check_om_l.value(True)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_l.value(True)
        self.check_h_l.value(True)

    def check_right_seg(self):
        self.check_op_r.value(True)
        self.check_ip_r.value(True)
        self.check_om_r.value(True)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_r.value(True)
        self.check_h_r.value(True)

    def check_all_seg(self):
        self.check_left_seg()
        self.check_right_seg()

    def check_not_left_seg(self):
        self.check_op_l.value(False)
        self.check_ip_l.value(False)
        self.check_om_l.value(False)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_l.value(False)
        self.check_h_l.value(False)

    def check_not_right_seg(self):
        self.check_op_r.value(False)
        self.check_ip_r.value(False)
        self.check_om_r.value(False)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_r.value(False)
        self.check_h_r.value(False)

    def check_not_all_seg(self):
        self.check_not_left_seg()
        self.check_not_right_seg()

    def check_tiptoe_all(self):
        self.check_tiptoe_left()
        self.check_tiptoe_right()

    def check_tiptoe_left(self):
        self.check_op_l.value(True)
        self.check_ip_l.value(True)
        self.check_om_l.value(False)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_l.value(False)
        self.check_h_l.value(False)

    def check_tiptoe_right(self):
        self.check_op_r.value(True)
        self.check_ip_r.value(True)
        self.check_om_r.value(False)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_r.value(False)
        self.check_h_r.value(False)

    def check_tilt_left_all(self):
        self.check_op_l.value(True)
        self.check_ip_l.value(False)
        self.check_om_l.value(True)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_l.value(False)
        # self.check_h_l.value(False)
        self.check_h_l.value(True)

        self.check_op_r.value(False)
        self.check_ip_r.value(True)
        self.check_om_r.value(False)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_r.value(True)
        self.check_h_r.value(True)

    def check_tilt_right_all(self):
        self.check_op_l.value(False)
        self.check_ip_l.value(True)
        self.check_om_l.value(False)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_l.value(True)
        self.check_h_l.value(True)

        self.check_op_r.value(True)
        self.check_ip_r.value(False)
        self.check_om_r.value(True)
        if self.ENABLE_INSIDE_METATARSAL:
            self.check_im_r.value(False)
        self.check_h_r.value(False)

    def check_heel_off(self):
        self.check_h_l.value(False)
        self.check_h_r.value(False)

    def get_contact_state(self):
        contact = 0
        contact = contact+1 if self.check_h_r.value() or self.check_op_r.value() or self.check_om_r.value() or self.check_ip_r.value() else contact+0
        contact = contact+2 if self.check_h_l.value() or self.check_op_l.value() or self.check_om_l.value() or self.check_ip_l.value() else contact+0

        return contact
