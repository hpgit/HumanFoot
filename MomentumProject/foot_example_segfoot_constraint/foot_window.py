import numpy as np

from fltk import *
from OpenGL.GL import *
from PyCommon.modules.Renderer import ysRenderer as yr
from PyCommon.modules.Math import mmMath as mm

class PressureFrameInfo:
    def __init__(self):
        self.contact_seg_idx = []
        self.contact_seg_position_local = []
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


class FootWindow(Fl_Window):
    def __init__(self, x, y, w, h, title, model):
        Fl_Window.__init__(self, x, y, w, h, title)
        y_padding = 20

        self.model = model

        self.begin()

        self.check_op_l = Fl_Check_Button(10, 10, 30, 30, 'OP')
        self.check_ip_l = Fl_Check_Button(50, 10, 30, 30, 'IP')
        self.check_om_l = Fl_Check_Button(10, 50, 30, 30, 'OM')
        self.check_im_l = Fl_Check_Button(50, 50, 30, 30, 'IM')
        self.check_h_l = Fl_Check_Button(30, 90, 30, 30, 'H')

        self.check_op_r = Fl_Check_Button(150, 10, 30, 30, 'OP')
        self.check_ip_r = Fl_Check_Button(110, 10, 30, 30, 'IP')
        self.check_om_r = Fl_Check_Button(150, 50, 30, 30, 'OM')
        self.check_im_r = Fl_Check_Button(110, 50, 30, 30, 'IM')
        self.check_h_r = Fl_Check_Button(130, 90, 30, 30, 'H')

        self.foot_pressure_gl_window = FootPressureGlWindow(50, 150, 200, 200, model)

        # self.check_op_l.value(True)
        # self.check_om_l.value(True)
        # self.check_h_l.value(True)
        # self.check_op_r.value(True)
        # self.check_om_r.value(True)
        # self.check_h_r.value(True)

        self.end()

