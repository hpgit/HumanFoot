from fltk import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import numpy as np
import math


class Point2D(np.ndarray):
    def __new__(cls, *args):
        if len(args) == 0:
            obj = np.asarray(np.zeros(2)).view(cls)
            return obj
        elif len(args) == 1:
            if isinstance(args[0], float) or isinstance(args[0], int):
                obj = np.asarray(args[0] * np.ones(2)).view(cls)
                return obj
            elif len(args[0]) == 2:
                obj = np.asarray(np.array(args[0])).view(cls)
                return obj

            raise Exception

        elif len(args) == 2:
            if isinstance(args[0], float) and isinstance(args[1], float):
                obj = np.asarray(np.array([args[0], args[1]])).view(cls)
                return obj
            elif isinstance(args[0], int) and isinstance(args[1], int):
                obj = np.asarray(np.array([args[0], args[1]])).view(cls)
                return obj

        raise Exception

class BezierSpline:
    def __init__(self):
        self.ctrlPoint = []

    def addControlPoint(self, controlPoint):
        self.ctrlPoint.append(controlPoint)

    def setControlPoint(self, idx, controlPoint):
        self.ctrlPoint[idx] = controlPoint

    def getControlPoints(self):
        return self.ctrlPoint

    def getControlPoint(self, idx):
        return self.ctrlPoint[idx]

    def getValue(self, tt):
        if len(self.ctrlPoint) == 0:
            return Point2D()

        idx = 3*int(tt)
        t = tt - int(tt)

        if len(self.ctrlPoint) - 1 == idx:
            idx -= 3
            t = 1.

        invt = 1-t
        sqt = t*t
        sqinvt = invt*invt

        return sqinvt*invt*self.ctrlPoint[idx] + 3*sqinvt*t*self.ctrlPoint[idx+1] \
            + 3*sqt*invt*self.ctrlPoint[idx+2] + sqt*t*self.ctrlPoint[idx+3]

    def clear(self):
        del self.ctrlPoint[:]


class SplineWindow(Fl_Gl_Window):
    def __init__(self, x, y, w, h, parent = None):
        self.base = parent
        self.initGLFlag = False
        self.projectionChanged = False
        self.projectionMode = False
        Fl_Gl_Window.__init__(self, x,y,w,h)


        self.pickIdx = 0
        self.isPick = False

    def initGL(self):
        #se add
        glutInitDisplayMode(GLUT_DEPTH|GLUT_DOUBLE|GLUT_RGBA|GLUT_STENCIL)
        glClearColor(1., 1., 1., 1.)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        #se add
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        glCullFace(GL_BACK)
        glShadeModel(GL_SMOOTH)
        self.setupLights()
        # self.projectPerspective()
        self.projectOrtho(10.)

    def projectOrtho(self, distance):
        self.projectionMode = True
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # x = float(self.w())/float(self.h()) * distance
        # y = 1. * distance
        x = self.w()
        y = self.h()

        #        glOrtho(-x/2., x/2., -y/2., y/2., .1 ,1000.)
        glOrtho(-x/2., x/2., -y/2., y/2., -1000. ,1000.)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def projectPerspective(self):
        self.projectionMode = False
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        #        gluPerspective( 45., float(self.w())/float(self.h()), 0.1, 1000.)
        gluPerspective( 50., float(self.w())/float(self.h()), 0.1, 1000.)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def draw(self):
        if self.initGLFlag == True:
            self.initGL()
            self.initGLFlag = False

        glClearColor(1.0, 1.0, 1.0, 1.0);
        # glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT |GL_STENCIL_BUFFER_BIT)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


        if self.projectionChanged:
            if self.projectionMode:
                self.projectOrtho(self.camera.distance)
            else:
                self.projectPerspective()
            self.projectionChanged = False


        glLoadIdentity()

        glTranslatef(-.9, -.2, .0)
        glScalef(.5, .25, .5)

        self.draw_coord()

        glPointSize(10.)
        glColor3f(1., 0., 0.)
        glBegin(GL_POINTS)
        points = self.base.curve.getControlPoints()
        for i in range(len(points)):
            glVertex2d(points[i][0], points[i][1])
        glEnd()

        getCurve = self.base.curve.getValue
        glColor3f(1., 0., 0.)
        glBegin(GL_LINE_STRIP)
        for i in range(101):
            t = float(i)/100
            glVertex2dv(getCurve(t))
        glEnd()


        glFlush()

    def draw_coord(self):
        glPushMatrix()
        glColor3f(0., 0., 0.)
        glBegin(GL_LINES)
        glVertex2f(-2., 0.)
        glVertex2f(10., 0.)

        glVertex2f(0., -10.)
        glVertex2f(0., 10.)

        for i in range(-2, 10):
            glVertex2f(-.05, i/2.)
            glVertex2f(.05, i/2.)

        for i in range(-2, 30):
            glVertex2f(i/5., -.2)
            glVertex2f(i/5., .2)
        glEnd()

        glPopMatrix()

    def handle(self, e):

        returnVal = 0
        if e == 11:
            if self.isPick:
                x = Fl.event_x()
                y = Fl.event_y()

                modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
                projection = glGetDoublev(GL_PROJECTION_MATRIX)
                viewport = glGetIntegerv(GL_VIEWPORT)

                winX = x
                winY = viewport[3] - y
                winZ = 0

                pos_tmp = gluUnProject(winX, winY, winZ, modelview, projection, viewport)
                pos = np.array((pos_tmp[0], pos_tmp[1]))

                points = self.base.curve.getControlPoints()
                if self.pickIdx == 0 or self.pickIdx == 3:
                    points[self.pickIdx] = Point2D(points[self.pickIdx][0], pos[1])
                else:
                    points[self.pickIdx] = Point2D(pos)
                self.redraw()

        elif e == FL_PUSH:
            x = Fl.event_x()
            y = Fl.event_y()

            modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
            projection = glGetDoublev(GL_PROJECTION_MATRIX)
            viewport = glGetIntegerv(GL_VIEWPORT)

            winX = x
            winY = viewport[3] - y
            winZ = 0

            pos_tmp = gluUnProject(winX, winY, winZ, modelview, projection, viewport)
            pos = np.array((pos_tmp[0], pos_tmp[1]))

            points = self.base.curve.getControlPoints()

            if self.isPick:
                if self.pickIdx == 0 or self.pickIdx == 3:
                    points[self.pickIdx] = Point2D(points[self.pickIdx][0], pos[1])
                else:
                    points[self.pickIdx] = Point2D(pos)
                self.isPick = False
                self.pickIdx = 0
                self.redraw()

            else:
                for i in range(len(points)):
                    dist = np.linalg.norm(points[i]-pos)
                    if dist < 0.03:
                        self.isPick = True
                        self.pickIdx = i
                        break
                # print self.isPick, self.pickIdx

            returnVal = 1

        elif e == FL_RELEASE:
            pass

        elif e == FL_DRAG:
            x = Fl.event_x()
            y = Fl.event_y()
            print x,y

            modelview = []
            projection = []
            viewport = []
            modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
            projection = glGetDoublev(GL_PROJECTION_MATRIX)
            viewport = glGetIntegerv(GL_VIEWPORT)

            winX = x
            # winY = viewport[3] - y
            winY = viewport[3] - y
            winZ = 0

            pos_tmp = gluUnProject(winX, winY, winZ, modelview, projection, viewport)
            pos = np.array((pos_tmp[0], pos_tmp[1]))
            print pos

            points = self.base.curve.getControlPoints()
            points[self.pickIdx][0] = pos[0]
            points[self.pickIdx][1] = pos[1]
            self.redraw()
            returnVal = 1

        if e == FL_KEYUP:
            print Fl.event_key()
            returnVal = 1

        return Fl_Gl_Window.handle(self, e)
        # return returnVal



class SplineUi(Fl_Window):
    def __init__(self, x, y, w, z, parent=None):
        Fl_Window.__init__(self, x, y, w, z)
        self.base = parent



class SplineEditor(Fl_Window):
    def __init__(self, rect=None, title='SplineEditor'):
        if False and rect is not None:
            settings.x = rect[0]
            settings.y = rect[1]
            settings.w = rect[2]
            settings.h = rect[3]
        Fl_Window.__init__(self, 300, 400, 1400, 400, title)

        self.curve = BezierSpline()
        self.curve.addControlPoint(Point2D(0., 0.))
        self.curve.addControlPoint(Point2D(1., 0.))
        self.curve.addControlPoint(Point2D(2., 0.))
        self.curve.addControlPoint(Point2D(3., 0.))

        self.begin()
        self.splinewindow = SplineWindow(0, 0, 1200, 400, self)
        self.splineui = SplineUi(1200, 0, 200, 400, self)
        self.end()

        self.callback(self.onClose)


    def show(self):
        if False and len(self.settingsFile)>0:
            self.settings.load(self.settingsFile)
            self.settings.setToApp(self)
        Fl_Window.show(self)
    def onClose(self, data):
        if False and len(self.settingsFile)>0:
            self.settings.getFromApp(self)
            self.settings.save(self.settingsFile)
        self.default_callback(self, data)


if __name__=='__main__':
    window = SplineEditor()
    window.show()
    Fl.run()