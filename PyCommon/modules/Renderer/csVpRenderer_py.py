from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import math

from PyCommon.modules.Simulator.csVpUtil import *

# RendererContext, OdeModelRenderer
POLYGON_LINE = 0
POLYGON_FILL = 1

RENDER_OBJECT = 0
RENDER_SHADOW = 1
RENDER_REFLECTION = 2

_T= []*16
_SLICE_SIZE = 24

def _draw_box(_sz):
    glPushMatrix()
    glScalef(_sz[0], _sz[1], _sz[2])
    glutSolidCube(1.)
    glPopMatrix()


def _draw_sphere(rad):
    glPushMatrix()
    glutSolidSphere(rad, _SLICE_SIZE, _SLICE_SIZE)
    glPopMatrix()


def _draw_capsule(rad, height):
    glPushMatrix()
    glutSolidSphere(rad, _SLICE_SIZE, _SLICE_SIZE)
    glutSolidCylinder(rad, height, _SLICE_SIZE, _SLICE_SIZE)
    glutSolidSphere(rad, _SLICE_SIZE, _SLICE_SIZE)
    glPopMatrix()

def renderVpNode(pNode):
    glPushMatrix()
    _T = pNode.body.GetFrame()

    _t = _T.GetPosition()
    # print _t
    _r = LogR(_T)
    print(_r)

    glTranslatef(_t[0], _t[1], _t[2])
    # glMultMatrixd(_T)


    for j in range(len(pNode.geoms)):
        pGeom = pNode.geoms[j]
        glPushMatrix()
        # _T = SE3_2_pySE3(pGeom.GetLocalFrame())
        _T = pGeom.GetLocalFrame()

        _t = _T.GetPosition()
        _r = LogR(_T)

        glTranslatef(_t[0], _t[1], _t[2])
        # glMultMatrixd(_T)

        geomType = pGeom.GetType()
        # pGeom->GetShape(&type, data)
        data = []
        if geomType ==  'B' or geomType == 'M':
            data = pGeom.GetHalfSize()
            # data[0] *= SCALAR_1_2
            # data[1] *= SCALAR_1_2
            # data[2] *= SCALAR_1_2
            _draw_box(data)
        elif geomType == 'C':
            data.append(pGeom.GetRadius())
            data.append(pGeom.GetHeight())
            data[1] -= 2. * data[0]
            _draw_capsule(data[0], data[1])
        elif geomType == 'S':
            _draw_sphere(data[0])
        glPopMatrix()
    glPopMatrix()


class VpModelRenderer:
    def __init__(self, model, color, polygonStyle=POLYGON_FILL, lineWidth=1.):
        self._model = model
        self._color = color
        self._polygonStyle = polygonStyle
        self._lineWidth = lineWidth


    def render(self, renderType):
        if self._polygonStyle == POLYGON_FILL:
            glPolygonMode(GL_FRONT, GL_FILL)
        else:
            glPolygonMode(GL_FRONT, GL_LINE)
        glLineWidth(self._lineWidth)

        if renderType == RENDER_SHADOW:
            glColor3ub(90, 90, 90)
        else:
            glColor3ubv(self._color)
            # glEnable(GL_BLEND)
            # glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        for node in self._model._nodes:
            if node is not None:
                if (renderType != RENDER_SHADOW):
                    if node.color[0] != 0 or node.color[1] != 0 or node.color[2] != 0:
                        c = [ node.color[0], node.color[1], node.color[2], node.color[3] ]
                        glColor4ubv(c)
                    else:
                        glColor3ubv(self._color)
                renderVpNode(node)

        if renderType!=RENDER_SHADOW:
            glDisable(GL_BLEND)
