from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

import math

from PyCommon.modules.Simulator.csVpUtil import *
from PyCommon.modules.Math import mmMath as mm

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

        for i in range(self._model.getBodyNum()):
            self.renderVpBody(i)

        if renderType != RENDER_SHADOW:
            glDisable(GL_BLEND)


    def renderVpBody(self, body_idx):
        # glPushMatrix()
        # _t = self._model.getBodyPositionGlobal(body_idx)
        # glTranslatef(_t[0], _t[1], _t[2])

        # print(body_idx, self._model.index2name(body_idx), self._model.getBodyShape(body_idx))
        # print(self._model.index2name(body_idx), self._model.getBodyGeomsType(body_idx), self._model.getBodyGeomsSize(body_idx))
        # print(self._model.index2name(body_idx), self._model.getBodyGeomsGlobalFrame(body_idx))

        geom_types = self._model.getBodyGeomsType(body_idx)
        print('renderVpBody: ', geom_types)
        geom_sizes = self._model.getBodyGeomsSize(body_idx)
        print('renderVpBody: ', geom_sizes)
        geom_frames = self._model.getBodyGeomsGlobalFrame(body_idx)
        print('renderVpBody: ', geom_frames)

        for i in range(self._model.getBodyGeomNum(body_idx)):
            glPushMatrix()
            geom_type, geom_sizes, _T = geom_types[i], geom_sizes[i], geom_frames[i]

            _t = _T[:3, 3].flatten()
            _r = mm.logSO3(_T[:3, :3])

            # print(_t)

            glTranslatef(_t[0], _t[1], _t[2])
            # glMultMatrixd(_T)

            # pGeom->GetShape(&type, data)
            if geom_type == 'B' or geom_type == 'M':
                data = .5 * geom_sizes
                # data[0] *= SCALAR_1_2
                # data[1] *= SCALAR_1_2
                # data[2] *= SCALAR_1_2
                _draw_box(data)
            elif geom_type == 'C':
                data = geom_sizes[i]
                # data.append(pGeom.GetRadius())
                # data.append(pGeom.GetHeight())
                data[1] -= 2. * data[0]
                _draw_capsule(data[0], data[1])
            elif geom_type == 'S':
                data = geom_sizes[i]
                _draw_sphere(data[0])

            glPopMatrix()

        # glPopMatrix()


