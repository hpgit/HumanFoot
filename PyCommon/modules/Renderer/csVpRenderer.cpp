#include "stdafx.h"

#ifdef WIN32
#include <windows.h> 
#endif
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
//#include <gl/glut.h>
 
#include "boostPythonUtil.h"
#include <VP/vphysics.h>

#include "csVpRenderer.h"
#include "../Simulator/csVpModel.h"


static scalar _T[16];
#define _SLICE_SIZE		24
#ifndef M_PI_2	
#define M_PI_2		1.57079632679489661923
#endif

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(render_overloads, render, 0, 1);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(renderFrame_overloads, renderFrame, 1, 2);

BOOST_PYTHON_MODULE(csVpRenderer)
{
    class_<VpModelRenderer>("VpModelRenderer", init<VpMotionModel*, const boost::python::tuple&, optional<int, double> >())
        .def(init<VpControlModel*, const boost::python::tuple&, optional<int, double> >())
        .def("render", &VpModelRenderer::render, render_overloads())
        .def("renderFrame", &VpModelRenderer::renderFrame, renderFrame_overloads())
        .def("saveState", &VpModelRenderer::saveState)
        .def("get_max_saved_frame", &VpModelRenderer::get_max_saved_frame)
        ;
    scope().attr("POLYGON_FILL") = POLYGON_FILL;
    scope().attr("POLYGON_LINE") = POLYGON_LINE;
}

void _draw_box(const scalar _sz[3])
{
    float sz[3]		= { (float)_sz[0], (float)_sz[1], (float)_sz[2] };

    float data[][8] = {	{  0.0f,  0.0f,  0.0f,  0.0f,  1.0f, -sz[0], -sz[1],  sz[2] },
    {  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,  sz[0], -sz[1],  sz[2] },
    {  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,  sz[0],  sz[1],  sz[2] },
    {  0.0f,  1.0f,  0.0f,  0.0f,  1.0f, -sz[0],  sz[1],  sz[2] },
    {  1.0f,  0.0f,  0.0f,  0.0f, -1.0f, -sz[0], -sz[1], -sz[2] },
    {  1.0f,  1.0f,  0.0f,  0.0f, -1.0f, -sz[0],  sz[1], -sz[2] },
    {  0.0f,  1.0f,  0.0f,  0.0f, -1.0f,  sz[0],  sz[1], -sz[2] },
    {  0.0f,  0.0f,  0.0f,  0.0f, -1.0f,  sz[0], -sz[1], -sz[2] },
    {  0.0f,  1.0f,  0.0f,  1.0f,  0.0f, -sz[0],  sz[1], -sz[2] },
    {  0.0f,  0.0f,  0.0f,  1.0f,  0.0f, -sz[0],  sz[1],  sz[2] },
    {  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,  sz[0],  sz[1],  sz[2] },
    {  1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  sz[0],  sz[1], -sz[2] },
    {  1.0f,  1.0f,  0.0f, -1.0f,  0.0f, -sz[0], -sz[1], -sz[2] },
    {  0.0f,  1.0f,  0.0f, -1.0f,  0.0f,  sz[0], -sz[1], -sz[2] },
    {  0.0f,  0.0f,  0.0f, -1.0f,  0.0f,  sz[0], -sz[1],  sz[2] },
    {  1.0f,  0.0f,  0.0f, -1.0f,  0.0f, -sz[0], -sz[1],  sz[2] },
    {  1.0f,  0.0f,  1.0f,  0.0f,  0.0f,  sz[0], -sz[1], -sz[2] },
    {  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  sz[0],  sz[1], -sz[2] },
    {  0.0f,  1.0f,  1.0f,  0.0f,  0.0f,  sz[0],  sz[1],  sz[2] },
    {  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,  sz[0], -sz[1],  sz[2] },
    {  0.0f,  0.0f, -1.0f,  0.0f,  0.0f, -sz[0], -sz[1], -sz[2] },
    {  1.0f,  0.0f, -1.0f,  0.0f,  0.0f, -sz[0], -sz[1],  sz[2] },
    {  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -sz[0],  sz[1],  sz[2] },
    {  0.0f,  1.0f, -1.0f,  0.0f,  0.0f, -sz[0],  sz[1], -sz[2] }	};

    glInterleavedArrays(GL_T2F_N3F_V3F, 0, data);
    glDrawArrays(GL_QUADS, 0, 24);
}

void _draw_sphere(const scalar &rad)
{
    glBegin(GL_QUAD_STRIP);
    for ( int i = 0; i < _SLICE_SIZE - 1; i++ )
    {
        scalar t = (scalar)i / (scalar)(_SLICE_SIZE - 1);
        scalar t2 = (scalar)(i + 1) / (scalar)(_SLICE_SIZE - 1);
        scalar cp = cos((scalar)M_PI * t - (scalar)M_PI_2);
        scalar sp = sin((scalar)M_PI * t - (scalar)M_PI_2);
        scalar cp2 = cos((scalar)M_PI * t2 - (scalar)M_PI_2);
        scalar sp2 = sin((scalar)M_PI * t2 - (scalar)M_PI_2);

        for ( int j = 0; j < _SLICE_SIZE; j++ )
        {
            scalar s = (scalar)j / (scalar)(_SLICE_SIZE - 1);
            scalar ct = cos(M_2PI * s);
            scalar st = sin(M_2PI * s);

            glTexCoord2d(s, t);
            glNormal3d(cp * ct, sp,  -cp * st);
            glVertex3d(rad * cp * ct, rad * sp,  -rad * cp * st);

            glTexCoord2d(s, t2);
            glNormal3d(cp2 * ct, sp2,  -cp2 * st);
            glVertex3d(rad * cp2 * ct, rad * sp2,  -rad * cp2 * st);
        }
    }
    glEnd();
}

void _draw_capsule(const scalar &rad, const scalar &height)
{
    int i, j;
    scalar ct_i, st_i, ct_im1 = SCALAR_1, st_im1 = SCALAR_0, cp_i, sp_i, cp_im1, sp_im1;

    glDisable(GL_CULL_FACE);
    glBegin(GL_QUADS);
    for ( i = 1; i < _SLICE_SIZE + 1; i++ )
    {
        ct_i = cos(M_2PI * (scalar)i / (scalar)_SLICE_SIZE);
        st_i = sin(M_2PI * (scalar)i / (scalar)_SLICE_SIZE);

        glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)SCALAR_0);
        glNormal3d(ct_im1, st_im1, SCALAR_0);
        glVertex3d(rad * ct_im1, rad * st_im1, -SCALAR_1_2 * height);
        glTexCoord2d((double)i / (double)_SLICE_SIZE, SCALAR_0);
        glNormal3d(ct_i, st_i, SCALAR_0);
        glVertex3d(rad * ct_i, rad * st_i, -SCALAR_1_2 * height);
        glTexCoord2d((double)i / (double)_SLICE_SIZE, SCALAR_1);
        glNormal3d(ct_i, st_i, SCALAR_0);
        glVertex3d(rad * ct_i, rad * st_i, SCALAR_1_2 * height);
        glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, SCALAR_1);
        glNormal3d(ct_im1, st_im1, SCALAR_0);
        glVertex3d(rad * ct_im1, rad * st_im1, SCALAR_1_2 * height);

        cp_im1 = SCALAR_1;
        sp_im1 = SCALAR_0;
        for ( j = 1; j < _SLICE_SIZE + 1; j++ )
        {
            cp_i = cos(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);
            sp_i = sin(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);

            glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
            glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  sp_im1);
            glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
            glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
            glNormal3d(ct_i * cp_im1, st_i * cp_im1,  sp_im1);
            glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
            glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
            glNormal3d(ct_i * cp_i, st_i * cp_i,  sp_i);
            glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, rad * sp_i + SCALAR_1_2 * height);
            glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
            glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  sp_i);
            glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, rad * sp_i + SCALAR_1_2 * height);

            glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
            glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  -sp_im1);
            glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
            glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
            glNormal3d(ct_i * cp_im1, st_i * cp_im1,  -sp_im1);
            glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
            glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
            glNormal3d(ct_i * cp_i, st_i * cp_i,  -sp_i);
            glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, -rad * sp_i - SCALAR_1_2 * height);
            glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
            glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  -sp_i);
            glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, -rad * sp_i - SCALAR_1_2 * height);

            cp_im1 = cp_i;
            sp_im1 = sp_i;
        }

        ct_im1 = ct_i;
        st_im1 = st_i;
    }
    glEnd();
}

void _draw_cylinder(const scalar &rad, const scalar &height)
{
    int i, j;
    scalar ct_i, st_i, ct_im1 = SCALAR_1, st_im1 = SCALAR_0, cp_i, sp_i, cp_im1, sp_im1;

    glBegin(GL_QUADS);
    for ( i = 1; i < _SLICE_SIZE + 1; i++ )
    {
        ct_i = cos(M_2PI * (scalar)i / (scalar)_SLICE_SIZE);
        st_i = sin(M_2PI * (scalar)i / (scalar)_SLICE_SIZE);

        glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)SCALAR_0);
        glNormal3d(ct_im1, st_im1, SCALAR_0);
        glVertex3d(rad * ct_im1, rad * st_im1, -SCALAR_1_2 * height);
        glTexCoord2d((double)i / (double)_SLICE_SIZE, SCALAR_0);
        glNormal3d(ct_i, st_i, SCALAR_0);
        glVertex3d(rad * ct_i, rad * st_i, -SCALAR_1_2 * height);
        glTexCoord2d((double)i / (double)_SLICE_SIZE, SCALAR_1);
        glNormal3d(ct_i, st_i, SCALAR_0);
        glVertex3d(rad * ct_i, rad * st_i, SCALAR_1_2 * height);
        glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, SCALAR_1);
        glNormal3d(ct_im1, st_im1, SCALAR_0);
        glVertex3d(rad * ct_im1, rad * st_im1, SCALAR_1_2 * height);

        cp_im1 = SCALAR_1;
        sp_im1 = SCALAR_0;
//		for ( j = 1; j < _SLICE_SIZE + 1; j++ )
//		{
//			cp_i = cos(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);
//			sp_i = sin(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);
//
//			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
//			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  sp_im1);
//			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
//			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
//			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  sp_im1);
//			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
//			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
//			glNormal3d(ct_i * cp_i, st_i * cp_i,  sp_i);
//			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, rad * sp_i + SCALAR_1_2 * height);
//			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
//			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  sp_i);
//			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, rad * sp_i + SCALAR_1_2 * height);
//
//			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
//			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  -sp_im1);
//			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
//			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
//			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  -sp_im1);
//			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
//			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
//			glNormal3d(ct_i * cp_i, st_i * cp_i,  -sp_i);
//			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, -rad * sp_i - SCALAR_1_2 * height);
//			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
//			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  -sp_i);
//			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, -rad * sp_i - SCALAR_1_2 * height);
//
//			cp_im1 = cp_i;
//			sp_im1 = sp_i;
//		}

        ct_im1 = ct_i;
        st_im1 = st_i;
    }
    glEnd();
}


void renderVpBody(const vpBody* pBody)
{
    glPushMatrix();
    pBody->GetFrame().ToArray(_T);

    glMultMatrixd(_T);

    const vpGeom *pGeom;
    char type;
    scalar data[3];
    for ( int j = 0; j < pBody->GetNumGeometry(); j++ )
    {
        pGeom = pBody->GetGeometry(j);
        glPushMatrix();
        pGeom->GetLocalFrame().ToArray(_T);
        glMultMatrixd(_T);

        pGeom->GetShape(&type, data);
        switch ( type )
        {
        case 'B':
        case 'M':
        case 'N':
            data[0] *= SCALAR_1_2;
            data[1] *= SCALAR_1_2;
            data[2] *= SCALAR_1_2;
            _draw_box(data);
            break;
        case 'C':
        case 'D':
        case 'E':
        {

            data[1] -= SCALAR_2 * data[0];
            _draw_capsule(data[0], data[1]);

        #if 0
            glPointSize(2.);

            glColor3f(1.0, 0.0, 0.0);
            glBegin(GL_POINTS);
            const vector<Vec3>& verticesLocal = pGeom->getVerticesLocal();
            for (int i = 0; i < verticesLocal.size(); ++i)
                glVertex3d(verticesLocal[i][0], verticesLocal[i][1], verticesLocal[i][2]);
            glEnd();

            glPointSize(1.);
        #endif

        }
            break;
        case 'S':
            _draw_sphere(data[0]);
            break;
        }
        glPopMatrix();
    }
    glPopMatrix();
}



VpModelRenderer::VpModelRenderer(VpModel* pModel, const boost::python::tuple& color, int polygonStyle, double lineWidth)
{
    _pModel = pModel;

    _color[0] = (GLubyte)XI(color[0]);
    _color[1] = (GLubyte)XI(color[1]);
    _color[2] = (GLubyte)XI(color[2]);

    _polygonStyle = polygonStyle;
    _lineWidth = lineWidth;

    max_frame = -1;
}

void VpModelRenderer::render(int renderType)
{
    if(_polygonStyle == POLYGON_FILL)
        glPolygonMode(GL_FRONT, GL_FILL);
    else
        glPolygonMode(GL_FRONT, GL_LINE);
    glLineWidth(_lineWidth);

//	glColor3ubv(_color);
    if(renderType==RENDER_SHADOW)
        glColor3ub(90, 90, 90);
    else
    {
        glColor3ubv(_color);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    for( VpModel::NODES_ITOR it=_pModel->_nodes.begin(); it!=_pModel->_nodes.end(); ++it)
    {
        Node* pNode = *it;
        if (pNode != NULL)
        {
            if (renderType != RENDER_SHADOW)
            {
                if (pNode->color[0] != 0 || pNode->color[1] != 0 || pNode->color[2] != 0)
                {
                    unsigned char c[4] = { pNode->color[0], pNode->color[1], pNode->color[2], pNode->color[3] };
                    glColor4ubv(c);
                }
                else
                {
                    glColor3ubv(_color);
                }
            }
            renderVpBody(&pNode->body);
        }
    }

    if(renderType!=RENDER_SHADOW)
    {
        glDisable(GL_BLEND);
    }
}

void VpModelRenderer::renderFrame(int frame, int renderType)
{
    if (frame == -1)
    {
        GeomState state;
        getState(state);
        renderState(state, renderType);
    }
    else if (frame == max_frame + 1)
    {
        saveState();
        renderState(saved_state[frame]);
    }
    else if (frame <= max_frame)
    {
        renderState(saved_state[frame]);
    }
    else
    {
        renderState(saved_state[max_frame]);
    }
}

void VpModelRenderer::renderState(const GeomState& state, int renderType)
{
    scalar _T[16];
    scalar data[3];
    vpGeom* geom;
    char geom_type;
    Vec3 geom_color;


    for(std::vector<int>::size_type i=0; i<state.body_id.size(); i++)
    {
        geom_color = state.color[i];
        geom = state.geoms[i];

        if (renderType == RENDER_OBJECT)
            glColor3f(geom_color[0], geom_color[1], geom_color[2]);
        else if (renderType == RENDER_SHADOW)
            glColor3ub(90, 90, 90);

        glPushMatrix();
        state.frames[i].ToArray<scalar>(_T);
        glMultMatrixd(_T);
        geom->GetShape(&geom_type, data);

        switch(geom_type)
        {
        case 'B':
        case 'M':
        case 'N':
            data[0] *= SCALAR_1_2;
            data[1] *= SCALAR_1_2;
            data[2] *= SCALAR_1_2;
            _draw_box(data);
            break;
        case 'C':
        case 'D':
        case 'E':
            data[1] -= SCALAR_2 * data[0];
            _draw_capsule(data[0], data[1]);
            break;
        case 'S':
            _draw_sphere(data[0]);
            break;
        }
        glPopMatrix();
    }

}

void VpModelRenderer::getState(GeomState& state)
{
    vpBody* body;
    vpGeom* geom;
    char geom_type;
    scalar data[3];

    for(int i=0; i < _pModel->getBodyNum(); i++)
    {
        body = &(_pModel->_nodes[i]->body);
        for(int j=0; j<body->GetNumGeometry(); j++)
        {
            geom = body->GetGeometry(j);
            state.body_id.push_back(i);
            state.frames.push_back(geom->GetGlobalFrame());
            state.color.push_back(Vec3(_color[0]/255., _color[1]/255., _color[2]/255.));
            state.geoms.push_back(geom);
            // geom->GetShape(geom_type, data);
            // state.types.push_back(geom_type);
            // state.sizes.push_back(Vec3(data));
        }
    }
}

void VpModelRenderer::saveState()
{
    saved_state.push_back(GeomState());
    getState(saved_state[++max_frame]);
}

int VpModelRenderer::get_max_saved_frame()
{
    return max_frame;
}