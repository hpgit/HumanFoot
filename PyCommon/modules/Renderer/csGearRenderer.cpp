#pragma once

#include "stdafx.h"

#ifdef WIN32
#include <windows.h> 
#endif
#include <GL/gl.h>
#include <GL/glu.h>
 
#include "../../externalLibs/common/boostPythonUtil.h"
#include <gear/gear.h>

#include "csGearRenderer.h"
#include "../Simulator/csGearModel.h"

static double _T[16];
#define _SLICE_SIZE		24


BOOST_PYTHON_MODULE(csGearRenderer)
{
	class_<GearModelRenderer>("GearModelRenderer", init<GearModel*, const tuple&, optional<int, double> >())
		.def(init<GearControlModel*, const tuple&, optional<int, double> >())
		.def("render", &GearModelRenderer::render)
		;
	scope().attr("POLYGON_FILL") = POLYGON_FILL;
	scope().attr("POLYGON_LINE") = POLYGON_LINE;
}

void _draw_box(const double _sz[3])
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

void _draw_sphere(const double &rad)
{
	glBegin(GL_QUAD_STRIP);
	for ( int i = 0; i < _SLICE_SIZE - 1; i++ )
	{
		double t = (double)i / (double)(_SLICE_SIZE - 1);
		double t2 = (double)(i + 1) / (double)(_SLICE_SIZE - 1);
		double cp = cos((double)M_PI * t - (double)M_PI*2);
		double sp = sin((double)M_PI * t - (double)M_PI*2);
		double cp2 = cos((double)M_PI * t2 - (double)M_PI*2);
		double sp2 = sin((double)M_PI * t2 - (double)M_PI*2);

		for ( int j = 0; j < _SLICE_SIZE; j++ )
		{
			double s = (double)j / (double)(_SLICE_SIZE - 1);
			double ct = cos(M_PI*2 * s);
			double st = sin(M_PI*2 * s);

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

void _draw_capsule(const double &rad, const double &height)
{
	int i, j;
	double ct_i, st_i, ct_im1 = 1., st_im1 = 0., cp_i, sp_i, cp_im1, sp_im1;

	glBegin(GL_QUADS);
	for ( i = 1; i < _SLICE_SIZE + 1; i++ )
	{
		ct_i = cos(M_PI*2 * (double)i / (double)_SLICE_SIZE);
		st_i = sin(M_PI*2 * (double)i / (double)_SLICE_SIZE);

		glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)0.);
		glNormal3d(ct_im1, st_im1, 0.);
		glVertex3d(rad * ct_im1, rad * st_im1, -0.5 * height);
		glTexCoord2d((double)i / (double)_SLICE_SIZE, 0.);
		glNormal3d(ct_i, st_i, 0.);
		glVertex3d(rad * ct_i, rad * st_i, -0.5 * height);
		glTexCoord2d((double)i / (double)_SLICE_SIZE, 1.);
		glNormal3d(ct_i, st_i, 0.);
		glVertex3d(rad * ct_i, rad * st_i, 0.5 * height);
		glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, 1.0);
		glNormal3d(ct_im1, st_im1, 0.);
		glVertex3d(rad * ct_im1, rad * st_im1, 0.5 * height);

		cp_im1 = 1.;
		sp_im1 = 0.;
		for ( j = 1; j < _SLICE_SIZE + 1; j++ )
		{
			cp_i = cos(0.5 * (double)M_PI * (double)j / (double)_SLICE_SIZE);
			sp_i = sin(0.5 * (double)M_PI * (double)j / (double)_SLICE_SIZE);

			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, 1.0 - (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  sp_im1);
			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, rad * sp_im1 + 0.5 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, 1.0 - (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  sp_im1);
			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, rad * sp_im1 + 0.5 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, 1.0 - (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_i, st_i * cp_i,  sp_i);
			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, rad * sp_i + 0.5 * height);
			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, 1.0 - (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  sp_i);
			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, rad * sp_i + 0.5 * height);

			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  -sp_im1);
			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, -rad * sp_im1 - 0.5 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  -sp_im1);
			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, -rad * sp_im1 - 0.5 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_i, st_i * cp_i,  -sp_i);
			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, -rad * sp_i - 0.5 * height);
			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  -sp_i);
			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, -rad * sp_i - 0.5 * height);

			cp_im1 = cp_i;
			sp_im1 = sp_i;
		}

		ct_im1 = ct_i;
		st_im1 = st_i;
	}
	glEnd();
}


void renderGearBody(const HpGBody* pBody)
{
	glPushMatrix();
	SE3 bodyFrame= ((GBody*)pBody)->getPoseGlobal();
	//bodyFrame.ToArray(_T);
	//glMultMatrixd(_T);
	glMultMatrixd(bodyFrame.GetArray());
	

	//const vpGeom *pGeom;
	char type;
	double data[3];
	//for ( int j = 0; j < pBody->GetNumGeometry(); j++ )
	{
		//pGeom = pBody->GetGeometry(j);
		glPushMatrix();
		//pBody->GetLocalFrame().ToArray(_T);
		glMultMatrixd(pBody->getGeomTransform().GetArray());

		pBody->getShape(&type, data);
		//std::cout << data[0]<<" " << data[1]<< " " << data[2] <<std::endl;
		switch ( type )
		{
		case 'B':
			data[0] *= 0.5;
			data[1] *= 0.5;
			data[2] *= 0.5;
			_draw_box(data);
			break;
		case 'C':
			data[1] -= 2 * data[0];
			_draw_capsule(data[0], data[1]);
			break;
		case 'S':
			_draw_sphere(data[0]);
			break;
		}
		glPopMatrix();
	}
	glPopMatrix();
}

GearModelRenderer::GearModelRenderer( GearModel* pModel, const tuple& color, int polygonStyle, double lineWidth)
{
	_pModel = pModel;

	_color[0] = (GLubyte)XI(color[0]);
	_color[1] = (GLubyte)XI(color[1]);
	_color[2] = (GLubyte)XI(color[2]);

	_polygonStyle = polygonStyle;
	_lineWidth = lineWidth;
}
void GearModelRenderer::render()
{
	if(_polygonStyle == POLYGON_FILL)
		glPolygonMode(GL_FRONT, GL_FILL);
	else
		glPolygonMode(GL_FRONT, GL_LINE);
	glLineWidth(_lineWidth);

	glColor3ubv(_color);

	for( GearModel::NODES_ITOR it=_pModel->_nodes.begin(); it!=_pModel->_nodes.end(); ++it)
	{
		GearModel::Node* pNode = *it;
		if(pNode != NULL)
			renderGearBody((HpGBody*)&pNode->body);
	}
}
