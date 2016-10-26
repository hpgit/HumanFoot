// +-------------------------------------------------------------------------
// | myGeom.cpp
// |
// | Author: Yoonsang Lee
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Yoonsang Lee 2013
// |    See the included COPYRIGHT.txt file for further details.
// |    
// |    This file is part of the DataDrivenBipedController.
// |    DataDrivenBipedController is free software: you can redistribute it and/or modify
// |    it under the terms of the MIT License.
// |
// |    You should have received a copy of the MIT License
// |    along with DataDrivenBipedController.  If not, see <mit-license.org>.
// +-------------------------------------------------------------------------

#include "stdafx.h"

#include "myGeom.h"

#define MAX_X 1	// 0001
#define MAX_Y 2	// 0010
#define MAX_Z 4	// 0100

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

const vector<Vec3>& _getVerticesGlobal( const vpGeom* pGeom, const vector<Vec3>& verticesLocal )
{
	static vector<Vec3> verticesGlobal;

	verticesGlobal.resize(verticesLocal.size());

	for(int i=0; i<verticesLocal.size(); ++i)
		verticesGlobal[i] = pGeom->GetGlobalFrame() * verticesLocal[i];

	return verticesGlobal;
}
const vector<Vec3>& _getVerticesLocal(const vpGeom* pGeom, const vector<Vec3>& verticesGlobal)
{
	static vector<Vec3> verticesLocal;

	//verticesLocal.resize(verticesLocal.size());

	for (int i = 0; i<verticesLocal.size(); ++i)
		verticesLocal.push_back( Inv(pGeom->GetGlobalFrame()) * verticesGlobal[i]);

	return verticesLocal;
}

MyBox::MyBox( const Vec3 &size ):vpBox(size)
{
	char type;
	scalar data[3];

	_verticesLocal.resize(8);
	this->GetShape(&type, data);

	for( int p=0; p<_verticesLocal.size(); ++p)
	{
		_verticesLocal[p][0] = (p & MAX_X) ? data[0]/2. : -data[0]/2.;
		_verticesLocal[p][1] = (p & MAX_Y) ? data[1]/2. : -data[1]/2.;
		_verticesLocal[p][2] = (p & MAX_Z) ? data[2]/2. : -data[2]/2.;
	}
}

void sphere(vector<Vec3>& verticesLocal,const Vec3& center, const scalar &rad, int row, int col, scalar height_ratio=1.)
{
	int col_real;
	for ( int i = 0; i < int(row * height_ratio); i++ )
	{
		scalar t = (scalar)i / (scalar)(row);
		//scalar t2 = (scalar)(i + 1) / (scalar)(row);
		scalar cp = cos((scalar)M_PI * t - (scalar)M_PI_2);
		scalar sp = sin((scalar)M_PI * t - (scalar)M_PI_2);
		//scalar cp2 = cos((scalar)M_PI * t2 - (scalar)M_PI_2);
		//scalar sp2 = sin((scalar)M_PI * t2 - (scalar)M_PI_2);

		col_real = i==0 ? 1 : col;
		for ( int j = 0; j < col_real; j++ )
		{
			scalar s = (scalar)j / (scalar)(col_real);
			scalar ct = cos(M_2PI * s);
			scalar st = sin(M_2PI * s);

			verticesLocal.push_back(Vec3(rad * cp * ct, rad * sp,  -rad * cp * st) + center);
		}
	}
}

void sphere2(vector<Vec3>& verticesLocal,const Vec3& center, const scalar &rad, int row, int col, scalar start_height_ratio, scalar end_height_ratio, scalar x_ratio, scalar y_ratio, scalar z_ratio)
{
	int col_real;
	for ( int i = int(row * start_height_ratio); i < int(row * end_height_ratio); i++ )
	{
		scalar t = (scalar)i / (scalar)(row);
		//scalar t2 = (scalar)(i + 1) / (scalar)(row);
		scalar cp = cos((scalar)M_PI * t - (scalar)M_PI_2);
		scalar sp = sin((scalar)M_PI * t - (scalar)M_PI_2);
		//scalar cp2 = cos((scalar)M_PI * t2 - (scalar)M_PI_2);
		//scalar sp2 = sin((scalar)M_PI * t2 - (scalar)M_PI_2);

		col_real = i==0 ? 1 : col;
		for ( int j = 0; j < col_real; j++ )
		{
			scalar s = (scalar)j / (scalar)(col_real);
			scalar ct = cos(M_2PI * s);
			scalar st = sin(M_2PI * s);

			verticesLocal.push_back(Vec3(rad * cp * ct * x_ratio, rad * sp * y_ratio,  -rad * cp * st* z_ratio) + center);
		}
	}
}

MyFoot1::MyFoot1( const Vec3 &size ):vpBox(size)
{
	_verticesLocal.clear();

	// top, bottom, left, right
	scalar t = size[2]/2.; scalar b = -size[2]/2.;
	scalar l = -size[0]/2.; scalar r = size[0]/2.;
	scalar l0 = -size[1]/2.;

	// 4 spheres
	scalar rad = .05, gap = 0.01;
	int row = 6, col = 6;
	scalar height_ratio = .5;
	sphere(_verticesLocal, Vec3(l+gap, l0+rad, t-gap), rad, row, col, height_ratio);
	sphere(_verticesLocal, Vec3(l+gap, l0+rad, b+gap), rad, row, col, height_ratio);
	sphere(_verticesLocal, Vec3(r-gap, l0+rad, t-gap), rad, row, col, height_ratio);
	sphere(_verticesLocal, Vec3(r-gap, l0+rad, b+gap), rad, row, col, height_ratio);

	sphere(_verticesLocal, Vec3(l+gap, l0+rad, 0), rad, row, col, height_ratio);
	sphere(_verticesLocal, Vec3(r-gap, l0+rad, 0), rad, row, col, height_ratio);
}

MyFoot2::MyFoot2( const Vec3 &size ):vpBox(size)
{
	_verticesLocal.clear();

	// top, bottom, left, right
	scalar t = size[2]/2.; scalar b = -size[2]/2.;
	scalar l = -size[0]/2.; scalar r = size[0]/2.;
	scalar l0 = -size[1]/2.;

	scalar rad = .1; scalar gap = .02; scalar bgap = .0; scalar tgap = -.02;
	int row = 12, col = 6;
//	sphere2(_verticesLocal, Vec3(0,l0,t-.5*rad), rad, row, col, .5, .6, 1,1,1);
//	sphere2(_verticesLocal, Vec3(0,l0,b+rad), rad, row, col, .5, .6, 1,1,1);

	sphere2(_verticesLocal, Vec3(l+gap,l0+rad,t-gap-tgap), rad, row, col, .1, .25, 1,1,1);
	sphere2(_verticesLocal, Vec3(l+gap,l0+rad,b+gap+bgap), rad, row, col, .1, .25, 1,1,1);
	sphere2(_verticesLocal, Vec3(r-gap,l0+rad,t-gap-tgap), rad, row, col, .1, .25, 1,1,1);
	sphere2(_verticesLocal, Vec3(r-gap,l0+rad,b+gap+bgap), rad, row, col, .1, .25, 1,1,1);
}

MyFoot3::MyFoot3(scalar radius, scalar height) :vpCapsule(radius, height)
{
	_verticesLocal.clear();
	
	// top, bottom, left, right
	//scalar l = height / 2. - radius; scalar r = -height / 2. + radius;
	_r = radius;
	_h = height;
	//int row = 12, col = 6;

	//sphere(_verticesLocal, Vec3(0.0, 0.0, l), radius, row, col);
	//sphere(_verticesLocal, Vec3(0.0, 0.0, r), radius, row, col);

}

void MyFoot3::getContactVertices(vector<Vec3>& verticesLocal, vector<Vec3>& verticesGlobal)
{
	getContactVerticesGlobal(verticesGlobal);
	verticesLocal.clear();
	SE3 globalFrame = this->GetGlobalFrame();
	for(int i=0; i<verticesGlobal.size(); i++)
        verticesLocal.push_back(Inv(globalFrame)*verticesGlobal[i]);
}
void MyFoot3::getContactVerticesGlobal(vector<Vec3>& verticesGlobal)
{
	int contact_segment = 0;
	scalar l = _h / 2. - _r; scalar r = -_h / 2. + _r;

	Vec3 center[2];
	//scalar depth[2];
	center[0] = this->GetGlobalFrame()* Vec3(0, 0, l);
	center[1] = this->GetGlobalFrame()* Vec3(0, 0, r);
	verticesGlobal.clear();
	verticesGlobal.push_back(center[0] + Vec3(0, -_r, 0));
	verticesGlobal.push_back(center[1] + Vec3(0, -_r, 0));

	double r_contact = 0.0;
	Vec3 contactPoint(0., 0., 0.);
	if(contact_segment != 0)
	{
        for(int j=0; j<2; j++)
        {
            if(center[j][1] -_r < 0.0)
            {
                r_contact = sqrt( _r * _r - center[j][1]*center[j][1]);
                for (int i=0; i<contact_segment; i++)
                {
                    contactPoint[0] = center[j][0] + r_contact * cos(double(i)/contact_segment * 2 * M_PI);
                    contactPoint[2] = center[j][2] + r_contact * sin(double(i)/contact_segment * 2 * M_PI);
                    verticesGlobal.push_back(contactPoint);
                }
            }
        }
	}
}

void MyFoot4::getContactVertices(vector<Vec3>& verticesLocal, vector<Vec3>& verticesGlobal)
{
	getContactVerticesGlobal(verticesGlobal);
	verticesLocal.clear();
	SE3 globalFrame = this->GetGlobalFrame();
	for(int i=0; i<verticesGlobal.size(); i++)
        verticesLocal.push_back(Inv(globalFrame)*verticesGlobal[i]);
}
void MyFoot4::getContactVerticesGlobal(vector<Vec3>& verticesGlobal)
{
    int contact_segment = 0;
	scalar l = _h / 2. - _r; //scalar r = -_h / 2. + _r;
	Vec3 center[2];
	//scalar depth[2];
	center[0] = this->GetGlobalFrame()* Vec3(0, 0, l);
	verticesGlobal.clear();
	verticesGlobal.push_back(center[0] + Vec3(0, -_r, 0));

	double r_contact = 0.0;
	Vec3 contactPoint(0., 0., 0.);
	if(contact_segment != 0)
	{
        for(int j=0; j<1; j++)
        {
            if(center[j][1] -_r < 0.0)
            {
                r_contact = sqrt( _r * _r - center[j][1]*center[j][1]);
                for (int i=0; i<contact_segment; i++)
                {
                    contactPoint[0] = center[j][0] + r_contact * cos(double(i)/contact_segment * 2 * M_PI);
                    contactPoint[2] = center[j][2] + r_contact * sin(double(i)/contact_segment * 2 * M_PI);
                    verticesGlobal.push_back(contactPoint);
                }
            }
        }
	}
}

void MyFoot5::getContactVertices(vector<Vec3>& verticesLocal, vector<Vec3>& verticesGlobal)
{
	verticesLocal.clear();
}

void MyFoot5::getContactVerticesGlobal(vector<Vec3>& verticesGlobal)
{
	verticesGlobal.clear();
}

//MyShin::MyShin( const Vec3 &size ):vpBox(size)
//{
//	char type;
//	scalar data[3];
//
//	_verticesLocal.resize(8);
//	this->GetShape(&type, data);
//
//	for( int p=0; p<_verticesLocal.size(); ++p)
//	{
//		_verticesLocal[p][0] = (p & MAX_X) ? data[0]/2. : -data[0]/2.;
//		_verticesLocal[p][1] = (p & MAX_Y) ? data[1]/2. : -data[1]/2.;
//		_verticesLocal[p][2] = (p & MAX_Z) ? data[2]/2. : -data[2]/2.;
//	}
//}
//

