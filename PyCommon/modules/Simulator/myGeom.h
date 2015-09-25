// +-------------------------------------------------------------------------
// | myGeom.h
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

#pragma once

#include <vector>
#include <VP/vphysics.h>

const vector<Vec3>& _getVerticesGlobal(const vpGeom* pGeom, const vector<Vec3>& verticesLcoal);

class MyBox : public vpBox
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyBox(const Vec3 &size);
	virtual const vector<Vec3>& getVerticesLocal() { return _verticesLocal; }
	virtual const vector<Vec3>& getVerticesGlobal() { return _getVerticesGlobal(this, _verticesLocal); }
};

class MyFoot1: public vpBox
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyFoot1(const Vec3 &size);
	virtual const vector<Vec3>& getVerticesLocal() { return _verticesLocal; }
	virtual const vector<Vec3>& getVerticesGlobal() { return _getVerticesGlobal(this, _verticesLocal); }
	virtual void GetShape(char *type, scalar *data)const { vpBox::GetShape(type, data); type[0] = 'M';}
};

class MyFoot2: public vpBox
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyFoot2(const Vec3 &size);
	virtual const vector<Vec3>& getVerticesLocal() { return _verticesLocal; }
	virtual const vector<Vec3>& getVerticesGlobal() { return _getVerticesGlobal(this, _verticesLocal); }
	virtual void GetShape(char *type, scalar *data) const { vpBox::GetShape(type, data); type[0] = 'M';}
};

class MyFoot3 : public vpCapsule
{
private:
	vector<Vec3> _verticesLocal;
public:
	scalar _r, _h;
	MyFoot3(scalar radius, scalar height);
	//virtual const vector<Vec3>& getVerticesLocal() const { return _verticesLocal; }
	//virtual const vector<Vec3>& getVerticesGlobal() const { return _getVerticesGlobal(this, _verticesLocal); }
	virtual const vector<Vec3>& getVerticesLocal() 
	{ 
		vector<Vec3> verticesGlobal;
		getContactVertices(_verticesLocal, verticesGlobal);
		return _verticesLocal; 
	}
	virtual const vector<Vec3>& getVerticesGlobal() 
	{
		vector<Vec3> verticesGlobal;
		getContactVertices(_verticesLocal, verticesGlobal);
		return _getVerticesGlobal(this, _verticesLocal); 
	}
	void getContactVertices(vector<Vec3>& verticesLocal, vector<Vec3>& verticesGlobal);
	void getContactVerticesGlobal(vector<Vec3>& verticesGlobal);
	virtual void GetShape(char *type, scalar *data) const { vpCapsule::GetShape(type, data); type[0] = 'C'; }
};

class MyFoot4 : public MyFoot3
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyFoot4(scalar radius, scalar height):MyFoot3(radius, height){}
	virtual const vector<Vec3>& getVerticesLocal() 
	{ 
		vector<Vec3> verticesGlobal;
		getContactVertices(_verticesLocal, verticesGlobal);
		return _verticesLocal; 
	}
	virtual const vector<Vec3>& getVerticesGlobal() 
	{
		vector<Vec3> verticesGlobal;
		getContactVertices(_verticesLocal, verticesGlobal);
		return _getVerticesGlobal(this, _verticesLocal); 
	}
	void getContactVertices(vector<Vec3>& verticesLocal, vector<Vec3>& verticesGlobal);
	void getContactVerticesGlobal(vector<Vec3>& verticesGlobal);
	virtual void GetShape(char *type, scalar *data) const { vpCapsule::GetShape(type, data); type[0] = 'C'; }
};

class MyShin: public vpBox
{
private:
	vector<Vec3> _verticesLocal;
public:
	MyShin(const Vec3 &size);
	virtual const vector<Vec3>& getVerticesLocal() { return _verticesLocal; }
	virtual const vector<Vec3>& getVerticesGlobal() { return _getVerticesGlobal(this, _verticesLocal); }
	virtual void GetShape(char *type, scalar *data) const { vpBox::GetShape(type, data); type[0] = 'Q';}
};
