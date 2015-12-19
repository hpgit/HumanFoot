#ifndef _PYVPGEOM_H_
#define _PYVPGEOM_H_

#include <VP/vpDataType.h>

class pyVpGeom : public vpGeom, public bp::wrapper<vpGeom>{
public:

	/*!
		get a coordinate frame of the geometry w.r.t a body frame.
	*/
	// const SE3				&GetLocalFrame(void) const;

	/*!
		get a coordinate frame of the geometry w.r.t a global frame.
	*/
	// const SE3				&GetGlobalFrame(void) const;

	/*!
		get an inertia of the geometry.
	*/
	virtual Inertia			 GetInertia(scalar d) const 
	{return this->get_override("GetInertia")(d);}

	/*!
		get a radius of a bounding sphere.
	*/
	virtual	scalar			 GetBoundingSphereRadius(void) const 
	{return this->get_override("GetBoundingSphereRadius")();}
	
	/*!
		get a shape information.
	*/
	virtual void			 GetShape(char *type, scalar *data) const
	{this->get_override("GetShape")(type, data);}

	//void					 UpdateGlobalFrame(void);
	virtual bool			 DetectCollision(const vpGeom *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpGeom" <<std::endl;}
	virtual bool			 DetectCollision(const vpBox *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpBox" <<std::endl;}
	virtual bool			 DetectCollision(const vpSphere *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpSphere" <<std::endl;}
	virtual bool			 DetectCollision(const vpCapsule *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpCapsule" <<std::endl;}
	virtual bool			 DetectCollision(const vpPlane *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpPlane" <<std::endl;}
	virtual bool			 DetectCollision(const vpCylinder *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpCylinder" <<std::endl;}
	virtual bool			 DetectCollision(const vpTorus *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpTorus" <<std::endl;}

	//ys
	//virtual const vector<Vec3>& getVerticesLocal() { vector<Vec3> v; return v;}
	//virtual const vector<Vec3>& getVerticesGlobal(){ vector<Vec3> v; return v;}

	//virtual void draw(void) const {}

};

class pyVpBox : public vpBox, public bp::wrapper<vpBox> {
public:
	virtual Inertia GetInertia(scalar d) const
	{
		if (bp::override py_override = this->get_override("GetInertia"))
		{
			return py_override(d);
		}
		return vpBox::GetInertia(d);
	}

	virtual Inertia default_GetInertia(scalar d) const
	{
		return vpBox::GetInertia(d);
	}

	virtual scalar GetBoundingSphereRadius() const
	{
		if(bp::override py_override = this->get_override("GetBoundingSphereRadius"))
		{
			return py_override();
		}
		return vpBox::GetBoundingSphereRadius();
	}
	virtual scalar default_GetBoundingSphereRadius() const
	{
		return vpBox::GetBoundingSphereRadius();
	}

	virtual void GetShape(char *type, scalar *data) const
	{
		if(bp::override py_override = this->get_override("GetShape"))
		{
			py_override(type, data);
		}
		vpBox::GetShape(type, data);
	}
	virtual void default_GetShape(char *type, scalar *data) const
	{
		vpBox::GetShape(type, data);
	}


	object GetShape_py();
	void SetSize_py(object pvV);
};

#endif // _PYVPGEOM_H_