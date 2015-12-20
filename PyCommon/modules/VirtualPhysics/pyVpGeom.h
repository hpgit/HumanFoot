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
	virtual bool			 DetectCollision(const vpGeom *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpGeom" <<std::endl; return false;}
	virtual bool			 DetectCollision(const vpBox *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpBox" <<std::endl; return false;}
	virtual bool			 DetectCollision(const vpSphere *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpSphere" <<std::endl; return false;}
	virtual bool			 DetectCollision(const vpCapsule *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpCapsule" <<std::endl; return false;}
	virtual bool			 DetectCollision(const vpPlane *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpPlane" <<std::endl; return false;}
	virtual bool			 DetectCollision(const vpCylinder *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpCylinder" <<std::endl; return false;}
	virtual bool			 DetectCollision(const vpTorus *a, vpCollisionInfoArray &b) const {std::cout << "DetectCollision vpTorus" <<std::endl; return false;}

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


	object GetLocalFrame_py(void);

	/*!
		get a coordinate frame of the geometry w.r.t a global frame.
	*/
	object GetGlobalFrame_py(void);
	void UpdateGlobalFrame_py(void);

	void SetSize_py(object &pvV);

	bp::list GetShape_py();
	object GetInertia_py(scalar);

	object GetHalfSize_py(void);
	object GetSize_py(void);
};


class pyVpCapsule : public vpCapsule, public bp::wrapper<vpCapsule>
{
public:
	/*!
		default radius and height of the capsule is SCALAR_1_2 and 1.5, respectively.
	*/

	virtual Inertia GetInertia(scalar d) const
	{
		if (bp::override py_override = this->get_override("GetInertia"))
		{
			return py_override(d);
		}
		return vpCapsule::GetInertia(d);
	}

	virtual Inertia default_GetInertia(scalar d) const
	{
		return vpCapsule::GetInertia(d);
	}

	virtual scalar GetBoundingSphereRadius() const
	{
		if(bp::override py_override = this->get_override("GetBoundingSphereRadius"))
		{
			return py_override();
		}
		return vpCapsule::GetBoundingSphereRadius();
	}
	virtual scalar default_GetBoundingSphereRadius() const
	{
		return vpCapsule::GetBoundingSphereRadius();
	}

	virtual void GetShape(char *type, scalar *data) const
	{
		if(bp::override py_override = this->get_override("GetShape"))
		{
			py_override(type, data);
		}
		vpCapsule::GetShape(type, data);
	}
	virtual void default_GetShape(char *type, scalar *data) const
	{
		vpCapsule::GetShape(type, data);
	}


	/*!
		get a shape information.
		return type = 'C', data[0] = radius, data[1] = height
	*/
	bp::list GetShape_py();
	object GetInertia_py(scalar);
};

class pyVpSphere : public vpSphere, public bp::wrapper<vpSphere>
{
public:
	/*!
		default radius and height of the capsule is SCALAR_1_2 and 1.5, respectively.
	*/

	virtual Inertia GetInertia(scalar d) const
	{
		if (bp::override py_override = this->get_override("GetInertia"))
		{
			return py_override(d);
		}
		return vpSphere::GetInertia(d);
	}

	virtual Inertia default_GetInertia(scalar d) const
	{
		return vpSphere::GetInertia(d);
	}

	virtual scalar GetBoundingSphereRadius() const
	{
		if(bp::override py_override = this->get_override("GetBoundingSphereRadius"))
		{
			return py_override();
		}
		return vpSphere::GetBoundingSphereRadius();
	}
	virtual scalar default_GetBoundingSphereRadius() const
	{
		return vpSphere::GetBoundingSphereRadius();
	}

	virtual void GetShape(char *type, scalar *data) const
	{
		if(bp::override py_override = this->get_override("GetShape"))
		{
			py_override(type, data);
		}
		vpSphere::GetShape(type, data);
	}
	virtual void default_GetShape(char *type, scalar *data) const
	{
		vpSphere::GetShape(type, data);
	}


	/*!
		get a shape information.
		return type = 'S', data[0] = radius
	*/
	bp::list GetShape_py();
	object GetInertia_py(scalar);
};

#endif // _PYVPGEOM_H_