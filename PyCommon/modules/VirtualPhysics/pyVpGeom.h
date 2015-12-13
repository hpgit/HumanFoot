#ifndef _PYVPGEOM_H_
#define _PYVPGEOM_H_

#include <VP/vpDataType.h>

{
public:

	/*!
		get a coordinate frame of the geometry w.r.t a body frame.
	*/
	const SE3				&GetLocalFrame(void) const;

	/*!
		get a coordinate frame of the geometry w.r.t a global frame.
	*/
	const SE3				&GetGlobalFrame(void) const;

	/*!
		get an inertia of the geometry.
	*/
	virtual Inertia			 GetInertia(scalar) const = 0;

	/*!
		get a radius of a bounding sphere.
	*/
	virtual	scalar			 GetBoundingSphereRadius(void) const = 0;
	
	/*!
		get a shape information.
	*/
	virtual void			 GetShape(char *, scalar *) const = 0;

	void					 UpdateGlobalFrame(void);
	virtual bool			 DetectCollision(const vpGeom *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpBox *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpSphere *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpCapsule *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpPlane *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpCylinder *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpTorus *, vpCollisionInfoArray &) const = 0;

	//ys
	virtual const vector<Vec3>& getVerticesLocal() { vector<Vec3> v; return v;}
	virtual const vector<Vec3>& getVerticesGlobal(){ vector<Vec3> v; return v;}

	virtual void draw(void) const {}

};

#endif // _PYVPGEOM_H_