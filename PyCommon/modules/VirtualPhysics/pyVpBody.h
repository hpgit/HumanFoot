#ifndef _PYVPBODY_H_
#define _PYVPBODY_H_

#include <VP/vpDataType.h>
#include "pyVpWorld.h"
#include "pyVpJoint.h"

class pyVpWorld;
class pyVpBJoint;

class pyVpBody : public vpBody, public wrapper<vpBody>
{


public:

	pyVpBody&               self();
	/*!
		set up a joint to the body.
		\param J a joint that you want to connect to the body
		\param T transformation of the joint frame represented in a body frame.
	*/
	//needed to implemented by all joint type
	void					 SetJoint_py(vpJoint *J, const object &T = object());

	/*!
		apply a force to the body.
		The force will accumulated during the simulation time period.
		After each simulation step, all the applied forces will be set to be zero.
		If you do not want accumulating the force, then you should call ResetForce_py() explicitly, before calling Apply*Force_py().
		\param F a force that you want to apply to the body. F is represented in a global frame
		\param p an appyling position of the force represented in a body frame.
	*/
	void					 ApplyGlobalForce_py(object &pyF, object &pyP);

	/*!
		apply a force to the body.
		\param F a force that you want to apply to the body. F is represented in a body frame
		\param p an appyling position of the force represented in a body frame.
	*/
	void					 ApplyLocalForce_py(object &F, const object &p=object());

	/*!
		release the force from the body.
	*/
	void					 ResetForce_py(void);

	/*!
		set an inertia tensor tothe body.
		Evenif you do not choose an inertia for the body,
		the inertia will be generated automatically from the geometries consisting the body.
		However you can override or ignore the generated inertia using this method.
	*/

	void					 SetInertia_py(object &);

	/*!
		get an inertia tensor of the body.
	*/
	object 					GetInertia_py(void);

	/*!
		set a transformation of the joint frame.
		\param J the joint should be set to the body previously using SetJoint_py() method.
	*/
	void					 SetJointFrame_py(vpJoint *J, const object &pyT);

	/*!
		get a transformation of the joint frame.
	*/
	object				GetJointFrame_py(vpJoint *);

	/*!
		set a transformation of the body frame w.r.t a global frame.

		\sa vpWorld::SetGlobalFrame
	*/
	void					 SetFrame_py(object &pySE3);

	/*!
		get a transformation of the body frame w.r.t a global frame.
	*/
	object				    GetFrame_py(void);

	/*!
		set a linear velocity of the body.
		The velocity is reprenseted in a global frame.
	*/
	void					 SetVelocity_py(object &);

	/*!
		set an angular velocity of the body.
		The velocity is reprenseted in a global frame.
	*/
	void					 SetAngularVelocity_py(object &);

	/*!
		set a generalized velocity of the body.
		The velocity is reprenseted in a global frame.
		Note that it depends on the current frame. Hence it should be called before SetFrame_py().
	*/
	void					 SetGenVelocity_py(object &);

	/*!
		set a generalized velocity of the body.
		The velocity is reprenseted in a local frame.
	*/
	void					 SetGenVelocityLocal_py(object &);

	/*!
		set a generalized acceleration of the body.
		The acceleration is reprenseted in a global frame.
		Note that it depends on the current frame. Hence it should be called before SetFrame_py().
	*/
	void					 SetGenAcceleration_py(object &);

	/*!
		set a generalized acceleration of the body.
		The acceleration is reprenseted in a local frame.
	*/
	void					 SetGenAccelerationLocal_py(object &);

	/*!
		get a generalized velocity of the body.
		The velocity is reprenseted in a global frame.
	*/
	object						 GetGenVelocity_py(void);

	/*!
		get a generalized velocity of the body.
		The velocity is reprenseted in a local frame.
	*/
	object				GetGenVelocityLocal_py(void);

	/*!
		get a linear velocity of a given point in the body.
		The velocity is reprenseted in a global frame.
		\param p position of the point in the body. It is represented in the body frame.
	*/
	object					 GetLinVelocity_py(object &p);

	/*!
		get an angular velocity of the body.
		The velocity is reprenseted in a global frame.
	*/
	object					 GetAngVelocity_py(void);

	/*!
		get a generalized acceleration of the body.
		The velocity is reprenseted in a global frame.
	*/
	object						 GetGenAcceleration_py(void);

	/*!
		get a generalized acceleration of the body.
		The velocity is reprenseted in a local frame.
	*/
	object				GetGenAccelerationLocal_py(void);

	/*!
		query whether the body can collide with other bodies.
	*/
	bool					 IsCollidable_py(void);

	/*!
		query whether the body is desclared as a ground.
	*/
	bool					 IsGround_py(void);

	/*!
		set a collidability of the body.
	*/
	void					 SetCollidable_py(bool);

	/*!
		Add a primitive geometry to the body
		\param pGeom a pointer to a primitive geometry
		\param T a location of the geometry represented in the body frame.
	*/
	void					 AddGeometry_py(vpGeom *pGeom, const object &T = object());

	/*!
		get a radius of a bounding sphere including the body, where the center is located at the center of body frame
	*/
	scalar					 GetBoundingSphereRadius_py(void);

	/*!
		set a material for the body.
		If you do not set up a material,
		default material properties will be applied.
		\sa vpMaterial::GetDefaultMaterial_py()
	*/
	void					 SetMaterial_py(const vpMaterial *);

	/*!
		get a meterial applied to the body.
		\sa vpBody::SetMaterial_py()
	*/
	pyVpMaterial		&GetMaterial_py(void);

	/*!
		get a center of mass
	*/
	object				GetCenterOfMass_py(void);

	/*!
		generate a display list
	*/
	//void					 GenerateDisplayList_py(bool);

	/*!
		get a sum of all forces applied to the body including gravity
	*/
	object					 GetForce_py(void);

	/*!
		get a sum of all forces applied to the body excluding gravity
	*/
	object 					GetNetForce_py(void);


	/*!
		get a force applied to the body due to the gravity
	*/
	object					 GetGravityForce_py(void);

	/*!
		return whether the inertia of the body is assigend by user
		\sa vpBody::SetInertia
	*/
	bool					 IsSetInertia_py(void);

	/*!
		get a number of geometries attached to the body
		\sa vpBody::GetGeometry_py()
	*/
	int						 GetNumGeometry_py(void) ;

	/*!
		get a pointer to the ith geometry
	*/
	//boost::shared_ptr<vpGeom>			GetGeometry_py(int);
	vpGeom&			GetGeometry_py(int);

	/*!
		get a unique identifying integer value which is assigned by VirtualPhysics
	*/
	int						 GetID_py(void) ;

	/*!
		set the body as a ground. Bodies set as a ground don't move under any external forces.
	*/
	void					 SetGround_py(bool = true);

	/*!
		Apply gravity for the body.
		\sa vpWorld::SetGravity
	*/
	//void					 ApplyGravity_py(bool flag = true);

	/*!
		return wheter the gravity is applied to the body
		\sa vpBody::ApplyGravity
	*/
	//bool					 IsApplyingGravity_py(void);

	/*!
		return the world including with the body
		\sa vpWorld::AddBody
	*/
	pyVpWorld&					GetWorld_py(void);

	/*!
		return whether the body is collided with pBody approximated with bounding sphere
	*/
	bool					 DetectCollisionApprox_py(object &Body);

	pyVpSystem&				GetSystem_py(void);

	void					 SetHybridDynamicsType_py(std::string);
	std::string				 GetHybridDynamicsType_py(void);

	void					 BackupState_py(void);
	void					 RollbackState_py(void);

	void					 UpdateGeomFrame_py(void);

};

#endif // _PYVPBODY_H_
