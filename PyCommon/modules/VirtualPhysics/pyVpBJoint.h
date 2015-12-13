#ifndef _PYVPBJOINT_H_
#define _PYVPBJOINT_H_

#include <VP/vpDataType.h>
#include "pyVpWorld.h"
#include "pyVpBody.h"

class vpJointWrapper : public vpJoint, public wrapper<vpJoint>
{
public:
	int   GetDOF(void) const{return this->get_override("GetDOF")();}
	scalar   GetNormalForce(void) const{return this->get_override("GetNormalForce")();}
	scalar   GetNormalTorque(void) const{return this->get_override("GetNormalTorque")();}

	void   BuildKinematics(void){this->get_override("BuildKinematics")();}
	SE3   Transform(void) const{return this->get_override("Transform")();}
	void   UpdateSpringDamperTorque(void){this->get_override("UpdateSpringDamperTorque")();}
	scalar   GetPotentialEnergy(void) const{return this->get_override("GetPotentialEnergy")();}
	
	const  scalar & GetDisplacement_(int a) const{return this->get_override("GetDisplacement_")(a);}
	void   SetDisplacement_(int a, const scalar &b){this->get_override("SetDisplacement_")(a,b);}
	const  scalar & GetVelocity_(int a) const{return this->get_override("GetVelocity_")(a);}
	void   SetVelocity_(int a, const scalar &b){this->get_override("SetVelocity_")(a,b);}
	const  scalar & GetAcceleration_(int a) const{return this->get_override("GetAcceleration_")(a);}
	void   SetAcceleration_(int a, const scalar & b){this->get_override("SetAcceleration_")(a,b);}
	const  scalar & GetImpulsiveTorque_(int a) const{return this->get_override("GetImpulsiveTorque_")(a);}
	void   SetImpulsiveTorque_(int a, const scalar &b){this->get_override("SetImpulsiveTorque_")(a,b);}
	scalar   GetTorque_(int a) const{return this->get_override("GetTorque_")(a);}
	void   SetTorque_(int a, const scalar &b){this->get_override("SetTorque_")(a,b);}
	void   SetSpringDamperTorque_(int a, const scalar &b){this->get_override("SetSpringDamperTorque_")(a,b);}
	const  scalar & GetRestitution_(int a) const{return this->get_override("GetRestitution_")(a);}
	bool   ViolateUpperLimit_(int a) const{return this->get_override("ViolateUpperLimit_")(a);}
	bool   ViolateLowerLimit_(int a) const{return this->get_override("ViolateLowerLimit_")(a);}

	void   UpdateTorqueID(void){this->get_override("UpdateTorqueID")();}
	void   UpdateTorqueHD(void){this->get_override("UpdateTorqueHD")();}
	void   UpdateVelocity(const se3 &a){this->get_override("UpdateVelocity")(a);}
	void   UpdateAccelerationID(const se3 &a){this->get_override("UpdateAccelerationID")(a);}
	void   UpdateAccelerationFD(const se3 &a){this->get_override("UpdateAccelerationFD")(a);}
	void   UpdateAInertia(AInertia &a){this->get_override("UpdateAInertia")(a);}
	void   UpdateLOTP(void){this->get_override("UpdateLOTP")();}
	void   UpdateTP(void){this->get_override("UpdateTP")();}
	void   UpdateLP(void){this->get_override("UpdateLP")();}
	dse3   GetLP(void){return this->get_override("GetLP")();}
	void   ClearTP(void){this->get_override("ClearTP")();}
	
	void   IntegrateDisplacement(const scalar &a){this->get_override("IntegrateDisplacement")(a);}
	void   IntegrateVelocity(const scalar &a){this->get_override("IntegrateVelocity")(a);}
};

class pyVpBJoint : public vpBJoint
{
public:
	pyVpBJoint& 			self();

////////////////// common vpJoint functions
	/*!
		break the joint.
	*/
	void					 Break_py(void);

	/*!
		get the maximum magnitude of normal force that can break the joint.
	*/
	scalar			GetMaxNormalForce_py(void);

	/*!
		set the maximum magnitude of normal force that can break the joint.
	*/
	void					 SetMaxNormalForce_py(scalar);

	/*!
		get the maximum magnitude of normal torque that can break the joint.
	*/
	scalar			GetMaxNormalTorque_py(void);

	/*!
		set the maximum magnitude of normal torque that can break the joint.
	*/
	void					 SetMaxNormalTorque_py(scalar);

	/*!
		set whether hybrid dynamics respects acceleration or torque
	*/
	void					 SetHybridDynamicsType_py(std::string);
	std::string				 GetHybridDynamicsType_py(void);

//////////////////// vpBJoint functions

	/*!
		set a joint angle of the idx-th joint variable.
	*/
	void					 SetOrientation_py(object &);

	/*!
		set a angular velocity of the joint frame which is represented in a local frame.
	*/
	void					 SetVelocity_py(object &);

	/*!
		set a angular acceleration of the joint frame which is represented in a local frame.
	*/
	void					 SetAcceleration_py(object &);

	/*!
		set an initial orientation of the joint frame.
	*/
	void					 SetInitialOrientation_py(object &);

	/*!
		set an elasticity of the joint frame.
	*/
	void					 SetElasticity_py(scalar);

	/*!
		set a damping parameter of the joint frame.
	*/
	void					 SetDamping_py(scalar);

	/*!
		set a torque applied to the joint which is represented in a local frame.
	*/
	void					 SetTorque_py(object &);
	void					 AddTorque_py(object &);

	/*!
		get an orientation of the joint frame.
	*/
	object						 GetOrientation_py(void);

	/*!
		get a angular velocity of the joint frame which is represented in a local frame.
	*/
	object					 GetVelocity_py(void);

	object					 GetAcceleration_py(void);

	/*!
		get a torque applied to the joint which is represented in a local frame.
	*/
	object					 GetTorque_py(void);

	int				 GetDOF_py(void);
	scalar			 GetNormalForce_py(void);
	scalar			 GetNormalTorque_py(void);

	std::string			 streamOut_py(void);
};





#endif // _PYVPBJOINT_H_