#ifndef _PYVPJOINT_H_
#define _PYVPJOINT_H_

#include <VP/vpDataType.h>
#include "pyVpWorld.h"
#include "pyVpBody.h"

class pyVpJoint : public vpJoint, public wrapper<vpJoint>
{
public:
	virtual int   GetDOF(void) const{return this->get_override("GetDOF")();}
	virtual scalar   GetNormalForce(void) const{return this->get_override("GetNormalForce")();}
	virtual scalar   GetNormalTorque(void) const{return this->get_override("GetNormalTorque")();}

	void   BuildKinematics(void){std::cout << "BuildKinematics"<<std::endl;}
	SE3   Transform(void) const{std::cout << "Transform" << std::endl; return SE3();}
	void   UpdateSpringDamperTorque(void){std::cout << "UpdateSpringDamperTorque" <<std::endl;}
	scalar   GetPotentialEnergy(void) const{std::cout << "GetPotentialEnergy" <<std::endl; return 0;}
	
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

class pyVpBJoint : public vpBJoint, public wrapper<vpBJoint>
{
public:
	pyVpBJoint& 			self();

	virtual int   GetDOF(void) const;
	int GetDOF_default(void) const;
	virtual scalar   GetNormalForce(void) const;
	virtual scalar   GetNormalForce_default(void) const;
	virtual scalar   GetNormalTorque(void) const;
	virtual scalar   GetNormalTorque_default(void) const;
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





#endif // _PYVPJOINT_H_