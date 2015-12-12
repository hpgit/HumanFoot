#ifndef _PYVPBJOINT_H_
#define _PYVPBJOINT_H_

#include <VP/vpDataType.h>
#include "pyVpWorld.h"
#include "pyVpBody.h"

class pyVpBJoint : public vpJoint
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