#include "stdafx.h"

#include "pyVpBJoint.h"
#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"
#include <VP/vpDataType.h>

#include <sstream>

#define make_tuple boost::python::make_tuple

BOOST_PYTHON_MODULE(vpBJoint)
{
    numeric::array::set_module_and_type("numpy", "ndarray");

    class_<pyVpBJoint>("vpBJoint", init<>())
        .def("self", &pyVpBJoint::self, return_value_policy<reference_existing_object>())
        .def("Break", &pyVpBJoint::Break_py)
		.def("GetMaxNormalForce", &pyVpBJoint::GetMaxNormalForce_py)
		.def("SetMaxNormalForce", &pyVpBJoint::SetMaxNormalForce_py)
		.def("GetMaxNormalTorque", &pyVpBJoint::GetMaxNormalTorque_py)
		.def("SetMaxNormalTorque", &pyVpBJoint::SetMaxNormalTorque_py)
		.def("SetHybridDynamicsType", &pyVpBJoint::SetHybridDynamicsType_py)
		.def("GetHybridDynamicsType", &pyVpBJoint::GetHybridDynamicsType_py)
		//m_szName;
		.def("SetOrientation", &pyVpBJoint::SetOrientation_py)
		.def("SetVelocity", &pyVpBJoint::SetVelocity_py)
		.def("SetAcceleration", &pyVpBJoint::SetAcceleration_py)
		.def("SetInitialOrientation", &pyVpBJoint::SetInitialOrientation_py)
		.def("SetElasticity", &pyVpBJoint::SetElasticity_py)
		.def("SetDamping", &pyVpBJoint::SetDamping_py)
		.def("SetTorque", &pyVpBJoint::SetTorque_py)
		.def("AddTorque", &pyVpBJoint::AddTorque_py)
		.def("GetOrientation", &pyVpBJoint::GetOrientation_py)
		.def("GetVelocity", &pyVpBJoint::GetVelocity_py)
		.def("GetAcceleration", &pyVpBJoint::GetAcceleration_py)
		.def("GetTorque", &pyVpBJoint::GetTorque_py)
		.def("GetDOF", &pyVpBJoint::GetDOF_py)
		.def("GetNormalForce", &pyVpBJoint::GetNormalForce_py)
		.def("GetNormalTorque", &pyVpBJoint::GetNormalTorque_py)
		.def("streamOut", &pyVpBJoint::streamOut_py)
		;
}

pyVpBJoint& pyVpBJoint::self()
{
	return *this;
}

////////////////// common vpJoint functions
void pyVpBJoint::Break_py(void)
{
	Break();
}

scalar pyVpBJoint::GetMaxNormalForce_py(void)
{
	return GetMaxNormalForce();
}

void pyVpBJoint::SetMaxNormalForce_py(scalar pyD)
{
	SetMaxNormalForce(pyD);
}

scalar pyVpBJoint::GetMaxNormalTorque_py(void)
{
	return GetMaxNormalTorque();
}
void pyVpBJoint::SetMaxNormalTorque_py(scalar pyD)
{
	SetMaxNormalTorque(pyD);
}

void pyVpBJoint::SetHybridDynamicsType_py(std::string typeStr)
{
    if(typeStr == "KINEMATIC")
        SetHybridDynamicsType(VP::KINEMATIC);
    else if(typeStr == "DYNAMIC")
        SetHybridDynamicsType(VP::DYNAMIC);
    std::cout << "Error: not available Hybrid Dynamics Type." << std::endl;
}

std::string pyVpBJoint::GetHybridDynamicsType_py(void)
{
    VP::HD_TYPE type = GetHybridDynamicsType();
    if (type == VP::DYNAMIC)
        return std::string("DYNAMIC");
    else
        return std::string("KINEMATIC");
}

//////////////////// vpBJoint functions

void pyVpBJoint::SetOrientation_py(object &pyR)
{
	SE3 R = pySO3_2_SE3(pyR);
	SetOrientation(R);
}

void pyVpBJoint::SetVelocity_py(object &pyV)
{
	assert(checkPyVLen(pyV) && "Check python vector length == 3");
	Vec3 V = pyVec3_2_Vec3(pyV);
	SetVelocity(V);
}

void pyVpBJoint::SetAcceleration_py(object &pyV)
{
	assert(checkPyVLen(pyV) && "Check python vector length == 3");
	Vec3 V = pyVec3_2_Vec3(pyV);
	SetAcceleration(V);
}

void pyVpBJoint::SetInitialOrientation_py(object &pyR)
{
	SE3 R = pySO3_2_SE3(pyR);
	SetOrientation(R);
}

void pyVpBJoint::SetElasticity_py(scalar pyD)
{
	SpatialSpring el(pyD);
	SetElasticity(el);
}

void pyVpBJoint::SetDamping_py(scalar pyD)
{
	SpatialDamper dam(pyD);
	SetDamping(dam);
}

void pyVpBJoint::SetTorque_py(object &pyV)
{
	assert(checkPyVLen(pyV) && "Check python vector length == 3");
	Vec3 V = pyVec3_2_Vec3(pyV);
	SetTorque(V);
}

void pyVpBJoint::AddTorque_py(object &pyV)
{
	assert(checkPyVLen(pyV) && "Check python vector length == 3");
	Vec3 V = pyVec3_2_Vec3(pyV);
	AddTorque(V);
}

object pyVpBJoint::GetOrientation_py(void)
{
	SE3 R = GetOrientation();
	object pyR;
	make_pySO3(pyR);
	SE3_2_pySO3(R, pyR);
	return pyR;
}

object pyVpBJoint::GetVelocity_py(void)
{
	Vec3 V = GetVelocity();
	object pyV;
	make_pyVec3(pyV);
	Vec3_2_pyVec3(V, pyV);
	return pyV;
}

object pyVpBJoint::GetAcceleration_py(void)
{
	Vec3 V = GetAcceleration();
	object pyV;
	make_pyVec3(pyV);
	Vec3_2_pyVec3(V, pyV);
	return pyV;
}

object pyVpBJoint::GetTorque_py(void)
{
	Vec3 V = GetTorque();
	object pyV;
	make_pyVec3(pyV);
	Vec3_2_pyVec3(V, pyV);
	return pyV;

}

int pyVpBJoint::GetDOF_py(void)
{
	return GetDOF();
}

scalar pyVpBJoint::GetNormalForce_py(void)
{
	return GetNormalForce();
}

scalar pyVpBJoint::GetNormalTorque_py(void)
{
	return GetNormalTorque();
}

std::string pyVpBJoint::streamOut_py()
{
	std::stringstream ss;
	streamOut(ss);
	return ss.str();
}
