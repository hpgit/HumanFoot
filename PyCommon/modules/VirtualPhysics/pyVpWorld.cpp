#include "stdafx.h"

#include "pyVpWorld.h"
#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"
#include <VP/vpDataType.h>

#include <sstream>


#define make_tuple boost::python::make_tuple

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(pyVpWorld_EnableCollision_py_overloads, EnableCollision_py, 0, 1);

BOOST_PYTHON_MODULE(vpWorld)
{
    numeric::array::set_module_and_type("numpy", "ndarray");
    
	class_<pyVpWorld>("vpWorld", init<>())
	    .def("self", &pyVpWorld::self, return_value_policy<reference_existing_object>())
		.def("AddBody", &pyVpWorld::AddBody_py)
		.def("AddWorld", &pyVpWorld::AddWorld_py)
		.def("SetGlobalFrame", &pyVpWorld::SetGlobalFrame_py)
		.def("GetGlobalFrame", &pyVpWorld::GetGlobalFrame_py)
		.def("Initialize", &pyVpWorld::Initialize_py)
		.def("SetTimeStep", &pyVpWorld::SetTimeStep_py)
		.def("GetTimeStep", &pyVpWorld::GetTimeStep_py)
		.def("SetIntegrator", &pyVpWorld::SetIntegrator_py)
		.def("GetBoundingSphere", &pyVpWorld::GetBoundingSphere_py)
		.def("SetGravity",  &pyVpWorld::SetGravity_py)
		.def("GetGravity", &pyVpWorld::GetGravity_py)
		.def("EnableCollision", &pyVpWorld::EnableCollision_py, pyVpWorld_EnableCollision_py_overloads())
		.def("IgnoreCollision", &pyVpWorld::IgnoreCollision_py)
		.def("GetSimulationTime", &pyVpWorld::GetSimulationTime_py)
		.def("GetKineticEnergy", &pyVpWorld::GetKineticEnergy_py)
		.def("GetPotentialEnergy", &pyVpWorld::GetPotentialEnergy_py)
		.def("GetTotalEnergy", &pyVpWorld::GetTotalEnergy_py)
		.def("GetNumBody", &pyVpWorld::GetNumBody_py)
		// .def("GetBody", &pyVpWorld::GetBody_py, return_value_policy<reference_existing_object>())
		.def("GetBody", &pyVpWorld::GetBody_py, return_value_policy<copy_non_const_reference>())
		//.def(" //const vpBody						*GetBodyByName(const string &name_py) const;
		.def("GetNumGeometry", &pyVpWorld::GetNumGeometry_py)
		.def("BackupState", &pyVpWorld::BackupState_py)
		.def("RollbackState", &pyVpWorld::RollbackState_py)
		.def("UpdateFrame", &pyVpWorld::UpdateFrame_py)
		.def("GetNumMaterial", &pyVpWorld::GetNumMaterial_py)
		.def("GetMaterial", &pyVpWorld::GetMaterial, return_value_policy<reference_existing_object>())
		//.def(" //const vpMaterial					*GetMaterialByName(const string &name_py) const;
		.def("GetNumJoint", &pyVpWorld::GetNumJoint_py)
		.def("GetJoint", &pyVpWorld::GetJoint, return_value_policy<reference_existing_object>())
		//.def(" //const vpJoint						*GetJointByName(const string &name_py) const;
		.def("Clear", &pyVpWorld::Clear_py)
		.def("report", &pyVpWorld::report_py)
		.def("GetNumCollision", &pyVpWorld::GetNumCollision_py)
		.def("GetNumContact", &pyVpWorld::GetNumContact_py)
		.def("SetNumThreads", &pyVpWorld::SetNumThreads_py)
		.def("GetNumThreads", &pyVpWorld::GetNumThreads_py)
		.def("SetGlobalDamping", &pyVpWorld::SetGlobalDamping_py)
		.def("GetGlobalDampling", &pyVpWorld::GetGlobalDampling_py)
		.def("GetFrameCount", &pyVpWorld::GetFrameCount_py)
		.def("ReportStatistics", &pyVpWorld::ReportStatistics_py)
		.def("ResetStatistics", &pyVpWorld::ResetStatistics_py)
	;
}


/*********************
vpWorld wrapper
*********************/

pyVpWorld& pyVpWorld::self()
{
    return *this;
}

//TODO:
//check!!!!!!!!!!
//add a body to the world
void pyVpWorld::AddBody_py(pyVpBody * objBody)
{
    AddBody(reinterpret_cast<vpBody*>(objBody));
}

//TODO:
//check!!!!!!!!!!
//add a world
//World's parameters such as a time step and the gravity will not be updated with newly added world'parameters.
//Only the body and joints will be added.
void pyVpWorld::AddWorld_py(pyVpWorld * objWorld)
{
    AddWorld(reinterpret_cast<vpWorld*>(objWorld));
}

//	set a global frame
//	All the coordinate frame is represented by this global frame.
//	Should be followed by vpWorld::Initialize.
void pyVpWorld::SetGlobalFrame_py(object &pySE3)
{
    SE3 vpSE3 = pySE3_2_SE3(pySE3);
    SetGlobalFrame(vpSE3);
}

//get a global frame
object pyVpWorld::GetGlobalFrame_py(void)
{
    SE3 T = GetGlobalFrame();
	numeric::array O(make_tuple(make_tuple(0., 0., 0., 0.),
	            make_tuple(0., 0., 0., 0.),
	            make_tuple(0., 0., 0., 0.),
	            make_tuple(0.,0.,0.,0.)));
	object pySE3 = O.copy();
	SE3_2_pySE3(T, pySE3);

    return pySE3;
}
void pyVpWorld::Initialize_py()
{
	Initialize();
}

void pyVpWorld::SetTimeStep_py(scalar t)
{
	SetTimeStep(t);
}

scalar pyVpWorld::GetTimeStep_py()
{
	return GetTimeStep();
}

void pyVpWorld::SetIntegrator_py(std::string typeStr)
{
	VP::INTEGRATOR_TYPE type = VP::RK4;
	if(!typeStr.compare("IMPLICIT_EULER"))
		type = VP::IMPLICIT_EULER;
	else if(!typeStr.compare("IMPLICIT_EULER_FAST"))
		type = VP::IMPLICIT_EULER_FAST;
	else if(!typeStr.compare("EULER"))
		type = VP::EULER;
	else
		type = VP::RK4;
	SetIntegrator(type);
}

object pyVpWorld::GetBoundingSphere_py()
{
	Vec3 center;
	scalar rad = GetBoundingSphere(center);
	return make_tuple(rad, Vec3_2_pyVec3(center));
}

void pyVpWorld::SetGravity_py(object &g)
{
	SetGravity(pyVec3_2_Vec3(g));
}

object pyVpWorld::GetGravity_py(void)
{
	Vec3 g = GetGravity();
	return Vec3_2_pyVec3(g);
}

void pyVpWorld::EnableCollision_py(bool isCollision)
{
    EnableCollision(isCollision);
}

void pyVpWorld::IgnoreCollision_py(object &objBody0, object &objBody1)
{
    IgnoreCollision(reinterpret_cast<vpBody*>(&objBody0), reinterpret_cast<vpBody*>(&objBody1));
}

scalar pyVpWorld::GetSimulationTime_py()
{
	return GetSimulationTime();
}

scalar pyVpWorld::GetKineticEnergy_py()
{
	return GetKineticEnergy();
}

scalar pyVpWorld::GetPotentialEnergy_py()
{
	return GetPotentialEnergy();
}

scalar pyVpWorld::GetTotalEnergy_py()
{
	return GetTotalEnergy();
}

int pyVpWorld::GetNumBody_py()
{
	return GetNumBody();
}

//	get a pointer to the ith body.
	//	\sa vpBody::GetID
	//const vpBody						*GetBody(int) const;
pyVpBody& pyVpWorld::GetBody_py(int pyI)
{
	// return reinterpret_cast<pyVpBody*>(GetBody(pyI));
	return *(reinterpret_cast<pyVpBody*>(GetBody(pyI)));
}

	//get a pointer to the body with the name
	//const vpBody						*GetBodyByName(const string &name) const;

int pyVpWorld::GetNumGeometry_py()
{
	return GetNumGeometry();
}

void pyVpWorld::BackupState_py()
{
	BackupState();
}

void pyVpWorld::RollbackState_py()
{
	RollbackState();
}

void pyVpWorld::UpdateFrame_py()
{
	UpdateFrame();
}

int pyVpWorld::GetNumMaterial_py()
{
	return GetNumMaterial();
}

pyVpMaterial& pyVpWorld::GetMaterial_py(int pyI)
{
	return *reinterpret_cast<pyVpMaterial *>(const_cast<vpMaterial*>(GetMaterial(pyI)));
}

	//get a pointer to the material with the name
	//const vpMaterial					*GetMaterialByName(const string &name) const;

int pyVpWorld::GetNumJoint_py()
{
	return GetNumJoint();
}

pyVpBJoint& pyVpWorld::GetJoint_py(int pyI)
{
	// return *const_cast<vpJoint*>(GetJoint(pyI));
	return *reinterpret_cast<pyVpBJoint*>(const_cast<vpJoint*>(GetJoint(pyI)));
}

	//get a pointer to the joint with the name
	//const vpJoint						*GetJointByName(const string &name) const;

void pyVpWorld::Clear_py()
{
	Clear();
}

std::string pyVpWorld::report_py()
{
	std::stringstream ss;
    report(ss);
    return ss.str();
}

int pyVpWorld::GetNumCollision_py()
{
	return GetNumCollision();
}

int pyVpWorld::GetNumContact_py()
{
	return GetNumContact();
}

void pyVpWorld::SetNumThreads_py(int n)
{
	SetNumThreads(n);
}
int pyVpWorld::GetNumThreads_py()
{
	return GetNumThreads();
}

void pyVpWorld::SetGlobalDamping_py(scalar d)
{
	SetGlobalDamping(d);
}

scalar pyVpWorld::GetGlobalDampling_py()
{
	return GetGlobalDampling();
}

int pyVpWorld::GetFrameCount_py()
{
	return GetFrameCount();
}

void pyVpWorld::ReportStatistics_py()
{
	ReportStatistics();
}

void pyVpWorld::ResetStatistics_py()
{
	ResetStatistics();
}
