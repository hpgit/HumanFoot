#include "stdafx.h"

#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include <VP/vpDataType.h>

#define make_tuple boost::python::make_tuple

BOOST_PYTHON_MODULE(VpWorld)
{
	class_<VpWorld>("VpWorld", init<>())
	    //.def_readonly("_world", &VpWorld::_world)
	    .def_readwrite("_world", &VpWorld::_world)
		.def("step", &VpWorld::step)
		.def("initialize", &VpWorld::initialize)
		.def("calcPenaltyForce", &VpWorld::calcPenaltyForce)
		.def("applyPenaltyForce", &VpWorld::applyPenaltyForce)
		.def("getBodyNum", &VpWorld::getBodyNum)
		.def("getContactPoints", &VpWorld::getContactPoints)
		.def("getTimeStep", &VpWorld::getTimeStep)
		.def("AddBody", &VpWorld::AddBody)
		.def("AddWorld", &VpWorld::AddWorld)
		//.def(" //void								 SetGlobalFrame(const SE3 &);
		//.def(" //const SE3							&GetGlobalFrame(void);
		.def("Initialize", &VpWorld::Initialize)
		.def("SetTimeStep", &VpWorld::SetTimeStep)
		.def("GetTimeStep", &VpWorld::GetTimeStep)
		.def("SetIntegrator", &VpWorld::SetIntegrator)
		.def("GetBoundingSphere", &VpWorld::GetBoundingSphere)
		.def("SetGravity",  &VpWorld::SetGravity)
		.def("GetGravity", &VpWorld::GetGravity)
		.def("EnableCollision", &VpWorld::EnableCollision)
		.def("IgnoreCollision", &VpWorld::IgnoreCollision)
		.def("GetSimulationTime", &VpWorld::GetSimulationTime)
		.def("GetKineticEnergy", &VpWorld::GetKineticEnergy)
		.def("GetPotentialEnergy", &VpWorld::GetPotentialEnergy)
		.def("GetTotalEnergy", &VpWorld::GetTotalEnergy)
		.def("GetNumBody", &VpWorld::GetNumBody)
		//.def(" //const vpBody						*GetBody(int) const;
		//.def(" //vpBody								*GetBody(int);
		//.def(" //const vpBody						*GetBodyByName(const string &name) const;
		.def("GetNumGeometry", &VpWorld::GetNumGeometry)
		.def("BackupState", &VpWorld::BackupState)
		.def("RollbackState", &VpWorld::RollbackState)
		.def("UpdateFrame", &VpWorld::UpdateFrame)
		.def("GetNumMaterial", &VpWorld::GetNumMaterial)
		//.def(" //const vpMaterial					*GetMaterial(int) const;
		//.def(" //const vpMaterial					*GetMaterialByName(const string &name) const;
		.def("GetNumJoint", &VpWorld::GetNumJoint)
		//.def(" //const vpJoint						*GetJoint(int) const;
		//.def(" //const vpJoint						*GetJointByName(const string &name) const;
		.def("Clear", &VpWorld::Clear)
		//.def(" //void								 report(ostream &);
		.def("GetNumCollision", &VpWorld::GetNumCollision)
		.def("GetNumContact", &VpWorld::GetNumContact)
		.def("SetNumThreads", &VpWorld::SetNumThreads)
		.def("GetNumThreads", &VpWorld::GetNumThreads)
		.def("SetGlobalDamping", &VpWorld::SetGlobalDamping)
		.def("GetGlobalDampling", &VpWorld::GetGlobalDampling)
		.def("GetFrameCount", &VpWorld::GetFrameCount)
		.def("ReportStatistics", &VpWorld::ReportStatistics)
		.def("ResetStatistics", &VpWorld::ResetStatistics)
	;
}


VpWorld::VpWorld(const object& config)
{
	_world.SetTimeStep(XD(config.attr("timeStep")));
	_world.SetGravity(pyVec3_2_Vec3(config.attr("gravity")));
	setOpenMP();

	//std::cout << _world.GetGlobalDampling() << std::endl;
	//_world.SetGlobalDamping(0.99);

	_planeHeight = XD(config.attr("planeHeight"));
	_lockingVel = XD(config.attr("lockingVel"));

	if(XB(config.attr("useDefaultContactModel")))
	{
		vpMaterial::GetDefaultMaterial()->SetRestitution(0.01);
		vpMaterial::GetDefaultMaterial()->SetDynamicFriction(100);
		vpMaterial::GetDefaultMaterial()->SetStaticFriction(100);
		_ground.AddGeometry(new vpBox(Vec3(100, 0, 100)));
		_ground.SetFrame(Vec3(0, _planeHeight, 0));
		_ground.SetGround();
		_world.AddBody(&_ground);
	}
}

void VpWorld::initialize()
{
	_world.Initialize();

	//_world.SetIntegrator(VP::IMPLICIT_EULER);
	_world.SetIntegrator(VP::IMPLICIT_EULER_FAST);
}

void VpWorld::setOpenMP()
{
	int a = 1;

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	//#pragma omp parallel
	std::cout << "OpenMP versions: " << _OPENMP << std::endl;
	std::cout << "OpenMP max threads: " << omp_get_max_threads() << std::endl;
	_world.SetNumThreads((a = omp_get_max_threads()-2));
	std::cout << "csVpWorld: parallelized with " << a << " cores" << std::endl;
#else
    std::cout << "OpenMP is not supported in this environment." << std::endl;
#endif

}


/*********************
vpWorld wrapper
*********************/

	//add a body to the world
void AddBody(object & objBody)
{
    _world.AddBody(&(pBody.attr("_body")));
}

	//add a world
	//World's parameters such as a time step and the gravity will not be updated with newly added world'parameters.
	//Only the body and joints will be added.
void AddWorld(object & objWorld)
{
    _world.AddWorld(&(objWorld.attr("_world")));
}

	//	set a global frame
	//	All the coordinate frame is represented by this global frame.
	//	Should be followed by vpWorld::Initialize.
/*
void SetGlobalFrame(const SE3 &)
{

}
*/

	//set a global frame
	//const SE3							&GetGlobalFrame(void);
void VpWorld::Initialize()
{
	_world.Initialize();
}

void VpWorld::SetTimeStep(scalar t)
{
	_world.SetTimeStep(t);
}

scalar VpWorld::GetTimeStep()
{
	return _world.GetTimeStep();
}

void VpWorld::SetIntegrator(std::string typeStr)
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
	_world.SetIntegrator(type);
}

object VpWorld::GetBoundingSphere()
{
	Vec3 center;
	scalar rad = _world.GetBoundingSphere(center);
	return make_tuple(rad, Vec3_2_pyVec3(center));
}

void VpWorld::SetGravity(const object &g)
{
	_world.SetGravity(pyVec3_2_Vec3(g));
}

object VpWorld::GetGravity(void)
{
	Vec3 g = _world.GetGravity();
	return Vec3_2_pyVec3(g);
}

//void								 EnableCollision();

//void								 IgnoreCollision(vpBody *B0, vpBody *B1);

scalar VpWorld::GetSimulationTime()
{
	return _world.GetSimulationTime();
}

scalar VpWorld::GetKineticEnergy()
{
	return _world.GetKineticEnergy();
}

scalar VpWorld::GetPotentialEnergy()
{
	return _world.GetPotentialEnergy();
}

scalar VpWorld::GetTotalEnergy()
{
	return _world.GetTotalEnergy();
}

int VpWorld::GetNumBody()
{
	return _world.GetNumBody();
}

	//	get a pointer to the ith body.
	//	\sa vpBody::GetID
	//const vpBody						*GetBody(int) const;
	//vpBody								*GetBody(int);

	//get a pointer to the body with the name
	//const vpBody						*GetBodyByName(const string &name) const;

int VpWorld::GetNumGeometry()
{
	return _world.GetNumGeometry();
}

void VpWorld::BackupState()
{
	_world.BackupState();
}

void VpWorld::RollbackState()
{
	_world.RollbackState();
}

void VpWorld::UpdateFrame()
{
	_world.UpdateFrame();
}

int VpWorld::GetNumMaterial()
{
	return _world.GetNumMaterial();
}

	//get a pointer to the ith material
	//const vpMaterial					*GetMaterial(int) const;

	//get a pointer to the material with the name
	//const vpMaterial					*GetMaterialByName(const string &name) const;

int VpWorld::GetNumJoint()
{
	return _world.GetNumJoint();
}

	//get a pointer to the ith joint
	//const vpJoint						*GetJoint(int) const;

	//get a pointer to the joint with the name
	//const vpJoint						*GetJointByName(const string &name) const;

void VpWorld::Clear()
{
	_world.Clear();
}

	//void								 report(ostream &);

int VpWorld::GetNumCollision()
{
	return _world.GetNumCollision();
}

int VpWorld::GetNumContact()
{
	return _world.GetNumContact();
}

void VpWorld::SetNumThreads(int n)
{
	_world.SetNumThreads(n);
}
int VpWorld::GetNumThreads()
{
	return _world.GetNumThreads();
}

void VpWorld::SetGlobalDamping(scalar d)
{
	_world.SetGlobalDamping(d);
}

scalar VpWorld::GetGlobalDampling()
{
	return _world.GetGlobalDampling();
}

int VpWorld::GetFrameCount()
{
	return _world.GetFrameCount();
}

void VpWorld::ReportStatistics()
{
	_world.ReportStatistics();
}

void VpWorld::ResetStatistics()
{
	_world.ResetStatistics();
}
