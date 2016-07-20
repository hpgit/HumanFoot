#include "stdafx.h"

#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"

#include "csDartWorld.h"

#define make_tuple boost::python::make_tuple

BOOST_PYTHON_MODULE(csVpWorld)
{
	class_<VpWorld>("VpWorld", init<const object&>())
	    .def_readwrite("_ground", &VpWorld::_ground)
	    .def("self", &VpWorld::self, return_value_policy<reference_existing_object>())
		.def("step", &VpWorld::step)
		.def("initialize", &VpWorld::initialize)
		.def("calcPenaltyForce", &VpWorld::calcPenaltyForce)
		.def("applyPenaltyForce", &VpWorld::applyPenaltyForce)
		.def("getBodyNum", &VpWorld::getBodyNum)
		.def("getContactPoints", &VpWorld::getContactPoints)
		//.def("getTimeStep", &VpWorld::getTimeStep)
		//.def(" //void								 AddBody(vpBody *);
		//.def(" //void								 AddWorld(vpWorld *);
		//.def(" //void								 SetGlobalFrame(const SE3 &);
		//.def(" //const SE3							&GetGlobalFrame(void);
		.def("Initialize", &VpWorld::Initialize)
		.def("SetTimeStep", &VpWorld::SetTimeStep)
		.def("GetTimeStep", &VpWorld::GetTimeStep)
		.def("SetIntegrator", &VpWorld::SetIntegrator)
		.def("GetBoundingSphere", &VpWorld::GetBoundingSphere)
		.def("SetGravity",  &VpWorld::SetGravity)
		.def("GetGravity", &VpWorld::GetGravity)
		//.def(" //void								 EnableCollision();
		//.def(" //void								 IgnoreCollision(vpBody *B0, vpBody *B1);
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


DartWorld::DartWorld(const object& config, std::string worldXML)
{
	myWorld = dart::utils::SkelParser::readWorldXML(worldXML);
}