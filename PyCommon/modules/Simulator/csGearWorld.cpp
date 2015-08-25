#include "stdafx.h"

#include "GearUtil.h"
#include "csGearWorld.h"

#define MAX_X 1	// 0001
#define MAX_Y 2	// 0010
#define MAX_Z 4	// 0100

BOOST_PYTHON_MODULE(csGearWorld)
{
	class_<GearWorld>("GearWorld", init<const object&>())
		//.def("step", &VpWorld::step)
		//.def("initialize", &VpWorld::initialize)
		//.def("calcPenaltyForce", &VpWorld::calcPenaltyForce)
		//.def("applyPenaltyForce", &VpWorld::applyPenaltyForce)
		//.def("getBodyNum", &VpWorld::getBodyNum)
	;
}

GearWorld::GearWorld(const object& config)
{
	_timeStep = XD(config.attr("timeStep"));
	//_world.SetGravity(pyVec3_2_Vec3(config.attr("gravity")));
	_planeHeight = XD(config.attr("planeHeight"));
}
