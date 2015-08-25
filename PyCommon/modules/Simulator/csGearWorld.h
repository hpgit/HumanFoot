#pragma once

#include <gear/gear.h>

class GearWorld
{
public:
	GSystem _system;
	double _timeStep;
	GBody _ground;
	double _planeHeight;

private:
	//bool _calcPenaltyForce(const vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu);

public:	// expose to python
	GearWorld(const object& config);
	//void step();
	//void initialize();
	//tuple calcPenaltyForce(const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds);
	//void applyPenaltyForce(const bp::list& bodyIDs, const bp::list& positions, const bp::list& forces);
	//int getBodyNum() { return _world.GetNumBody(); }
};
