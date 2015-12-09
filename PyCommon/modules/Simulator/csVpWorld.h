#pragma once

#include <VP/vphysics.h>

class VpWorld
{
public:
	vpWorld _world;
	double _timeStep;
	vpBody _ground;
	double _planeHeight;
	double _lockingVel;

private:
	bool _calcPenaltyForce(const vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu);

public:	// expose to python
	VpWorld(const object& config);
	void step();
	void initialize();
	void setOpenMP();
	boost::python::tuple getContactPoints(const bp::list& bodyIDsToCheck);
	boost::python::tuple calcPenaltyForce(const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds);
	void applyPenaltyForce(const bp::list& bodyIDs, const bp::list& positions, const bp::list& forces);
	int getBodyNum() { return _world.GetNumBody(); }


	//add a body to the world
	//void								 AddBody(vpBody *);

	//add a world
	//World's parameters such as a time step and the gravity will not be updated with newly added world'parameters.
	//Only the body and joints will be added.
	//void								 AddWorld(vpWorld *);

	//	set a global frame
	//	All the coordinate frame is represented by this global frame.
	//	Should be followed by vpWorld::Initialize.
	//void								 SetGlobalFrame(const SE3 &);
	
	//set a global frame
	//const SE3							&GetGlobalFrame(void);

	/*!
		initialize the world.
		Before simulation, the world should be initialized.
	*/
	void								 Initialize(void);


	//set simulation time step used in integrating dynamics equations.
	//Default time step is 0.001. For stable simulation, smaller time step is preferred.
	//However how small the time step can depend on your model. 
	void								 SetTimeStep(scalar);

	//get a current time step.
	scalar								 GetTimeStep(void);

	//choose an integration algorithm used in simulation.
	//\param type VP::EULER is faster but less accurate than VP::RK4.
	void								 SetIntegrator(std::string);

	//get a bounding sphere including whole world.
	object								 GetBoundingSphere();

	//set a gravity. Default is zero gravity.
	void								 SetGravity(const object &);

	//get a gravity.
	object								 GetGravity(void);

	//enable or disable collision between bodies.
	//\sa vpWorld::IgnoreCollision()
	//void								 EnableCollision(bool = true);

	//declare that collision of B0 and B1 will be ignored.
	//void								 IgnoreCollision(vpBody *B0, vpBody *B1);

	//get a simulation time.
	scalar								 GetSimulationTime(void);

	//get a kinetic energy.
	scalar								 GetKineticEnergy(void);

	//get a potential energy.
	scalar								 GetPotentialEnergy(void);

	//get a total energy(= kinetic energy + potential energy).
	scalar								 GetTotalEnergy(void);

	//get a number of bodies in the world.
	int									 GetNumBody(void);

	//get a number of geometries in the world.
	int									 GetNumGeometry(void);

	//	get a pointer to the ith body.
	//	\sa vpBody::GetID
	//const vpBody						*GetBody(int) const;
	//vpBody								*GetBody(int);

	//get a pointer to the body with the name
	//const vpBody						*GetBodyByName(const string &name) const;
	
	//keep current states which are transformation matrices, velocities, joint angles, joint velocities.
	//\sa vpWorld::RestoreState
	void								 BackupState(void);

	//restore states to the values kept by BackupState()
	//\sa vpWorld::KeepCurrentState
	void								 RollbackState(void);

	//update transformation matrices from joint angles.
	//
	//It is useful when you change joint angles and want to compute corresponding transformation matrices of articulated bodies.
	//Basically, VP does not compute transformation matrices of bodies even if you change the joint angles.
	//The transformation matrices or relevant values will be updated after the simulation which is typically done by calling vpWorld::StepAhead().		
	void								 UpdateFrame(void);

	//get a number of materials defined in the world
	int									 GetNumMaterial(void);

	//get a pointer to the ith material
	//const vpMaterial					*GetMaterial(int) const;

	//get a pointer to the material with the name
	//const vpMaterial					*GetMaterialByName(const string &name) const;

	//get a number of joints in the world
	int									 GetNumJoint(void);

	//get a pointer to the ith joint
	//const vpJoint						*GetJoint(int) const;

	//get a pointer to the joint with the name
	//const vpJoint						*GetJointByName(const string &name) const;

	/*!
		clear all the instances managed by the world
	*/
	void								 Clear(void);

	//void								 report(ostream &);

	int									 GetNumCollision(void);
	int									 GetNumContact(void);

	void								 SetNumThreads(int);
	int									 GetNumThreads(void);

	void								 SetGlobalDamping(scalar);
	scalar								 GetGlobalDampling(void);

	int									 GetFrameCount(void);

	void								 ReportStatistics(void);
	void								 ResetStatistics(void);
};
