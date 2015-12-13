#ifndef _PYVPWORLD_H_
#define _PYVPWORLD_H_

#include <VP/vphysics.h>
#include "pyVpBody.h"

class pyVpBody;
class pyVpBJoint;
class pyVpMaterial;
class pyVpSystem;

class pyVpWorld : public vpWorld
{
public:
    pyVpWorld&                          self();
	//add a body to the world
	void								 AddBody_py(pyVpBody *objBody);

	//add a world
	//World's parameters such as a time step and the gravity will not be updated with newly added world'parameters.
	//Only the body and joints will be added.
	void								 AddWorld_py(pyVpWorld *objWorld);

	//	set a global frame
	//	All the coordinate frame is represented by this global frame.
	//	Should be followed by vpWorld::Initialize.
	//TODO:
	void								 SetGlobalFrame_py(object & pySE3);

	//set a global frame
	object							    GetGlobalFrame_py(void);

	/*!
		initialize the world.
		Before simulation, the world should be initialized.
	*/
	void								 Initialize_py(void);


	//set simulation time step used in integrating dynamics equations.
	//Default time step is 0.001. For stable simulation, smaller time step is preferred.
	//However how small the time step can depend on your model.
	void								 SetTimeStep_py(scalar);

	//get a current time step.
	scalar								 GetTimeStep_py(void);

	//choose an integration algorithm used in simulation.
	//\param type VP::EULER is faster but less accurate than VP::RK4.
	void								 SetIntegrator_py(std::string);

	//get a bounding sphere including whole world.
	object								 GetBoundingSphere_py();

	//set a gravity. Default is zero gravity.
	void								 SetGravity_py(object &);

	//get a gravity.
	object								 GetGravity_py(void);

	//enable or disable collision between bodies.
	//\sa vpWorld::IgnoreCollision_py()
	void								 EnableCollision_py(bool = true);

	//declare that collision of B0 and B1 will be ignored.
	void								 IgnoreCollision_py(object &objBody0, object &objBody1);

	//get a simulation time.
	scalar								 GetSimulationTime_py(void);

	//get a kinetic energy.
	scalar								 GetKineticEnergy_py(void);

	//get a potential energy.
	scalar								 GetPotentialEnergy_py(void);

	//get a total energy_py(= kinetic energy + potential energy).
	scalar								 GetTotalEnergy_py(void);

	//get a number of bodies in the world.
	int									 GetNumBody_py(void);

	//get a number of geometries in the world.
	int									 GetNumGeometry_py(void);

    //TODO:
	//	get a pointer to the ith body.
	//	\sa vpBody::GetID
	//const vpBody						*GetBody_py(int) const;
	pyVpBody&							GetBody_py(int);

	//get a pointer to the body with the name
	//const vpBody						*GetBodyByName_py(const string &name) const;

	//keep current states which are transformation matrices, velocities, joint angles, joint velocities.
	//\sa vpWorld::RestoreState
	void								 BackupState_py(void);

	//restore states to the values kept by BackupState_py()
	//\sa vpWorld::KeepCurrentState
	void								 RollbackState_py(void);

	//update transformation matrices from joint angles.
	//
	//It is useful when you change joint angles and want to compute corresponding transformation matrices of articulated bodies.
	//Basically, VP does not compute transformation matrices of bodies even if you change the joint angles.
	//The transformation matrices or relevant values will be updated after the simulation which is typically done by calling vpWorld::StepAhead_py().
	void								 UpdateFrame_py(void);

	//get a number of materials defined in the world
	int									 GetNumMaterial_py(void);

    //TODO:
	//get a pointer to the ith material
	pyVpMaterial&				GetMaterial_py(int);

	//get a pointer to the material with the name
	//const vpMaterial					*GetMaterialByName_py(const string &name) const;

	//get a number of joints in the world
	int									 GetNumJoint_py(void);

    //TODO:
	//get a pointer to the ith joint
	pyVpBJoint&					GetJoint_py(int);

	//get a pointer to the joint with the name
	//const vpJoint						*GetJointByName_py(const string &name) const;

	/*!
		clear all the instances managed by the world
	*/
	void								 Clear_py(void);

	std::string							 report_py();

	int									 GetNumCollision_py(void);
	int									 GetNumContact_py(void);

	void								 SetNumThreads_py(int);
	int									 GetNumThreads_py(void);

	void								 SetGlobalDamping_py(scalar);
	scalar								 GetGlobalDampling_py(void);

	int									 GetFrameCount_py(void);

	void								 ReportStatistics_py(void);
	void								 ResetStatistics_py(void);
};

#endif // _PYVPWORLD_H_
