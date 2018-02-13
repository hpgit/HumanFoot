#include "stdafx.h"

#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"

#include "csVpModel.h"
#include "csVpWorld.h"
#include "myGeom.h"

using boost::python::make_tuple;
namespace np = boost::python::numpy;
using boost::python::numpy::ndarray;

#define MAX_X 1	// 0001
#define MAX_Y 2	// 0010
#define MAX_Z 4	// 0100

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


VpWorld::VpWorld(const object& config)
{
    boost::python::numpy::initialize();
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

void VpWorld::step()
{
	_world.StepAhead();
}

void VpWorld::initialize()
{
	_world.Initialize();

	//_world.SetIntegrator(VP::IMPLICIT_EULER);
	_world.SetIntegrator(VP::IMPLICIT_EULER_FAST);
}

void VpWorld::setOpenMP()
{
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	int numThreads = 1;
	//#pragma omp parallel
	std::cout << "OpenMP versions: " << _OPENMP << std::endl;
	std::cout << "OpenMP max threads: " << omp_get_max_threads() << std::endl;
	numThreads = omp_get_max_threads();
	if (numThreads <= 0) numThreads = 1;
	_world.SetNumThreads(numThreads);
	std::cout << "csVpWorld: parallelized with " << numThreads << " cores" << std::endl;
#else
    std::cout << "OpenMP is not supported in this environment." << std::endl;
#endif

}

// @return ( bodyIDs, positions, postionLocals, forces)
boost::python::tuple VpWorld::calcPenaltyForce( const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds )
{
	bp::list bodyIDs, positions, forces, positionLocals, velocities;
	int bodyID;
	ndarray O_Vec3 = np::array(make_tuple(0., 0., 0.));
	const vpBody* pBody;
	vpGeom* pGeom;
	char type;
	scalar data[3];
	Vec3 position, velocity, force, positionLocal;

	for (int i = 0; i<len(bodyIDsToCheck); ++i)
	{
		bodyID = XI(bodyIDsToCheck[i]);
		pBody = _world.GetBody(bodyID);

		for (int j = 0; j<pBody->GetNumGeometry(); ++j)
		{
			pGeom = pBody->GetGeometry(j);
			
			pGeom->GetShape(&type, data);
			if (type == 'B')
			{
				const SE3& geomFrame = pGeom->GetGlobalFrame();

				for (int p = 0; p<8; ++p)
				{
					positionLocal[0] = (p & MAX_X) ? data[0] / 2. : -data[0] / 2.;
					positionLocal[1] = (p & MAX_Y) ? data[1] / 2. : -data[1] / 2.;
					positionLocal[2] = (p & MAX_Z) ? data[2] / 2. : -data[2] / 2.;
					position = geomFrame * positionLocal;

					velocity = pBody->GetLinVelocity(positionLocal);

					bool penentrated = _calcPenaltyForce(pBody, position, velocity, force, Ks, Ds, XD(mus[i]));
					if (penentrated)
					{
						bodyIDs.append(bodyID);

						object pyPosition = O_Vec3.copy();
						Vec3_2_pyVec3(position, pyPosition);
						positions.append(pyPosition);

						object pyVelocity = O_Vec3.copy();
						Vec3_2_pyVec3(velocity, pyVelocity);
						velocities.append(pyVelocity);

						object pyForce = O_Vec3.copy();
						Vec3_2_pyVec3(force, pyForce);
						forces.append(pyForce);

						object pyPositionLocal = O_Vec3.copy();
						Vec3_2_pyVec3(positionLocal, pyPositionLocal);
						positionLocals.append(pyPositionLocal);
					}
				}
			}
			else if (type == 'C' || type == 'D' || type == 'E' || type == 'M' || type == 'N')
			{
				const vector<Vec3>& verticesLocal = type == 'C'? ((MyFoot3*)pGeom)->getVerticesLocal() :
													type == 'D'? ((MyFoot4*)pGeom)->getVerticesLocal() :
													type == 'E'? ((MyFoot5*)pGeom)->getVerticesLocal() :
													type == 'M'? ((MyFoot1*)pGeom)->getVerticesLocal() :
																 ((MyFoot2*)pGeom)->getVerticesLocal();
				const vector<Vec3>& verticesGlobal = type == 'C'? ((MyFoot3*)pGeom)->getVerticesGlobal() :
													type == 'D'? ((MyFoot4*)pGeom)->getVerticesGlobal() :
													type == 'E'? ((MyFoot5*)pGeom)->getVerticesGlobal() :
													type == 'M'? ((MyFoot1*)pGeom)->getVerticesGlobal() :
																 ((MyFoot2*)pGeom)->getVerticesGlobal();
				for (int k = 0; k < verticesLocal.size(); ++k)
				{

					positionLocal = verticesLocal[k];
					position = verticesGlobal[k];
					velocity = pBody->GetLinVelocity(positionLocal);

					bool penentrated = _calcPenaltyForce(pBody, position, velocity, force, Ks, Ds, XD(mus[i]));
					if (penentrated)
					{

						bodyIDs.append(bodyID);

						object pyPosition = O_Vec3.copy();
						Vec3_2_pyVec3(position, pyPosition);
						positions.append(pyPosition);

						object pyForce = O_Vec3.copy();
						Vec3_2_pyVec3(force, pyForce);
						forces.append(pyForce);

						object pyVelocity = O_Vec3.copy();
						Vec3_2_pyVec3(velocity, pyVelocity);
						velocities.append(pyVelocity);

						object pyPositionLocal = O_Vec3.copy();
						Vec3_2_pyVec3(positionLocal, pyPositionLocal);
						positionLocals.append(pyPositionLocal);
					}
				}
			}
		}
	}

	/*
	bp::list bodyIDs, positions, forces, positionLocals;
	int bodyID;
	static numeric::array O_Vec3(make_tuple(0.,0.,0.));
	const vpBody* pBody;
	const vpGeom* pGeom;
	char type;
	scalar data[3];
	static Vec3 position, velocity, force, positionLocal;

	for(int i=0; i<len(bodyIDsToCheck); ++i)
	{
		bodyID = XI(bodyIDsToCheck[i]);
		pBody = _world.GetBody(bodyID);
		
		for(int j=0; j<pBody->GetNumGeometry(); ++j)
		{
			pGeom = pBody->GetGeometry(j);
			pGeom->GetShape(&type, data);
			const SE3& geomFrame = pGeom->GetGlobalFrame();

			for( int p=0; p<8; ++p)
			{
				positionLocal[0] = (p & MAX_X) ? data[0]/2. : -data[0]/2.;
				positionLocal[1] = (p & MAX_Y) ? data[1]/2. : -data[1]/2.;
				positionLocal[2] = (p & MAX_Z) ? data[2]/2. : -data[2]/2.;
				position = geomFrame * positionLocal;

				velocity = pBody->GetLinVelocity(positionLocal);

				bool penentrated = _calcPenaltyForce(pBody, position, velocity, force, Ks, Ds, XD(mus[i]));
				if(penentrated)
				{
					bodyIDs.append(bodyID);

					object pyPosition = O_Vec3.copy();
					Vec3_2_pyVec3(position, pyPosition);
					positions.append(pyPosition);

					object pyForce = O_Vec3.copy();
					Vec3_2_pyVec3(force, pyForce);
					forces.append(pyForce);

					object pyPositionLocal = O_Vec3.copy();
					Vec3_2_pyVec3(positionLocal, pyPositionLocal);
					positionLocals.append(pyPositionLocal);
				}
			}
		}
	}
	*/
	return make_tuple(bodyIDs, positions, positionLocals, forces);
}

bool VpWorld::_calcPenaltyForce( const vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu )
{
	Vec3 vRelVel, vNormalRelVel, vTangentialRelVel;
	scalar normalRelVel, tangentialRelVel;
	const Vec3 vNormal(0,1,0);
	Vec3 vNormalForce, vFrictionForce;
	scalar normalForce=0., frictionForce=0.;

	vRelVel = velocity;
	normalRelVel = Inner(vRelVel, vNormal);
	vNormalRelVel = normalRelVel * vNormal;
	vTangentialRelVel = vRelVel - vNormalRelVel;
	tangentialRelVel = Norm(vTangentialRelVel);
	//_planeHeight = .0;
	if(position[1] > _planeHeight)
		return false;
	else
	{
		// normal reaction force
		normalForce = Ks*(_planeHeight - position[1]);
//		if(velocity[1]>0.)
		normalForce -= Ds*velocity[1];
		if(normalForce<0.) normalForce = 0.;
		vNormalForce = normalForce * vNormal;
		
		// tangential reaction force
/*
		if(tangentialRelVel > 0.0)
		{
			frictionForce = mu * normalForce;
			vFrictionForce = frictionForce * -Normalize(vTangentialRelVel);
		}
*/
		frictionForce = mu * normalForce;

		// ?????? ????�� ?? ?̲??????? ???? ?????ϱ? ��??
		// rigid body?̹Ƿ? point locking?? ????? ��???????? ?????? ???��?
		// ?̲??????? ?????? ?ӵ??? ?ݴ?????��?? ū ???????? ?ۿ뿡 ??�� step?????? 
		// ?ٽ? ?? ?ݴ?????��?? ???????? ?ۿ??ϸ鼭 ????�� ?ϸ? ?̲??????? ????
		// ?̸? ?????ϱ? ��?? ??�� ?ӵ? ???Ͽ????? ???????? ??�� ??��?? ?????ϵ??? ?ӽ? ?ڵ?
		if(tangentialRelVel < _lockingVel) 
			frictionForce *= tangentialRelVel/_lockingVel;

		vFrictionForce = frictionForce * -Normalize(vTangentialRelVel);

		force = vNormalForce + vFrictionForce;
		return true;
	}
}

void VpWorld::applyPenaltyForce( const bp::list& bodyIDs, const bp::list& positionLocals, const bp::list& forces )
{
	int bodyID;
	vpBody* pBody;
	Vec3 position, force;

	for(int i=0; i<len(bodyIDs); ++i)
	{
		bodyID = XI(bodyIDs[i]);
		pyVec3_2_Vec3(positionLocals[i], position);
		pyVec3_2_Vec3(forces[i], force);

		pBody = _world.GetBody(bodyID);
		pBody->ApplyGlobalForce(force, position);
	}
}


// @return ( bodyIDs, positions, postionLocals)
boost::python::tuple VpWorld::getContactPoints( const bp::list& bodyIDsToCheck)
{
	bp::list bodyIDs, positions, forces, positionLocals, velocities;
	int bodyID;
	ndarray O_Vec3 = np::array(make_tuple(0., 0., 0.));
	const vpBody* pBody;
	vpGeom* pGeom;
	char type;
	scalar data[3];
	Vec3 position, velocity, force, positionLocal;
	scalar planeHeight = .000;

	for (int i = 0; i<len(bodyIDsToCheck); ++i)
	{
		bodyID = XI(bodyIDsToCheck[i]);
		pBody = _world.GetBody(bodyID);

		int numContactGeom = 0;
        bp::list _bodyIDs, _positions, _forces, _positionLocals, _velocities;

		for (int j = 0; j<pBody->GetNumGeometry(); ++j)
		{
			pGeom = pBody->GetGeometry(j);
			pGeom->GetShape(&type, data);
			if (type == 'C')
			{
			    const vector<Vec3>& verticesLocal = type == 'C'? ((MyFoot3*)pGeom)->getVerticesLocal() :
													type == 'D'? ((MyFoot4*)pGeom)->getVerticesLocal() :
																 ((MyFoot5*)pGeom)->getVerticesLocal();
				const vector<Vec3>& verticesGlobal = type == 'C'? ((MyFoot3*)pGeom)->getVerticesGlobal() :
													 type == 'D'? ((MyFoot4*)pGeom)->getVerticesGlobal() :
																  ((MyFoot5*)pGeom)->getVerticesGlobal();
				for (int k = 0; k < verticesLocal.size(); ++k)
				{
					position = verticesGlobal[k];
					positionLocal = verticesLocal[k];
					bool penentrated = position[1] <= planeHeight + LIE_EPS;
					if (penentrated)
					{
					    ++numContactGeom;
						velocity = pBody->GetLinVelocity(positionLocal);

						_bodyIDs.append(bodyID);

						object pyPosition = O_Vec3.copy();
						Vec3_2_pyVec3(position, pyPosition);
						_positions.append(pyPosition);

						object pyVelocity = O_Vec3.copy();
						Vec3_2_pyVec3(velocity, pyVelocity);
						_velocities.append(pyVelocity);

						object pyPositionLocal = O_Vec3.copy();
						Vec3_2_pyVec3(positionLocal, pyPositionLocal);
						_positionLocals.append(pyPositionLocal);
					}
				}
			}
			else if (true)
			{
				const SE3& geomFrame = pGeom->GetGlobalFrame();

				for (int p = 0; p<8; ++p)
				{
					positionLocal[0] = (p & MAX_X) ? data[0] / 2. : -data[0] / 2.;
					positionLocal[1] = (p & MAX_Y) ? data[1] / 2. : -data[1] / 2.;
					positionLocal[2] = (p & MAX_Z) ? data[2] / 2. : -data[2] / 2.;
					position = geomFrame * positionLocal;

					velocity = pBody->GetLinVelocity(positionLocal);

					bool penentrated = position[1] <= planeHeight;
					if (penentrated)
					{
						bodyIDs.append(bodyID);

						object pyPosition = O_Vec3.copy();
						Vec3_2_pyVec3(position, pyPosition);
						positions.append(pyPosition);

						object pyVelocity = O_Vec3.copy();
						Vec3_2_pyVec3(velocity, pyVelocity);
						velocities.append(pyVelocity);

						object pyPositionLocal = O_Vec3.copy();
						Vec3_2_pyVec3(positionLocal, pyPositionLocal);
						positionLocals.append(pyPositionLocal);
					}
				}
			}
		}

//		if(numContactGeom > 2)
		if(true)
		{
		    bodyIDs.extend(_bodyIDs);
		    positions.extend(_positions);
		    positionLocals.extend(_positionLocals);
		    velocities.extend(_velocities);
		}
	}


	return make_tuple(bodyIDs, positions, positionLocals, velocities);
}

/*********************
vpWorld wrapper
*********************/

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
#include <VP/vpDataType.h>
void VpWorld::Initialize()
{
	_world.Initialize();
}

void VpWorld::SetTimeStep(scalar t)
{
	_timeStep = t;
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
