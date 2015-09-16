#include "stdafx.h"

#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"

#include "VPUtil.h"
#include "csVpModel.h"
#include "csVpWorld.h"

#define make_tuple boost::python::make_tuple

#define MAX_X 1	// 0001
#define MAX_Y 2	// 0010
#define MAX_Z 4	// 0100

BOOST_PYTHON_MODULE(csVpWorld)
{
	class_<VpWorld>("VpWorld", init<const object&>())
		.def("step", &VpWorld::step)
		.def("initialize", &VpWorld::initialize)
		.def("calcPenaltyForce", &VpWorld::calcPenaltyForce)
		.def("applyPenaltyForce", &VpWorld::applyPenaltyForce)
		.def("getBodyNum", &VpWorld::getBodyNum)
	;
}


VpWorld::VpWorld(const object& config)
{
	_world.SetTimeStep(XD(config.attr("timeStep")));
	_world.SetGravity(pyVec3_2_Vec3(config.attr("gravity")));

	_planeHeight = XD(config.attr("planeHeight"));
	_lockingVel = XD(config.attr("lockingVel"));

	if(XB(config.attr("useDefaultContactModel")))
	{
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
	//_world.SetIntegrator(VP::IMPLICIT_EULER_FAST);
#ifndef __APPLE__
	setOpenMP();
#endif
}

#ifndef __APPLE__
void VpWorld::setOpenMP()
{
	int a = 1;


	#pragma omp parallel 
		_world.SetNumThreads((a = omp_get_num_threads()));
	std::cout << "parallelized with " << a << " cores" << std::endl;

}
#endif

// @return ( bodyIDs, positions, postionLocals, forces)
boost::python::tuple VpWorld::calcPenaltyForce( const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds )
{
	bp::list bodyIDs, positions, forces, positionLocals;
	int bodyID;
	static numeric::array O_Vec3(make_tuple(0., 0., 0.));
	const vpBody* pBody;
	vpGeom* pGeom;
	char type;
	scalar data[3];
	static Vec3 position, velocity, force, positionLocal;

	for (int i = 0; i<len(bodyIDsToCheck); ++i)
	{
		bodyID = XI(bodyIDsToCheck[i]);
		pBody = _world.GetBody(bodyID);

		for (int j = 0; j<pBody->GetNumGeometry(); ++j)
		{
			pGeom = pBody->GetGeometry(j);
			
			pGeom->GetShape(&type, data);
			if (type == 'C')
			{
				const vector<Vec3>& verticesLocal = pGeom->getVerticesLocal();
				const vector<Vec3>& verticesGlobal = pGeom->getVerticesGlobal();
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

						object pyPositionLocal = O_Vec3.copy();
						Vec3_2_pyVec3(positionLocal, pyPositionLocal);
						positionLocals.append(pyPositionLocal);
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

// @param position 작용할 지점(global)
// @param [out] force 발생한 penalty force(global)
// @return penalty force가 발생했으면 true
bool VpWorld::_calcPenaltyForce( const vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu )
{
	static Vec3 vRelVel, vNormalRelVel, vTangentialRelVel;
	static scalar normalRelVel, tangentialRelVel;
	static const Vec3 vNormal(0,1,0);
	static Vec3 vNormalForce, vFrictionForce;
	scalar normalForce=0., frictionForce=0.;

	vRelVel = velocity;
	normalRelVel = Inner(vRelVel, vNormal);
	vNormalRelVel = normalRelVel * vNormal;
	vTangentialRelVel = vRelVel - vNormalRelVel;
	tangentialRelVel = Norm(vTangentialRelVel);

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

		// 가만히 서있을 때 미끄러지는 현상 방지하기 위해
		// rigid body이므로 point locking이 힘들어서 정지마찰력 구현이 어려움
		// 미끄러지는 원인이 속도의 반대방향으로 큰 마찰력이 작용에 다음 step에서는 
		// 다시 그 반대방향으로 마찰력이 작용하면서 진동을 하며 미끄러지기 때문
		// 이를 방지하기 위해 일정 속도 이하에서는 마찰력의 일정 비율만 적용하도록 임시 코딩
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
	static Vec3 position, force;

	for(int i=0; i<len(bodyIDs); ++i)
	{
		bodyID = XI(bodyIDs[i]);
		pyVec3_2_Vec3(positionLocals[i], position);
		pyVec3_2_Vec3(forces[i], force);

		pBody = _world.GetBody(bodyID);
		pBody->ApplyGlobalForce(force, position);
	}
}