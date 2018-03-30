#include "stdafx.h"

#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

//#define make_tuple boost::python::make_tuple

using boost::python::make_tuple;
namespace bp = boost::python;
namespace np = boost::python::numpy;
using boost::python::numpy::ndarray;

namespace ublas = boost::numeric::ublas;

#include "csVpModel.h"
#include "csVpWorld.h"
#include "myGeom.h"
//#include "hpBJoint.h"
//#include "hpUJoint.h"
//#include "hpRJoint.h"

#define MAX_X 1	// 0001
#define MAX_Y 2	// 0010
#define MAX_Z 4	// 0100

#define QP



BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getJointPositionGlobal_py_overloads, getJointPositionGlobal, 1, 2);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyPositionGlobal_py_overloads, getBodyPositionGlobal_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyVelocityGlobal_py_overloads, getBodyVelocityGlobal_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyAccelerationGlobal_py_overloads, getBodyAccelerationGlobal_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpMotionModel_recordVelByFiniteDiff_overloads, recordVelByFiniteDiff, 0, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_applyBodyGenForceGlobal_overloads, applyBodyGenForceGlobal, 3, 4);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_applyBodyForceGlobal_overloads, applyBodyForceGlobal, 2, 3);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(initializeHybridDynamics_overloads, initializeHybridDynamics, 0, 1);

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_SetJointElasticity_overloads, SetJointElasticity, 2, 4);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_SetJointsElasticity_overloads, SetJointsElasticity, 1, 3);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_SetJointDamping_overloads, SetJointDamping, 2, 4);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_SetJointsDamping_overloads, SetJointsDamping, 1, 3);

BOOST_PYTHON_MODULE(csVpModel)
{
	class_<VpModel>("VpModel", init<VpWorld*, object, object>())
		.def("__str__", &VpModel::__str__)
		.def("getBodyNum", &VpModel::getBodyNum)
		.def("getBodyMasses", &VpModel::getBodyMasses)
		.def("getTotalMass", &VpModel::getTotalMass)
		.def("getBodyVerticesPositionGlobal", &VpModel::getBodyVerticesPositionGlobal)

		.def("getBodyGeomNum", &VpModel::getBodyGeomNum)
		.def("getBodyGeomsType", &VpModel::getBodyGeomsType)
		.def("getBodyGeomsSize", &VpModel::getBodyGeomsSize)
		.def("getBodyGeomsLocalFrame", &VpModel::getBodyGeomsLocalFrame)
		.def("getBodyGeomsGlobalFrame", &VpModel::getBodyGeomsGlobalFrame)
		.def("getBodyShape", &VpModel::getBodyShape)

		.def("getBodyByIndex", &VpModel::getBodyByIndex, return_value_policy<reference_existing_object>())
		.def("getBodyByName", &VpModel::getBodyByName, return_value_policy<reference_existing_object>())
		.def("getJointByIndex", &VpModel::getJointByIndex, return_value_policy<reference_existing_object>())
		.def("getJointByName", &VpModel::getJointByName, return_value_policy<reference_existing_object>())

		.def("index2name", &VpModel::index2name)
		.def("index2id", &VpModel::index2id)
		.def("name2index", &VpModel::name2index)
		.def("name2id", &VpModel::name2id)

		.def("getBodyInertiaLocal", &VpModel::getBodyInertiaLocal_py)
		.def("getBodyInertiaGlobal", &VpModel::getBodyInertiaGlobal_py)

		.def("getBodyInertiasLocal", &VpModel::getBodyInertiasLocal)
		.def("getBodyInertiasGlobal", &VpModel::getBodyInertiasGlobal)
		
		.def("getCOM", &VpModel::getCOM)
        .def("getBoneT", &VpModel::getBoneT)
        .def("getInvBoneT", &VpModel::getInvBoneT)

        .def("getBodyGenVelLocal", &VpModel::getBodyGenVelLocal)
        .def("getBodyGenVelGlobal", &VpModel::getBodyGenVelGlobal)
        .def("getBodyGenAccLocal", &VpModel::getBodyGenAccLocal)
        .def("getBodyGenAccGlobal", &VpModel::getBodyGenAccGlobal)
		.def("getBodyPositionGlobal", &VpModel::getBodyPositionGlobal_py, getBodyPositionGlobal_py_overloads())
		.def("getBodyVelocityGlobal", &VpModel::getBodyVelocityGlobal_py, getBodyVelocityGlobal_py_overloads())
		.def("getBodyAccelerationGlobal", &VpModel::getBodyAccelerationGlobal_py, getBodyAccelerationGlobal_py_overloads())
		.def("getBodyAngVelocityGlobal", &VpModel::getBodyAngVelocityGlobal)
		.def("getBodyAngAccelerationGlobal", &VpModel::getBodyAngAccelerationGlobal)

		.def("getBodyFrame", &VpModel::getBodyFrame)

		.def("getBodyPositionsGlobal", &VpModel::getBodyPositionsGlobal)
		.def("getBodyVelocitiesGlobal", &VpModel::getBodyVelocitiesGlobal)
		.def("getBodyAccelerationsGlobal", &VpModel::getBodyAccelerationsGlobal)
		.def("getBodyAngVelocitiesGlobal", &VpModel::getBodyAngVelocitiesGlobal)
		.def("getBodyAngAccelerationsGlobal", &VpModel::getBodyAngAccelerationsGlobal)
		.def("getBodyOrientationGlobal", &VpModel::getBodyOrientationGlobal)		

		.def("setBodyPositionGlobal", &VpModel::setBodyPositionGlobal_py)
		.def("setBodyVelocityGlobal", &VpModel::setBodyVelocityGlobal_py)
		.def("setBodyAccelerationGlobal", &VpModel::setBodyAccelerationGlobal_py)
		.def("setBodyAngVelocityGlobal", &VpModel::setBodyAngVelocityGlobal)
		.def("setBodyAngAccelerationGlobal", &VpModel::setBodyAngAccelerationGlobal)

		.def("translateByOffset", &VpModel::translateByOffset)
		.def("rotate", &VpModel::rotate)
		.def("SetGround", &VpModel::SetGround)
		.def("SetBodyColor", &VpModel::SetBodyColor)
		.def("id2index", &VpModel::id2index)
		;

	class_<VpMotionModel, bases<VpModel> >("VpMotionModel", init<VpWorld*, object, object>())
		.def("update", &VpMotionModel::update)
		.def("recordVelByFiniteDiff", &VpMotionModel::recordVelByFiniteDiff, VpMotionModel_recordVelByFiniteDiff_overloads())
		;

	class_<VpControlModel, bases<VpModel> >("VpControlModel", init<VpWorld*, object, object>())
		.def("__str__", &VpControlModel::__str__)
		.def("getJointNum", &VpControlModel::getJointNum)
		.def("getInternalJointNum", &VpControlModel::getInternalJointNum)
		.def("getDOFs", &VpControlModel::getDOFs)
		.def("getInternalJointDOFs", &VpControlModel::getInternalJointDOFs)
		.def("getTotalDOF", &VpControlModel::getTotalDOF)
		.def("getTotalInternalJointDOF", &VpControlModel::getTotalInternalJointDOF)

		.def("getJointDOFIndexes", &VpControlModel::getJointDOFIndexes)

		.def("update", &VpControlModel::update)
		.def("fixBody", &VpControlModel::fixBody)

		.def("initializeHybridDynamics", &VpControlModel::initializeHybridDynamics, initializeHybridDynamics_overloads())
		.def("initializeForwardDynamics", &VpControlModel::initializeForwardDynamics)
		.def("setHybridDynamics", &VpControlModel::setHybridDynamics)
		.def("solveHybridDynamics", &VpControlModel::solveHybridDynamics)
		.def("solveForwardDynamics", &VpControlModel::solveForwardDynamics)
		.def("solveInverseDynamics", &VpControlModel::solveInverseDynamics)

		.def("get_q", &VpControlModel::get_q)
		.def("get_dq", &VpControlModel::get_dq)
		.def("set_ddq", &VpControlModel::set_ddq)

		.def("getDOFPositions", &VpControlModel::getDOFPositions)
		.def("getDOFVelocities", &VpControlModel::getDOFVelocities)
		.def("getDOFAccelerations", &VpControlModel::getDOFAccelerations)
		.def("getDOFAxeses", &VpControlModel::getDOFAxeses)

		.def("getDOFPositionsLocal", &VpControlModel::getDOFPositionsLocal)
		.def("getDOFVelocitiesLocal", &VpControlModel::getDOFVelocitiesLocal)
		.def("getDOFAccelerationsLocal", &VpControlModel::getDOFAccelerationsLocal)
		.def("getDOFAxesesLocal", &VpControlModel::getDOFAxesesLocal)

		.def("getBodyRootDOFVelocitiesLocal", &VpControlModel::getBodyRootDOFVelocitiesLocal)
		.def("getBodyRootDOFAccelerationsLocal", &VpControlModel::getBodyRootDOFAccelerationsLocal)
		.def("getBodyRootDOFAxeses", &VpControlModel::getBodyRootDOFAxeses)

		.def("setDOFAccelerations", &VpControlModel::setDOFAccelerations)
		.def("setDOFTorques", &VpControlModel::setDOFTorques)

		.def("getJointOrientationLocal", &VpControlModel::getJointOrientationLocal)
		.def("getJointAngVelocityLocal", &VpControlModel::getJointAngVelocityLocal)
		.def("getJointAngAccelerationLocal", &VpControlModel::getJointAngAccelerationLocal)

        .def("getLocalJacobian", &VpControlModel::getLocalJacobian)
        .def("getLocalJointVelocity", &VpControlModel::getLocalJointVelocity)
        .def("getLocalJointDisplacementDerivatives", &VpControlModel::getLocalJointDisplacementDerivatives)


		.def("getJointPositionGlobal", &VpControlModel::getJointPositionGlobal, getJointPositionGlobal_py_overloads())
//		.def("getJointPositionGlobal", &VpControlModel::getJointPositionGlobal)
		.def("getJointVelocityGlobal", &VpControlModel::getJointVelocityGlobal)
		.def("getJointAccelerationGlobal", &VpControlModel::getJointAccelerationGlobal)
		.def("getJointOrientationGlobal", &VpControlModel::getJointOrientationGlobal)
		.def("getJointAngVelocityGlobal", &VpControlModel::getJointAngVelocityGlobal)
		.def("getJointAngAccelerationGlobal", &VpControlModel::getJointAngAccelerationGlobal)

		.def("getJointOrientationsLocal", &VpControlModel::getJointOrientationsLocal)
		.def("getJointAngVelocitiesLocal", &VpControlModel::getJointAngVelocitiesLocal)
		.def("getJointAngAccelerationsLocal", &VpControlModel::getJointAngAccelerationsLocal)

		.def("getJointPositionsGlobal", &VpControlModel::getJointPositionsGlobal)
		.def("getJointVelocitiesGlobal", &VpControlModel::getJointVelocitiesGlobal)
		.def("getJointAccelerationsGlobal", &VpControlModel::getJointAccelerationsGlobal)
		.def("getJointOrientationsGlobal", &VpControlModel::getJointOrientationsGlobal)
		.def("getJointAngVelocitiesGlobal", &VpControlModel::getJointAngVelocitiesGlobal)
		.def("getJointAngAccelerationsGlobal", &VpControlModel::getJointAngAccelerationsGlobal)

		.def("getInternalJointOrientationsLocal", &VpControlModel::getInternalJointOrientationsLocal)
		.def("getInternalJointAngVelocitiesLocal", &VpControlModel::getInternalJointAngVelocitiesLocal)
		.def("getInternalJointAngAccelerationsLocal", &VpControlModel::getInternalJointAngAccelerationsLocal)

		.def("getInternalJointPositionsGlobal", &VpControlModel::getInternalJointPositionsGlobal)
		.def("getInternalJointOrientationsGlobal", &VpControlModel::getInternalJointOrientationsGlobal)

		.def("setJointAngVelocityLocal", &VpControlModel::setJointAngVelocityLocal)
		.def("setJointAngAccelerationLocal", &VpControlModel::setJointAngAccelerationLocal)

		.def("setJointAccelerationGlobal", &VpControlModel::setJointAccelerationGlobal)
		.def("setJointAngAccelerationGlobal", &VpControlModel::setJointAngAccelerationGlobal)

		.def("setJointAngAccelerationsLocal", &VpControlModel::setJointAngAccelerationsLocal)
		
		.def("setInternalJointAngAccelerationsLocal", &VpControlModel::setInternalJointAngAccelerationsLocal)


		.def("applyBodyGenForceGlobal", &VpControlModel::applyBodyGenForceGlobal, VpControlModel_applyBodyGenForceGlobal_overloads())
		.def("applyBodyForceGlobal", &VpControlModel::applyBodyForceGlobal, VpControlModel_applyBodyForceGlobal_overloads())
		.def("applyBodyTorqueGlobal", &VpControlModel::applyBodyTorqueGlobal)

		.def("getBodyForceLocal", &VpControlModel::getBodyForceLocal)
		.def("getBodyNetForceLocal", &VpControlModel::getBodyNetForceLocal)
		.def("getBodyGravityForceLocal", &VpControlModel::getBodyGravityForceLocal)

        .def("SetJointElasticity", &VpControlModel::SetJointElasticity, VpControlModel_SetJointElasticity_overloads())
	    .def("SetJointsElasticity", &VpControlModel::SetJointsElasticity, VpControlModel_SetJointsElasticity_overloads())
	    .def("SetJointDamping", &VpControlModel::SetJointDamping, VpControlModel_SetJointDamping_overloads())
	    .def("SetJointsDamping", &VpControlModel::SetJointsDamping, VpControlModel_SetJointsDamping_overloads())

		.def("getJointTorqueLocal", &VpControlModel::getJointTorqueLocal)
		.def("getInternalJointTorquesLocal", &VpControlModel::getInternalJointTorquesLocal)

		.def("setJointTorqueLocal", &VpControlModel::setJointTorqueLocal)
		.def("setInternalJointTorquesLocal", &VpControlModel::setInternalJointTorquesLocal)
		
		.def("setSpring", &VpControlModel::setSpring)

		.def("getInverseEquationOfMotion", &VpControlModel::getInverseEquationOfMotion)
		.def("getEquationOfMotion", &VpControlModel::getEquationOfMotion)
		.def("stepKinematics", &VpControlModel::stepKinematics)
		;
}

/*
static SE3 skewVec3(const Vec3 &v)
{
	return SE3(0., v[2], -v[1], -v[2], 0., v[0], v[1], -v[0], 0.);
}

static SE3 skewVec3(const Axis &v)
{
	return SE3(0., v[2], -v[1], -v[2], 0., v[0], v[1], -v[0], 0.);
}
//*/
 
VpModel::VpModel( VpWorld* pWorld, const object& createPosture, const object& config ) 
:_pWorld(&pWorld->_world), _config(config), _skeleton(createPosture.attr("skeleton"))
{
	int num = XI(createPosture.attr("skeleton").attr("getJointNum")());
	_nodes.resize(num, NULL);
	_boneTs.resize(num, SE3());

	createBodies(createPosture);
	build_name2index();
}

VpModel::~VpModel()
{
	for( NODES_ITOR it=_nodes.begin(); it!=_nodes.end(); ++it)
		if(*it)
			delete *it;
}

void VpModel::SetGround(int index, bool flag)
{
	_nodes[index]->body.SetGround(flag);
}

void VpModel::createBodies( const object& posture )
{
	object joint = posture.attr("skeleton").attr("root");

	object rootPos = posture.attr("rootPos");
	SE3 T = SE3(pyVec3_2_Vec3(rootPos));

	object tpose = posture.attr("getTPose")();
	_createBody(joint, T, tpose);
}

void VpModel::_createBody( const object& joint, const SE3& parentT, const object& posture )
{
	int len_joint_children = len(joint.attr("children")); 
	if (len_joint_children == 0 )
		return;

	SE3 T = parentT;

	SE3 P = SE3(pyVec3_2_Vec3(joint.attr("offset")));
	T = T * P;

	string joint_name = XS(joint.attr("name"));
	//	int joint_index = XI(posture.attr("skeleton").attr("getElementIndex")(joint_name));
	//	SE3 R = pySO3_2_SE3(posture.attr("getLocalR")(joint_index));
	int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
	SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));
	T = T * R;

//	if (len_joint_children > 0 && _config.attr("hasNode")(joint_name))
	if (_config.attr("hasNode")(joint_name))
	{

		Vec3 offset(0);
		for( int i=0 ; i<len_joint_children; ++i)
			offset += pyVec3_2_Vec3(joint.attr("children")[i].attr("offset"));

		//if (joint.attr("parent") == object())
			offset *= (1./len_joint_children);


		SE3 boneT(offset*.5);


		// if(joint.attr("parent") == object())
		//  	boneT=SE3();

		Vec3 defaultBoneV(0,0,1);
		SE3 boneR = getSE3FromVectors(defaultBoneV, offset);


		// if(joint.attr("parent") != object())
			boneT = boneT * boneR;

		Node* pNode = new Node(joint_name);
		_nodes[joint_index] = pNode;

		object cfgNode = _config.attr("getNode")(joint_name);
		///*
		int numGeom = len(cfgNode.attr("geoms"));
		if (numGeom != 0)
		{
			for (int i=0; i<numGeom; i++)
			{
				string geomType = XS(cfgNode.attr("geoms")[i]);
				if (0 == geomType.compare("MyFoot3") || 0 == geomType.compare("MyFoot4") || 0 == geomType.compare("MyFoot5"))
				{
					scalar density = XD(cfgNode.attr("geomMaterial")[i].attr("density"));
					scalar radius = XD(cfgNode.attr("geomMaterial")[i].attr("radius"));
					scalar height = XD(cfgNode.attr("geomMaterial")[i].attr("height"));

					if (height <= 0.)
					    height = Norm(offset) + 2*radius;

					// vpMaterial *pMaterial = new vpMaterial();
					// pMaterial->SetDensity(density);
					// pNode->body.SetMaterial(pMaterial);
					pNode->material.SetDensity(density);
					pNode->body.SetMaterial(&(pNode->material));

					SE3 geomT;
					geomT.SetEye();
					if(cfgNode.attr("geomTs")[i])
					{
						geomT = pySO3_2_SE3(cfgNode.attr("geomTs")[i][1]);
						geomT.SetPosition(geomT*pyVec3_2_Vec3(cfgNode.attr("geomTs")[i][0]));
					}
					else
						std::cout << "there is no geom Ts!" << std::endl;

					if (0 == geomType.compare("MyFoot3"))
						pNode->body.AddGeometry(new MyFoot3(radius, height), geomT);
					else if(0 == geomType.compare("MyFoot4"))
						pNode->body.AddGeometry(new MyFoot4(radius, height), geomT);
					else
						pNode->body.AddGeometry(new MyFoot5(radius, height), geomT);
				}
				else
				{
					scalar density = XD(cfgNode.attr("geomMaterial")[i].attr("density"));
					scalar width = XD(cfgNode.attr("geomMaterial")[i].attr("width"));
					scalar length = XD(cfgNode.attr("geomMaterial")[i].attr("length"));
					scalar height = XD(cfgNode.attr("geomMaterial")[i].attr("height"));

					// vpMaterial *pMaterial = new vpMaterial();
					// pMaterial->SetDensity(density);
					// pNode->body.SetMaterial(pMaterial);
					pNode->material.SetDensity(density);
					pNode->body.SetMaterial(&(pNode->material));

					SE3 geomT;
					geomT.SetEye();
					if(cfgNode.attr("geomTs")[i])
					{
						geomT = pySO3_2_SE3(cfgNode.attr("geomTs")[i][1]);
						geomT.SetPosition(pyVec3_2_Vec3(cfgNode.attr("geomTs")[i][0]));
					}

					pNode->body.AddGeometry(new vpBox(Vec3(width, height, length)), geomT);
				}
			}
		}
		else
		{
		//*/	
		///*
			string geomType = XS(cfgNode.attr("geom"));
			if (0 == geomType.compare("MyFoot3") || 0 == geomType.compare("MyFoot4") || 0 == geomType.compare("MyFoot5"))
			{
				scalar radius = .05;
				if( cfgNode.attr("width") != object() )
					radius = XD(cfgNode.attr("width"));

				scalar length = Norm(offset) + 2*radius;
				scalar density = XD(cfgNode.attr("density"));
				scalar mass = 1.;
				if( cfgNode.attr("mass") != object() )
				{
					mass = XD(cfgNode.attr("mass"));
					density = mass/ (radius * radius * M_PI * length);
				}
				else
					mass = density * radius * radius * M_PI * length;

				// density = mass/ (width*width*M_PI*(length+width));
				if (0 == geomType.compare("MyFoot3"))
					pNode->body.AddGeometry(new MyFoot3(radius, length));
				else if(0 == geomType.compare("MyFoot4"))
					pNode->body.AddGeometry(new MyFoot4(radius, length));
				else
					pNode->body.AddGeometry(new MyFoot5(radius, length));
				pNode->body.SetInertia(CylinderInertia(density, radius, length));
			}
			else
			{
				scalar length;
				if( cfgNode.attr("length") != object() )
					length = XD(cfgNode.attr("length")) * XD(cfgNode.attr("boneRatio"));
				else
					length = Norm(offset) * XD(cfgNode.attr("boneRatio"));

				scalar density = XD(cfgNode.attr("density"));
				scalar width, height;
				if( cfgNode.attr("width") != object() )
				{
					width = XD(cfgNode.attr("width"));
					if( cfgNode.attr("mass") != object() )
						height = (XD(cfgNode.attr("mass")) / (density * length)) / width;
					else
						height = .1;
				}
				else
				{
					if( cfgNode.attr("mass") != object() )
						width = sqrt( (XD(cfgNode.attr("mass")) / (density * length)) );
					else
						width = .1;
					height = width;
				}
				string geomType = XS(cfgNode.attr("geom"));
				if(0 == geomType.compare("MyBox"))
					pNode->body.AddGeometry(new MyBox(Vec3(width, height, length)));
				else if(0 == geomType.compare("MyFoot1"))
					pNode->body.AddGeometry(new MyFoot1(Vec3(width, height, length)));
				else if(0 == geomType.compare("MyFoot2"))
					pNode->body.AddGeometry(new MyFoot2(Vec3(width, height, length)));
    		// else if(geomType == "MyShin")
    		// 	pNode->body.AddGeometry(new MyShin(Vec3(width, height, length)));
				else
				{
					pNode->body.AddGeometry(new vpBox(Vec3(width, height, length)));
					cout << geomType << " : undefined geom type!" << endl;
				}

            // pNode->body.AddGeometry(new vpBox(Vec3(width, height, length)));
				pNode->body.SetInertia(BoxInertia(density, Vec3(width/2.,height/2.,length/2.)));
			//*/
			}
		}

//		pNode->body.SetInertia(BoxInertia(density, Vec3(width,height,length)));
		//pNode->body.SetInertia(BoxInertia(density, Vec3(width/2.,height/2.,length/2.)));

		boneT = boneT * SE3(pyVec3_2_Vec3(cfgNode.attr("offset")));
        _boneTs[joint_index] = boneT;
		SE3 newT = T * boneT;

//		if (joint.attr("parent") == object())
//			pNode->body.SetFrame(T);
//		else
		pNode->body.SetFrame(newT);

		_id2index[pNode->body.GetID()] = joint_index;
	}

	for( int i=0 ; i<len_joint_children; ++i)
		_createBody(joint.attr("children")[i], T, posture);
}

//int VpModel::getParentIndex( int index )
//{
//	object parent = _skeleton.attr("getParentJointIndex")(index);
//	if(parent==object())
//		return -1;
//	else
//		return XI(parent);
//}

std::string VpModel::__str__()
{
	stringstream ss;
//	ss << "<NODES>" << endl;
//	for(int i=0; i<_nodes.size(); ++i)
//	{
//		ss << "[" << i << "]:";
////		if(_nodes[i]==NULL)
////			ss << "NULL, ";
////		else
//			ss << _nodes[i]->name << ", ";
//	}
//	ss << endl;
//
//	ss << "<BODIES INDEX:(NODE INDEX) NODE NAME>\n";
//	for(int i=0; i<_bodyElementIndexes.size(); ++i)
//		ss << "[" << i << "]:(" << _bodyElementIndexes[i] << ") " << _nodes[_bodyElementIndexes[i]]->name << ", ";
//	ss << endl;

	ss << "<BODIES (,JOINTS)>" << endl;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ss << "[" << i << "]:" << _nodes[i]->name << ", ";
	ss << endl;

	ss << "<BODY MASSES>" << endl;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
//		if(_nodes[i])
			ss << "[" << i << "]:" << _nodes[i]->body.GetInertia().GetMass() << ", ";
	ss << endl;

//	ss << "<BODY INERTIAS>" << endl;
//	ss << "I11 I22 I33 I12 I13 I23 offset.x offset.y offset.z mass" << endl;
//	for(int i=0; i<_nodes.size(); ++i)
//		if(_nodes[i])
//		{
//			ss << "[" << i << "]:";
//			for(int j=0; j<10; ++j)
//				ss << _nodes[i]->body.GetInertia()[j] << " ";
//			ss << endl;
//		}
//	ss << endl;

	return ss.str();
}

bp::list VpModel::getBodyMasses()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(_nodes[i]->body.GetInertia().GetMass());
	return ls;
}

scalar VpModel::getTotalMass()
{
	scalar mass = 0.;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		mass += _nodes[i]->body.GetInertia().GetMass();
	return mass;
}

int VpModel::getBodyGeomNum(int index)
{
    return _nodes[index]->body.GetNumGeometry();
}

bp::list VpModel::getBodyGeomsType(int index)
{
	char type;
	scalar data[3];
	bp::list ls;
	for(int i=0; i<_nodes[index]->body.GetNumGeometry(); ++i)
	{
        _nodes[index]->body.GetGeometry(i)->GetShape(&type, data);
		ls.append(type);
	}
	return ls;
}

bp::list VpModel::getBodyGeomsSize(int index)
{
	char type;
	scalar data[3];
	bp::list ls;

	for(int i=0; i<_nodes[index]->body.GetNumGeometry(); ++i)
    {
        ndarray O = np::array(make_tuple(0.,0.,0.));
        _nodes[index]->body.GetGeometry(i)->GetShape(&type, data);
        object pyV = O.copy();
        pyV[0] = data[0];
        pyV[1] = data[1];
        pyV[2] = data[2];
        ls.append(pyV);
    }

	return ls;
}

bp::list VpModel::getBodyGeomsLocalFrame(int index)
{
	bp::list ls;

	for(int i=0; i<_nodes[index]->body.GetNumGeometry(); ++i)
    {
        ndarray O = np::array(make_tuple(
                            make_tuple(0.,0.,0.,0.),
                            make_tuple(0.,0.,0.,0.),
                            make_tuple(0.,0.,0.,0.),
                            make_tuple(0.,0.,0.,1.)
                            )
                        );
        const SE3& geomFrame = _nodes[index]->body.GetGeometry(i)->GetLocalFrame();
        object pyT = O.copy();

        for(int j=0; j<12; j++)
            pyT[make_tuple(j%3, j/3)] = geomFrame[j];

        ls.append(pyT);
    }

	return ls;
}

bp::list VpModel::getBodyGeomsGlobalFrame(int index)
{
	bp::list ls;

	for(int i=0; i<_nodes[index]->body.GetNumGeometry(); ++i)
    {
        ndarray O = np::array(make_tuple(
                            make_tuple(0.,0.,0.,0.),
                            make_tuple(0.,0.,0.,0.),
                            make_tuple(0.,0.,0.,0.),
                            make_tuple(0.,0.,0.,1.)
                            )
                        );
        const SE3& geomFrame = _nodes[index]->body.GetGeometry(i)->GetGlobalFrame();
        object pyT = O.copy();

        for(int j=0; j<12; j++)
            pyT[make_tuple(j%3, j/3)] = geomFrame[j];

        ls.append(pyT);
    }

	return ls;
}


object VpModel::getBodyShape(int index)
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	char type;
	scalar data[3];

	_nodes[index]->body.GetGeometry(0)->GetShape(&type, data);

	object pyV = O.copy();
	pyV[0] = data[0];
	pyV[1] = data[1];
	pyV[2] = data[2];

	return pyV;
}

bp::list VpModel::getBodyVerticesPositionGlobal(int index)
{
	ndarray O = np::array(make_tuple(0.,0.,0.));

	const vpGeom *pGeom;
	char type;
	scalar data[3];

	bp::list ls_point;

	pGeom = _nodes[index]->body.GetGeometry(0);
	pGeom->GetShape(&type, data);
	const SE3& geomFrame = pGeom->GetGlobalFrame();

	Vec3 point;
	for( int p=0; p<8; ++p)
	{
		point[0] = (p & MAX_X) ? data[0]/2. : -data[0]/2.;
		point[1] = (p & MAX_Y) ? data[1]/2. : -data[1]/2.;
		point[2] = (p & MAX_Z) ? data[2]/2. : -data[2]/2.;
		point = geomFrame * point;

		object pyV = O.copy();
		Vec3_2_pyVec3(point, pyV);
		ls_point.append(pyV);
	}
	return ls_point;
}

//bp::list VpModel::getBodyPoints()
//{
//	bp::list ls;
//	char type;
//	scalar data[3];
//
//	const vpGeom *pGeom;
//
//	for(int i=0; i<_nodes.size(); ++i)
//		if(_nodes[i])
//		{
//			bp::list ls_point;
//			for( int j=0; j<_nodes[i]->body.GetNumGeometry(); ++j)
//			{
//				pGeom = _nodes[i]->body.GetGeometry(j);
//				pGeom->GetShape(&type, data);
//				const SE3& geomFrame = pGeom->GetGlobalFrame();
//
//				Vec3 point;
//				for( int p=0; p<8; ++p)
//				{
//					point[0] = (p & MAX_X) ? data[0]/2. : -data[0]/2.;
//					point[1] = (p & MAX_Y) ? data[1]/2. : -data[1]/2.;
//					point[2] = (p & MAX_Z) ? data[2]/2. : -data[2]/2.;
//					point = geomFrame * point;
//
//					object pyV = _O_Vec3->copy();
//					Vec3_2_pyVec3(point, pyV);
//					ls_point.append(pyV);
//				}
//			}
//			ls.append(ls_point);
//		}
//	return ls;
//}

//bp::list VpModel::getBodyShapes()
//{
//	bp::list ls;
//	char type;
//	scalar data[3];
//
//	for(int i=0; i<_nodes.size(); ++i)
//		if(_nodes[i])
//		{
//			bp::list ls_geom;
//			for( int j=0; j<_nodes[i]->body.GetNumGeometry(); ++j)
//			{
//				_nodes[i]->body.GetGeometry(j)->GetShape(&type, data);
//				ls_geom.append(make_tuple(data[0], data[1], data[2]));
//			}
//			ls.append(ls_geom);
//		}
//	return ls;
//}

void VpModel::getBodyInertiaLocal(int index, SE3& Tin)
{
//	if(!_nodes[index]) return;

//	ss << "I11 I22 I33 I12 I13 I23 offset.x offset.y offset.z mass" << endl;

//	pyIn[make_tuple(0,0)] = in[0];
//	pyIn[make_tuple(1,1)] = in[1];
//	pyIn[make_tuple(2,2)] = in[2];
//	pyIn[make_tuple(0,1)] = pyIn[make_tuple(1,0)] = in[3];
//	pyIn[make_tuple(0,2)] = pyIn[make_tuple(2,0)] = in[4];
//	pyIn[make_tuple(1,2)] = pyIn[make_tuple(2,1)] = in[5];
	
//		| T[0]	T[3]	T[6]	T[ 9] |
//		| T[1]	T[4]	T[7]	T[10] |
//		| T[2]	T[5]	T[8]	T[11] |

	const Inertia& in = _nodes[index]->body.GetInertia();

	Tin[0] = in[0];
	Tin[4] = in[1];
	Tin[8] = in[2];
	Tin[3] = Tin[1] = in[3];
	Tin[6] = Tin[2] = in[4];
	Tin[7] = Tin[5] = in[5];
}

boost::python::object VpModel::getBodyInertiaLocal_py( int index )
{
	ndarray I = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	SE3 Tin;
	object pyIn = I.copy();

	getBodyInertiaLocal(index, Tin);
	SE3_2_pySO3(Tin, pyIn);
	return pyIn;
}

boost::python::object VpModel::getBodyInertiaGlobal_py( int index )
{
	ndarray I = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	SE3 Tin_local, bodyFrame;
	object pyIn = I.copy();

	getBodyInertiaLocal(index, Tin_local);
	bodyFrame = _nodes[index]->body.GetFrame();
	SE3_2_pySO3(bodyFrame * Tin_local * Inv(bodyFrame), pyIn);
	return pyIn;
	
}

bp::list VpModel::getBodyInertiasLocal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getBodyInertiaLocal_py(i));
	return ls;
}

bp::list VpModel::getBodyInertiasGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getBodyInertiaGlobal_py(i));
	return ls;
}

object VpModel::getCOM()
{
	object pyV;
	make_pyVec3(pyV);
	Vec3 com(0., 0., 0.);
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		com += _nodes[i]->body.GetInertia().GetMass() * _nodes[i]->body.GetFrame().GetPosition();
	com *= 1./getTotalMass();

	Vec3_2_pyVec3(com, pyV);
	return pyV;

}

bp::list VpModel::getBoneT(int index)
{
	bp::list ls;
	ndarray I = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	ndarray O = np::array(make_tuple(0.,0.,0.));

	SE3_2_pySO3(_boneTs[index], I);
	Vec3_2_pyVec3(_boneTs[index].GetPosition(), O);

	ls.append(I);
	ls.append(O);

	return ls;
}
bp::list VpModel::getInvBoneT(int index)
{
	bp::list ls;
	ndarray I = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	ndarray O = np::array(make_tuple(0.,0.,0.));

	SE3 invBoneT = Inv(_boneTs[index]);

	SE3_2_pySO3(invBoneT, I);
	Vec3_2_pyVec3(invBoneT.GetPosition(), O);

	ls.append(I);
	ls.append(O);

	return ls;
}
object VpModel::getBodyGenVelLocal(int index)
{
	ndarray O = np::array( make_tuple(0., 0., 0., 0.,0.,0.) );
	object pyV = O.copy();
	se3_2_pyVec6(_nodes[index]->body.GetGenVelocityLocal(), pyV);
	return pyV;
}
object VpModel::getBodyGenVelGlobal(int index)
{
	ndarray O = np::array( make_tuple(0., 0., 0., 0.,0.,0.) );
	object pyV = O.copy();
	se3_2_pyVec6(_nodes[index]->body.GetGenVelocity(), pyV);
	return pyV;
}

object VpModel::getBodyGenAccLocal(int index)
{
	ndarray O = np::array( make_tuple(0., 0., 0., 0.,0.,0.) );
	object pyV = O.copy();
	se3_2_pyVec6(_nodes[index]->body.GetGenAccelerationLocal(), pyV);
	return pyV;
}
object VpModel::getBodyGenAccGlobal(int index)
{
	ndarray O = np::array( make_tuple(0., 0., 0., 0.,0.,0.) );
	object pyV = O.copy();
	se3_2_pyVec6(_nodes[index]->body.GetGenAcceleration(), pyV);
	return pyV;
}

object VpModel::getBodyPositionGlobal_py( int index, const object& positionLocal/*=object() */ )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 positionLocal_;

	if(positionLocal==object())
		Vec3_2_pyVec3(getBodyPositionGlobal(index), pyV);
	else
	{
		pyVec3_2_Vec3(positionLocal, positionLocal_);
		Vec3_2_pyVec3(getBodyPositionGlobal(index, &positionLocal_), pyV);
	}
	return pyV;
}
object VpModel::getBodyVelocityGlobal_py( int index, const object& positionLocal/*=object() */ )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 positionLocal_;
	
	if(positionLocal==object())
		Vec3_2_pyVec3(getBodyVelocityGlobal(index), pyV);
	else
	{
		pyVec3_2_Vec3(positionLocal, positionLocal_);
		Vec3_2_pyVec3(getBodyVelocityGlobal(index, positionLocal_), pyV);
	}
	return pyV;
}

bp::list VpModel::getBodyVelocitiesGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getBodyVelocityGlobal_py(i));
	return ls;
}

object VpModel::getBodyAngVelocityGlobal( int index )
{
	se3 genVel;
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genVel = _nodes[index]->body.GetGenVelocity();
	pyV[0] = genVel[0];
	pyV[1] = genVel[1];
	pyV[2] = genVel[2];
	return pyV;
}

bp::list VpModel::getBodyAngVelocitiesGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getBodyAngVelocityGlobal(i));
	return ls;
}

object VpModel::getBodyAccelerationGlobal_py(int index, const object& positionLocal )
{
	se3 genAcc;
	Vec3 positionLocal_;
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	if(positionLocal==object())
		Vec3_2_pyVec3(getBodyAccelerationGlobal(index), pyV);
	else
	{
		pyVec3_2_Vec3(positionLocal, positionLocal_);
		Vec3_2_pyVec3(getBodyAccelerationGlobal(index, &positionLocal_), pyV);
	}
	return pyV;
}

object VpModel::getBodyOrientationGlobal(int index)
{
	ndarray I = np::array(make_tuple(make_tuple(1., 0., 0.), make_tuple(0., 1., 0.), make_tuple(0., 0., 1.)));
	SE3 bodyFrame;
	object pyR = I.copy();

	bodyFrame = _nodes[index]->body.GetFrame();
	SE3_2_pySO3(bodyFrame, pyR);
	return pyR;
}

bp::list VpModel::getBodyAccelerationsGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getBodyAccelerationGlobal_py(i));
	return ls;
}

void VpModel::setBodyPositionGlobal_py( int index, const object& pos )
{
	Vec3 position;

	pyVec3_2_Vec3(pos, position);
	setBodyPositionGlobal(index, position); 
}

object VpModel::getBodyFrame(int index)
{
    object pyT;
    make_pySE3(pyT);
	SE3 bodyFrame = _nodes[index]->body.GetFrame();
    SE3_2_pySE3(bodyFrame, pyT);
	return pyT;
}

void VpModel::setBodyVelocityGlobal_py( int index, const object& vel )
{
	se3 genVel;
	genVel = _nodes[index]->body.GetGenVelocity();
	genVel[3] = XD(vel[0]);
	genVel[4] = XD(vel[1]);
	genVel[5] = XD(vel[2]);
	_nodes[index]->body.SetGenVelocity(genVel);
}

void VpModel::setBodyAccelerationGlobal_py( int index, const object& acc )
{
	se3 genAcc;
	genAcc = _nodes[index]->body.GetGenAcceleration();
	genAcc[3] = XD(acc[0]);
	genAcc[4] = XD(acc[1]);
	genAcc[5] = XD(acc[2]);
	_nodes[index]->body.SetGenAcceleration(genAcc);
}

void VpModel::setBodyAngVelocityGlobal( int index, const object& angvel )
{
	se3 genVel;
	genVel = _nodes[index]->body.GetGenVelocity();
	genVel[0] = XD(angvel[0]);
	genVel[1] = XD(angvel[1]);
	genVel[2] = XD(angvel[2]);
	_nodes[index]->body.SetGenVelocity(genVel);
}

void VpModel::setBodyAngAccelerationGlobal( int index, const object& angacc )
{
	se3 genAcc;
	genAcc = _nodes[index]->body.GetGenAcceleration();
	genAcc[0] = XD(angacc[0]);
	genAcc[1] = XD(angacc[1]);
	genAcc[2] = XD(angacc[2]);
	_nodes[index]->body.SetGenAcceleration(genAcc);
}

bp::list VpModel::getBodyPositionsGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getBodyPositionGlobal_py(i));
	return ls;
}

object VpModel::getBodyAngAccelerationGlobal( int index )
{
	se3 genAcc;
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genAcc = _nodes[index]->body.GetGenAcceleration();
	pyV[0] = genAcc[0];
	pyV[1] = genAcc[1];
	pyV[2] = genAcc[2];
	return pyV;
}

bp::list VpModel::getBodyAngAccelerationsGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getBodyAngAccelerationGlobal(i));
	return ls;
}

void VpModel::translateByOffset( const object& offset )
{
	Vec3 v;
	pyVec3_2_Vec3(offset, v);

	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		setBodyPositionGlobal(i, getBodyPositionGlobal(i) + v);
}

void VpModel::rotate( const object& rotation )
{
	SE3 R, bodyFrame;
	pySO3_2_SE3(rotation, R);

	bodyFrame = _nodes[0]->body.GetFrame();
	_nodes[0]->body.SetFrame(bodyFrame * R);

	// �ٲ� root body frame�� ���� joint�� ����� ������ body�� frame ������Ʈ. �̰��� ���� ������ root body �ϳ��� rotation�� ����ȴ�. 
	_pWorld->UpdateFrame();
}

void VpModel::ignoreCollisionWith( vpBody* pBody )
{
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		_pWorld->IgnoreCollision( &_nodes[i]->body, pBody );
}

void VpModel::ignoreCollisionWith_py( vpBody* pBody )
{
	ignoreCollisionWith(pBody);
}

Vec3 VpModel::getBodyPositionGlobal( int index, const Vec3* pPositionLocal )
{
	SE3 bodyFrame;
	bodyFrame = _nodes[index]->body.GetFrame();
	if(!pPositionLocal)
		return bodyFrame.GetPosition();
	else
		return bodyFrame * (*pPositionLocal);
}
Vec3 VpModel::getBodyVelocityGlobal( int index, const Vec3& positionLocal)
{
	return _nodes[index]->body.GetLinVelocity(positionLocal);

//	static se3 genAccLocal, genAccGlobal;
//	genAccLocal = _nodes[index]->body.GetGenVelocityLocal();
//	genAccLocal = MinusLinearAd(positionLocal, genAccLocal);
//	genAccGlobal = Rotate(_nodes[index]->body.GetFrame(), genAccLocal);
//	return Vec3(genAccGlobal[3], genAccGlobal[4], genAccGlobal[5]);
}

Vec3 VpModel::getBodyAccelerationGlobal( int index, const Vec3* pPositionLocal)
{
	se3 genAccLocal, genAccGlobal;

	genAccLocal = _nodes[index]->body.GetGenAccelerationLocal();
	if(pPositionLocal)
		genAccLocal = MinusLinearAd(*pPositionLocal, genAccLocal);
 
	genAccGlobal = Rotate(_nodes[index]->body.GetFrame(), genAccLocal);

	return Vec3(genAccGlobal[3], genAccGlobal[4], genAccGlobal[5]);
}

void VpModel::setBodyPositionGlobal( int index, const Vec3& position )
{
	SE3 bodyFrame;
	bodyFrame = _nodes[index]->body.GetFrame();
	bodyFrame.SetPosition(position);
	_nodes[index]->body.SetFrame(bodyFrame);
}

void VpModel::setBodyAccelerationGlobal( int index, const Vec3& acc, const Vec3* pPositionLocal)
{
//	if(pPositionLocal)
//		cout << "pPositionLocal : not implemented functionality yet" << endl;

	se3 genAcc;
	genAcc = _nodes[index]->body.GetGenAcceleration();
	genAcc[3] = acc[0];
	genAcc[4] = acc[1];
	genAcc[5] = acc[2];

	_nodes[index]->body.SetGenAcceleration(genAcc);
}

VpMotionModel::VpMotionModel( VpWorld* pWorld, const object& createPosture, const object& config )
	:VpModel(pWorld, createPosture, config), _recordVelByFiniteDiff(false), _inverseMotionTimeStep(30.)
{
	// OdeMotionModel�� node.body.disable()�� VpMotionModel������ pWorld->AddBody()��
	// �� ���ִ� ������ �� ������ �ϵ��� �Ѵ�.

	update(createPosture);
	
//	addBody(false);
}


void VpMotionModel::update( const object& posture)
{
	object joint = posture.attr("skeleton").attr("root");
	object rootPos = posture.attr("rootPos");
	SE3 T = SE3(pyVec3_2_Vec3(rootPos));
	_updateBody(joint, T, posture);
}


void VpMotionModel::_updateBody( const object& joint, const SE3& parentT, const object& posture)
{
	int len_joint_children = len(joint.attr("children")); 
	if (len_joint_children == 0 )
		return;

	SE3 T = parentT;

	SE3 P = SE3(pyVec3_2_Vec3(joint.attr("offset")));
	T = T * P;

	string joint_name = XS(joint.attr("name"));
//	int joint_index = XI(posture.attr("skeleton").attr("getElementIndex")(joint_name));
//	SE3 R = pySO3_2_SE3(posture.attr("getLocalR")(joint_index));
	int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
	SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));
	T = T * R;

//	int len_joint_children = len(joint.attr("children")); 
//	if (len_joint_children > 0 && _config.attr("hasNode")(joint_name))
	if (_config.attr("hasNode")(joint_name))
	{
		SE3 boneT = _boneTs[joint_index];
		SE3 newT = T * boneT;

		Node* pNode = _nodes[joint_index];

		if(_recordVelByFiniteDiff)
		{
			SE3 oldT, diffT;
			oldT = pNode->body.GetFrame();
			diffT = newT * Inv(oldT);

			Vec3 p = newT.GetPosition() - oldT.GetPosition();
			diffT.SetPosition(p);

			pNode->body.SetGenVelocity(Log(diffT) * _inverseMotionTimeStep);
		}

		pNode->body.SetFrame(newT);
	}

	for( int i=0 ; i<len_joint_children; ++i)
		_updateBody(joint.attr("children")[i], T, posture);	
}


VpControlModel::VpControlModel( VpWorld* pWorld, const object& createPosture, const object& config )
	:VpModel(pWorld, createPosture, config)
{
	addBodiesToWorld(createPosture);
	ignoreCollisionBtwnBodies();

	object tpose = createPosture.attr("getTPose")();
	createJoints(tpose);

	update(createPosture);

    m_total_dof = 0;
	int dof_start_index = 0;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); i++)
	{
	    _nodes[i]->dof_start_index = dof_start_index;
	    dof_start_index += _nodes[i]->dof;
	    m_total_dof += _nodes[i]->dof;
	}


//	addBody(true);
}

std::string VpControlModel::__str__()
{
	string s1 = VpModel::__str__();

	stringstream ss;

	ss << "<INTERNAL JOINTS>" << endl;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		ss << "[" << i-1 << "]:" << _nodes[i]->name << ", ";
	ss << endl;

	return s1 + ss.str();
}

bp::list VpControlModel::getInternalJointDOFs()
{
	bp::list ls;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		ls.append(3);
	return ls;
}

int VpControlModel::getTotalInternalJointDOF()
{
	int dof = 0;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		dof += _nodes[i]->dof;
		// dof += 3;
	return dof;
}

bp::list VpControlModel::getDOFs()
{
	bp::list ls;
	ls.append(6);
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		ls.append(3);
	return ls;
}

int VpControlModel::getTotalDOF()
{
	int dof = 0;
	dof += 6;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		dof += _nodes[i]->dof;
		// dof += 3;
	return dof;
}

bp::list VpControlModel::getJointDOFIndexes(int index)
{
    bp::list ls;
    int dof_index = _nodes[index]->dof_start_index;
    for(int i=0; i<_nodes[index]->dof; i++)
        ls.append(dof_index++);
    return ls;

}

void VpControlModel::createJoints( const object& posture )
{
	object joint = posture.attr("skeleton").attr("root");
	std::vector<int> empty;
	_createJoint(joint, posture, empty);
}

void VpControlModel::_createJoint( const object& joint, const object& posture, const std::vector<int> &parent_ancestors)
{
	int len_joint_children = len(joint.attr("children"));
	if (len_joint_children == 0 )
		return;

	SE3 invLocalT;

	object offset = joint.attr("offset");
	SE3 P = SE3(pyVec3_2_Vec3(joint.attr("offset")));

	string joint_name = XS(joint.attr("name"));
//	int joint_index = XI(posture.attr("skeleton").attr("getElementIndex")(joint_name));
//	SE3 R = pySO3_2_SE3(posture.attr("getLocalR")(joint_index));
	int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
	SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));

	// parent      <--------->        child
	// link     L1      L2      L3      L4
	// L4_M =  P1*R1 * P2*R2 * P3*R3 * P4*R4  (forward kinematics matrix of L4)
	// ���� ��Ÿ�������� ���⿡�� while loop�� ��� ������ back tracking�� �����
	// L4_M = Inv( Inv(R4)*Inv(P4) * Inv(R3)*Inv(P3) * ...)
	// ���� �ڵ�����.

	invLocalT = invLocalT * Inv(R);
	invLocalT = invLocalT * Inv(P);

	object temp_joint = joint;
	object nodeExistParentJoint = object();
	string temp_parent_name;
	int temp_parent_index;
	while(true)
	{
		if(temp_joint.attr("parent") == object())
		{
			nodeExistParentJoint = object();
			break;
		}
		else
		{
			temp_parent_name = XS(temp_joint.attr("parent").attr("name"));
//			temp_parent_index = XI(posture.attr("skeleton").attr("getElementIndex")(temp_parent_name));
			temp_parent_index = XI(posture.attr("skeleton").attr("getJointIndex")(temp_parent_name));

			if(_nodes[temp_parent_index] != NULL)
			{
				nodeExistParentJoint = temp_joint.attr("parent");
				break;
			}
			else
			{
				temp_joint = temp_joint.attr("parent");

				object offset = temp_joint.attr("offset");
				SE3 P = SE3(pyVec3_2_Vec3(offset));

				string joint_name = XS(temp_joint.attr("name"));
				object localSO3 = posture.attr("localRs")[joint_index];
				SE3 R = pySO3_2_SE3(localSO3);

				invLocalT = invLocalT * Inv(R);
				invLocalT = invLocalT * Inv(P);
			}
		}
	}

//	int len_joint_children = len(joint.attr("children"));

//	if ( nodeExistParentJoint!=object() && len_joint_children > 0  &&
//		_config.attr("hasNode")(joint_name))
	if ( nodeExistParentJoint!=object() && _config.attr("hasNode")(joint_name))
	{
		Node* pNode = _nodes[joint_index];
		object cfgNode = _config.attr("getNode")(joint_name);

		string parent_name = XS(nodeExistParentJoint.attr("name"));
//		int parent_index = XI(posture.attr("skeleton").attr("getElementIndex")(parent_name));
		int parent_index = XI(posture.attr("skeleton").attr("getJointIndex")(parent_name));
		Node* pParentNode = _nodes[parent_index];
		object parentCfgNode = _config.attr("getNode")(parent_name);

		//object offset = cfgNode.attr("offset");
		//SE3 offsetT = SE3(pyVec3_2_Vec3(offset));

		//object parentOffset = parentCfgNode.attr("offset");
		//SE3 parentOffsetT = SE3(pyVec3_2_Vec3(parentOffset));

		pParentNode->body.SetJoint(&pNode->joint, Inv(_boneTs[parent_index])*Inv(invLocalT));
		pNode->body.SetJoint(&pNode->joint, Inv(_boneTs[joint_index]));

		scalar kt = 16.;
		scalar dt = 8.;
		SpatialSpring el(kt);
		SpatialDamper dam(dt);
		//std::cout << el <<std::endl;
		// pNode->joint.SetElasticity(el);
		// pNode->joint.SetDamping(dam);
		pNode->use_joint = true;
	}
	for (std::vector<int>::size_type i=0; i<parent_ancestors.size(); i++)
	    _nodes[joint_index]->ancestors.push_back(parent_ancestors[i]);
    _nodes[joint_index]->ancestors.push_back(joint_index);

	for( int i=0 ; i<len_joint_children; ++i)
		_createJoint(joint.attr("children")[i], posture, _nodes[joint_index]->ancestors);
}

void VpControlModel::ignoreCollisionBtwnBodies()
{
	for( VpModel::NODES_ITOR it=_nodes.begin(); it!=_nodes.end(); ++it)
	{
		for( VpModel::NODES_ITOR it2=_nodes.begin(); it2!=_nodes.end(); ++it2)
		{
			Node* pNode0 = *it;
			Node* pNode1 = *it2;
//			if(pNode0 && pNode1)
				_pWorld->IgnoreCollision(&pNode0->body, &pNode1->body);
		}
	}	
}

void VpControlModel::addBodiesToWorld( const object& createPosture )
{
//	object joint = createPosture.attr("skeleton").attr("root");
//	string root_name = XS(joint.attr("name"));
//	int root_index = XI(createPosture.attr("skeleton").attr("getElementIndex")(root_name));
//	vpBody* pRootBody = &_nodes[root_index]->body;
	vpBody* pRootBody = &_nodes[0]->body;
	_pWorld->AddBody(pRootBody);
}

void VpControlModel::update( const object& posture )
{
	object joint = posture.attr("skeleton").attr("root");
	_updateJoint(joint, posture);
}

void VpControlModel::_updateJoint( const object& joint, const object& posture )
{
	int len_joint_children = len(joint.attr("children")); 
	if (len_joint_children == 0 )
		return;

	SE3 invLocalT;

	SE3 P = SE3(pyVec3_2_Vec3(joint.attr("offset")));

	string joint_name = XS(joint.attr("name"));
//	int joint_index = XI(posture.attr("skeleton").attr("getElementIndex")(joint_name));
//	SE3 R = pySO3_2_SE3(posture.attr("getLocalR")(joint_index));
	int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
	SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));

	// parent      <--------->        child
	// link     L1      L2      L3      L4
	// L4_M =  P1*R1 * P2*R2 * P3*R3 * P4*R4  (forward kinematics matrix of L4)
	// ���� ��Ÿ�������� ���⿡�� while loop�� ��� ������ back tracking�� �����
	// L4_M = Inv( Inv(R4)*Inv(P4) * Inv(R3)*Inv(P3) * ...)
	// ���� �ڵ�����.

	invLocalT = invLocalT * Inv(R);
	invLocalT = invLocalT * Inv(P);

	object temp_joint = joint;
	object nodeExistParentJoint = object();
	string temp_parent_name;
	int temp_parent_index;
	while(true)
	{
		if(temp_joint.attr("parent") == object())
		{
			nodeExistParentJoint = object();
			break;
		}
		else
		{
			temp_parent_name = XS(temp_joint.attr("parent").attr("name"));
//			temp_parent_index = XI(posture.attr("skeleton").attr("getElementIndex")(temp_parent_name));
			temp_parent_index = XI(posture.attr("skeleton").attr("getJointIndex")(temp_parent_name));

			if(_nodes[temp_parent_index] != NULL) 
			{
				nodeExistParentJoint = temp_joint.attr("parent");
				break;
			}
			else
			{
				temp_joint = temp_joint.attr("parent");

				object offset = temp_joint.attr("offset");
				SE3 P = SE3(pyVec3_2_Vec3(offset));

				string joint_name = XS(temp_joint.attr("name"));
				object localSO3 = posture.attr("localRs")[joint_index];
				SE3 R = pySO3_2_SE3(localSO3);

				invLocalT = invLocalT * Inv(R);
				invLocalT = invLocalT * Inv(P);
			}
		}
	}

//	int len_joint_children = len(joint.attr("children")); 

//	if(len_joint_children > 0 && _config.attr("hasNode")(joint_name))
	if(_config.attr("hasNode")(joint_name))
	{
		Node* pNode = _nodes[joint_index];

		if(nodeExistParentJoint!=object())
			pNode->joint.SetOrientation(R);
		else
			// root�� ���� body�� ���� SetFrame() ���ش�.
			pNode->body.SetFrame(SE3(pyVec3_2_Vec3(posture.attr("rootPos")))*P*R*_boneTs[joint_index]);
	}

	for( int i=0 ; i<len_joint_children; ++i)
		_updateJoint(joint.attr("children")[i], posture);
}

void VpControlModel::fixBody( int index )
{
	_nodes[index]->body.SetGround();
}

void VpControlModel::initializeHybridDynamics(bool floatingBase)
{
	std::vector<int>::size_type rootIndex = 0;
	
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
	{
		if(i == rootIndex)
		{
			if(floatingBase)
				_nodes[i]->body.SetHybridDynamicsType(VP::DYNAMIC);
			else
				_nodes[i]->body.SetHybridDynamicsType(VP::KINEMATIC);
		}
		else
			_nodes[i]->joint.SetHybridDynamicsType(VP::KINEMATIC);
	}
}

void VpControlModel::initializeForwardDynamics()
{
    std::vector<int>::size_type rootIndex = 0;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
	{
        _nodes[i]->body.SetHybridDynamicsType(VP::DYNAMIC);
	    if(i != rootIndex)
            _nodes[i]->joint.SetHybridDynamicsType(VP::DYNAMIC);
    }
}

void VpControlModel::setHybridDynamics(int jointIndex, std::string dynamicsType)
{
    if(dynamicsType == "DYNAMIC")
    {
        _nodes[jointIndex]->body.SetHybridDynamicsType(VP::DYNAMIC);
        _nodes[jointIndex]->joint.SetHybridDynamicsType(VP::DYNAMIC);
    }
    else if(dynamicsType == "KINEMATIC")
        _nodes[jointIndex]->joint.SetHybridDynamicsType(VP::KINEMATIC);
}

void VpControlModel::solveHybridDynamics()
{
	_nodes[0]->body.GetSystem()->HybridDynamics();	
}

void VpControlModel::solveForwardDynamics()
{
	_nodes[0]->body.GetSystem()->ForwardDynamics();
}

void VpControlModel::solveInverseDynamics()
{
	_nodes[0]->body.GetSystem()->InverseDynamics();
}

static Axis GetDq(const SE3& R, const Vec3& V)
{
    Axis m_rDq;
    Axis m_rQ = LogR(R);
	Axis W(V[0], V[1], V[2]);
	scalar t = Norm(m_rQ), delta, zeta, t2 = t * t;

	if ( t < BJOINT_EPS )
	{
		delta  = SCALAR_1_12 + SCALAR_1_720 * t2;
		zeta = SCALAR_1 - SCALAR_1_12 * t2;
	} else
	{
		zeta = SCALAR_1_2 * t * (SCALAR_1 + cos(t)) / sin(t);
		delta = (SCALAR_1 - zeta) / t2;
	}

	return (delta * Inner(m_rQ, V)) * m_rQ + zeta * W + SCALAR_1_2 * Cross(m_rQ, W);

}

bp::list VpControlModel::get_q()
{
    // Angular First for root joint
    bp::list ls;
	SE3 rootBodyFrame = _nodes[0]->body.GetFrame();
	SE3 rootJointFrame = rootBodyFrame * Inv(_boneTs[0]);
    Vec3 rootJointPos = rootJointFrame.GetPosition();
    Axis rootJointQ = LogR(rootJointFrame);
    Axis jointQ;

    for(int i=0; i<3; i++)
    {
        ls.append(rootJointQ[i]);
    }
    for(int i=0; i<3; i++)
    {
        ls.append(rootJointPos[i]);
    }
	for(std::vector<int>::size_type j=1; j<_nodes.size(); j++)
	{
	    jointQ = _nodes[j]->joint.GetDisplacement();
	    for(int i=0; i<3; i++)
	    {
            ls.append(jointQ[i]);
	    }
	}

	return ls;
}

bp::list VpControlModel::get_dq()
{
    // Angular First for root joint
    bp::list ls;
	SE3 rootBodyFrame = _nodes[0]->body.GetFrame();
	SE3 rootJointFrame = rootBodyFrame * Inv(_boneTs[0]);
    Vec3 rootJointPos = rootJointFrame.GetPosition();
    Axis rootJointQ = LogR(rootJointFrame);
    Axis jointDq;

	Vec3 rootJointLinVel = _nodes[0]->body.GetLinVelocity(Inv(_boneTs[0]).GetPosition());

    se3 rootBodyGenVelLocal = _nodes[0]->body.GetGenVelocityLocal();
    Vec3 rootJointAngVelLocal = Rotate(_boneTs[0], Vec3(rootBodyGenVelLocal[0], rootBodyGenVelLocal[1], rootBodyGenVelLocal[2]));
    Axis rootJointDq = GetDq(rootJointFrame, rootJointAngVelLocal);

    for(int i=0; i<3; i++)
    {
        ls.append(rootJointDq[i]);
    }
    for(int i=0; i<3; i++)
    {
        ls.append(rootJointLinVel[i]);
    }
	for(std::vector<int>::size_type j=1; j<_nodes.size(); j++)
	{
	    jointDq = _nodes[j]->joint.GetDisplacementDerivate();
	    for(int i=0; i<3; i++)
	    {
            ls.append(jointDq[i]);
	    }
	}

	return ls;
}

void VpControlModel::set_ddq(const object & ddq)
{
    int ddq_index = 6;
    //TODO:
    for(int i=0; i<3; i++)
    {
    }
    for(int i=0; i<6; i++)
    {
    }
	for(std::vector<int>::size_type j=1; j<_nodes.size(); j++)
	{
        Axis jointDdq(XD(ddq[ddq_index]), XD(ddq[ddq_index+1]), XD(ddq[ddq_index+2]));
        _nodes[j]->joint.SetDdq(jointDdq);
        ddq_index += 3;
	}
}

bp::list VpControlModel::getDOFPositions()
{
//	static ndarray rootFrame( make_tuple(make_tuple(1.,0.,0.,0.), make_tuple(0.,1.,0.,0.), make_tuple(0.,0.,1.,0.), make_tuple(0.,0.,0.,1.)) );
//
//	bp::list ls = getInternalJointOrientationsLocal();
//	SE3_2_pySE3(_nodes[0]->body.GetFrame() * Inv(_boneTs[0]), rootFrame);
//	ls.insert(0, rootFrame );
//	return ls;

	ndarray I = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	ndarray O = np::array(make_tuple(0.,0.,0.));
	SE3 rootFrame;

	object pyR = I.copy();
	object pyV = O.copy();

	bp::list ls = getInternalJointOrientationsLocal();

	rootFrame = _nodes[0]->body.GetFrame() * Inv(_boneTs[0]);

	Vec3_2_pyVec3(rootFrame.GetPosition(), pyV);
	SE3_2_pySO3(rootFrame, pyR);

	ls.insert(0, make_tuple(pyV, pyR));
	return ls;
}

bp::list VpControlModel::getDOFVelocities()
{
	ndarray rootGenVel = np::array(make_tuple(0.,0.,0.,0.,0.,0.));
	
	rootGenVel.slice(0,3) = getJointVelocityGlobal(0);
//	rootGenVel.slice(3,6) = getJointAngVelocityGlobal(0);
	rootGenVel.slice(3,6) = getJointAngVelocityLocal(0);

	bp::list ls = getInternalJointAngVelocitiesLocal();
	ls.insert(0, rootGenVel);

	return ls;
}

bp::list VpControlModel::getDOFAccelerations()
{
	ndarray rootGenAcc = np::array(make_tuple(0.,0.,0.,0.,0.,0.));
	
	rootGenAcc.slice(0,3) = getJointAccelerationGlobal(0);
//	rootGenAcc.slice(3,6) = getJointAngAccelerationGlobal(0);
	rootGenAcc.slice(3,6) = getJointAngAccelerationLocal(0);

	bp::list ls = getInternalJointAngAccelerationsLocal();

	ls.insert(0, rootGenAcc);
	return ls;
}

bp::list VpControlModel::getDOFAxeses()
{
	ndarray rootAxeses = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.),
										make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	//ndarray rootAxesTmp = (ndarray)getJointOrientationGlobal(0);
	ndarray rootAxesTmp = np::array(getJointOrientationGlobal(0));
	ndarray rootAxes = transpose_pySO3(rootAxesTmp);
	rootAxeses[3] = rootAxes[0];
	rootAxeses[4] = rootAxes[1];
	rootAxeses[5] = rootAxes[2];

	bp::list ls = getInternalJointOrientationsGlobal();
	for(int i=0; i<len(ls); ++i)
	{
		//ndarray lsTmp = (ndarray)ls[i];
		ndarray lsTmp = np::array(ls[i]);
		ls[i] = transpose_pySO3(lsTmp);
	}

	ls.insert(0, rootAxeses);
	return ls;
}

bp::list VpControlModel::getDOFPositionsLocal()
{
//	static ndarray rootFrame( make_tuple(make_tuple(1.,0.,0.,0.), make_tuple(0.,1.,0.,0.), make_tuple(0.,0.,1.,0.), make_tuple(0.,0.,0.,1.)) );
//
//	bp::list ls = getInternalJointOrientationsLocal();
//	SE3_2_pySE3(_nodes[0]->body.GetFrame() * Inv(_boneTs[0]), rootFrame);
//	ls.insert(0, rootFrame );
//	return ls;

	ndarray I = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	ndarray O = np::array(make_tuple(0.,0.,0.));
	SE3 rootFrame;

	object pyR = I.copy();
	object pyV = O.copy();

	bp::list ls = getInternalJointOrientationsLocal();

	rootFrame = _nodes[0]->body.GetFrame() * Inv(_boneTs[0]);

	Vec3_2_pyVec3(-Inv(rootFrame).GetPosition(), pyV);
	SE3_2_pySO3(rootFrame, pyR);

	ls.insert(0, make_tuple(pyV, pyR));
	return ls;
}

bp::list VpControlModel::getDOFVelocitiesLocal()
{
	ndarray rootGenVel = np::array(make_tuple(0.,0.,0.,0.,0.,0.));
	
	//rootGenVel.slice(0,3) = getJointVelocityGlobal(0);
	//rootGenVel.slice(3,6) = getJointAngVelocityGlobal(0);
	rootGenVel.slice(0,3) = getJointVelocityLocal(0);
	rootGenVel.slice(3,6) = getJointAngVelocityLocal(0);

	bp::list ls = getInternalJointAngVelocitiesLocal();

	ls.insert(0, rootGenVel);
	return ls;
}

bp::list VpControlModel::getDOFAccelerationsLocal()
{
	ndarray rootGenAcc = np::array(make_tuple(0.,0.,0.,0.,0.,0.));
	
	//rootGenAcc.slice(0,3) = getJointAccelerationGlobal(0);
	//rootGenAcc.slice(3,6) = getJointAngAccelerationGlobal(0);
	rootGenAcc.slice(0,3) = getJointAccelerationLocal(0);
	rootGenAcc.slice(3,6) = getJointAngAccelerationLocal(0);

	bp::list ls = getInternalJointAngAccelerationsLocal();

	ls.insert(0, rootGenAcc);
	return ls;
}

bp::list VpControlModel::getDOFAxesesLocal()
{
	ndarray rootAxeses = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.),
										make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	//ndarray rootAxesTmp = (ndarray)getJointOrientationGlobal(0);
	ndarray rootAxesTmp = np::array(getJointOrientationGlobal(0));
	ndarray rootAxes = transpose_pySO3(rootAxesTmp);
	rootAxeses[0] = rootAxes[0];
	rootAxeses[1] = rootAxes[1];
	rootAxeses[2] = rootAxes[2];
	rootAxeses[3] = rootAxes[0];
	rootAxeses[4] = rootAxes[1];
	rootAxeses[5] = rootAxes[2];

	bp::list ls = getInternalJointOrientationsGlobal();
//	bp::list ls = getInternalJointOrientationsLocal();
	for(int i=0; i<len(ls); ++i)
	{
		//ndarray lsTmp = (ndarray)ls[i];
		ndarray lsTmp = np::array(ls[i]);
		ls[i] = transpose_pySO3(lsTmp);
	}

	ls.insert(0, rootAxeses);
	return ls;
}


bp::list VpControlModel::getBodyRootDOFVelocitiesLocal()
{
	ndarray rootGenVel = np::array(make_tuple(0.,0.,0.,0.,0.,0.));

	rootGenVel.slice(0,3) = getBodyGenVelLocal(0).slice(3,6);
	rootGenVel.slice(3,6) = getBodyGenVelLocal(0).slice(0,3);

	bp::list ls = getInternalJointAngVelocitiesLocal();

	ls.insert(0, rootGenVel);
	return ls;
}

bp::list VpControlModel::getBodyRootDOFAccelerationsLocal()
{
	ndarray rootGenAcc = np::array(make_tuple(0.,0.,0.,0.,0.,0.));

	rootGenAcc.slice(0,3) = getBodyGenAccLocal(0).slice(3,6);
	rootGenAcc.slice(3,6) = getBodyGenAccLocal(0).slice(0,3);

	bp::list ls = getInternalJointAngAccelerationsLocal();

	ls.insert(0, rootGenAcc);
	return ls;
}

bp::list VpControlModel::getBodyRootDOFAxeses()
{
	ndarray rootAxeses = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.),
										make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	//ndarray rootAxesTmp = (ndarray)getBodyOrientationGlobal(0);
	ndarray rootAxesTmp = np::array(getBodyOrientationGlobal(0));
	ndarray rootAxes = transpose_pySO3(rootAxesTmp);
	rootAxeses[0] = rootAxes[0];
	rootAxeses[1] = rootAxes[1];
	rootAxeses[2] = rootAxes[2];
	rootAxeses[3] = rootAxes[0];
	rootAxeses[4] = rootAxes[1];
	rootAxeses[5] = rootAxes[2];

	bp::list ls = getInternalJointOrientationsGlobal();
//	bp::list ls = getInternalJointOrientationsLocal();
	for(int i=0; i<len(ls); ++i)
	{
		//ndarray lsTmp = (ndarray)ls[i];
		ndarray lsTmp = np::array(ls[i]);
		ls[i] = transpose_pySO3(lsTmp);
	}

	ls.insert(0, rootAxeses);
	return ls;
}

void VpControlModel::setDOFAccelerations( const bp::list& dofaccs)
{
	ndarray O = np::array(make_tuple(0.,0.,0.));

	setJointAccelerationGlobal(0, dofaccs[0].slice(0,3));

//	setJointAngAccelerationGlobal(0, dofaccs[0].slice(3,6));
	setJointAngAccelerationLocal(0, dofaccs[0].slice(3,6));

	setInternalJointAngAccelerationsLocal( ((bp::list)dofaccs.slice(1,_)) );
}

void VpControlModel::setDOFTorques(const bp::list& dofTorque)
{
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
	{
	    //std::cout << _nodes[i]->name << std::endl;
	    //std::cout << pyVec3_2_Vec3(dofTorque[i-1]) << std::endl;
		_nodes[i]->joint.SetTorque(pyVec3_2_Vec3(dofTorque[i-1]));
	}
}


boost::python::object VpControlModel::getJointOrientationLocal( int index )
{
	ndarray I = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	if(index == 0)
		return getJointOrientationGlobal(index);
	else
	{
		object pyR = I.copy();
		SE3_2_pySO3(_nodes[index]->joint.GetOrientation(), pyR);
		return pyR;
	}
}

boost::python::object VpControlModel::getJointAngVelocityLocal( int index )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	if(index == 0)
	{
		se3 genVelBodyLocal, genVelJointLocal;

		genVelBodyLocal = _nodes[index]->body.GetGenVelocityLocal();
		genVelJointLocal = InvAd(Inv(_boneTs[index]), genVelBodyLocal);
//		genVelJointLocal = Ad(_boneTs[index], genVelBodyLocal);	// �� ���ΰ� ���� ����
		pyV[0] = genVelJointLocal[0];
		pyV[1] = genVelJointLocal[1];
		pyV[2] = genVelJointLocal[2]; 
	}
	else
		Vec3_2_pyVec3(_nodes[index]->joint.GetVelocity(), pyV);
	
	return pyV;
}

boost::python::object VpControlModel::getJointAngAccelerationLocal( int index )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	if(index == 0)
	{
		se3 genAccBodyLocal, genAccJointLocal;

		genAccBodyLocal = _nodes[index]->body.GetGenAccelerationLocal();
		genAccJointLocal = InvAd(Inv(_boneTs[index]), genAccBodyLocal);
		pyV[0] = genAccJointLocal[0]; 
		pyV[1] = genAccJointLocal[1];
		pyV[2] = genAccJointLocal[2]; 
	}
	else
		Vec3_2_pyVec3(_nodes[index]->joint.GetAcceleration(), pyV);
	
	return pyV;
}

//object VpControlModel::getJointPositionGlobal( int index )
object VpControlModel::getJointPositionGlobal( int index, const object& positionLocal/*=object() */ )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	SE3 bodyFrame;
	object pyV = O.copy();
	Vec3 positionLocal_;

	// body frame�� Inv(boneT)�� ���� joint ��ġ ã�´�.
	bodyFrame = _nodes[index]->body.GetFrame();

	if(positionLocal.is_none())
        Vec3_2_pyVec3((bodyFrame * Inv(_boneTs[index])).GetPosition(), pyV);
	else
	{
		pyVec3_2_Vec3(positionLocal, positionLocal_);
        Vec3_2_pyVec3((bodyFrame * Inv(_boneTs[index])) * positionLocal_, pyV);
	}
	return pyV;

//	if(!_nodes[index])	// ������ ��� parent joint frame�� ã�� offset��ŭ transformation ��Ų��.
//	{
//		static SE3 parentJointFrame;
//		static Vec3 offset;
////		int parent = XI(_skeleton.attr("getParentIndex")(index));
//		int parent = XI(_skeleton.attr("getParentJointIndex")(index));
//		parentJointFrame = _nodes[parent]->body.GetFrame() * Inv(_boneTs[parent]);
//		offset = pyVec3_2_Vec3(_skeleton.attr("getOffset")(index));
//		Vec3_2_pyVec3(parentJointFrame * offset, pyV);
//	}
//	else	// ������ �ƴ� ��� body frame�� Inv(boneT)�� ���� joint ��ġ ã�´�.
//	{
//		static SE3 bodyFrame;
//		bodyFrame = _nodes[index]->body.GetFrame();
//		Vec3_2_pyVec3((bodyFrame * Inv(_boneTs[index])).GetPosition(), pyV);
//	}
//	return pyV;
}
object VpControlModel::getJointVelocityGlobal( int index )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	Vec3_2_pyVec3(getBodyVelocityGlobal(index, Inv(_boneTs[index]).GetPosition()), pyV);
	return pyV;
}

object VpControlModel::getJointAccelerationGlobal( int index )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 pospos = Inv(_boneTs[index]).GetPosition();

	Vec3_2_pyVec3(getBodyAccelerationGlobal(index, &(pospos)), pyV);
	return pyV;
}

boost::python::object VpControlModel::getJointOrientationGlobal( int index )
{
	ndarray I = np::array( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	SE3 bodyFrame;
	object pyR = I.copy();

	// body frame�� Inv(boneT)�� ���� joint frame ���Ѵ�
	bodyFrame = _nodes[index]->body.GetFrame();
	SE3_2_pySO3(bodyFrame * Inv(_boneTs[index]), pyR);
	return pyR;
}

boost::python::object VpControlModel::getJointAngVelocityGlobal( int index )
{
	return getBodyAngVelocityGlobal(index);

//	static ndarray O(make_tuple(0.,0.,0.));
//	static Vec3 angVel, parentAngVel;
//	object pyV = O.copy();
//
//	angVel = _nodes[index]->body.GetAngVelocity();
//
//	int parentIndex = getParentIndex(index);
//	if(parentIndex==-1)
//		parentAngVel = Vec3(0.,0.,0.);
//	else
//		parentAngVel = _nodes[parentIndex]->body.GetAngVelocity();
//
//	Vec3_2_pyVec3(angVel - parentAngVel, pyV);
//	return pyV;
}

boost::python::object VpControlModel::getJointAngAccelerationGlobal( int index )
{
	return getBodyAngAccelerationGlobal(index);
}

boost::python::object VpControlModel::getJointFrame( int index )
{
	ndarray frame = np::array( make_tuple(make_tuple(1.,0.,0.,0.), make_tuple(0.,1.,0.,0.), make_tuple(0.,0.,1.,0.), make_tuple(0.,0.,0.,1.)) );
	
	SE3 T = _nodes[index]->body.GetFrame() * Inv(_boneTs[index]);
	SE3_2_pySE3(T, frame);
	return frame;
}

object VpControlModel::getJointVelocityLocal( int index )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	SE3 jointFrame = _nodes[index]->body.GetFrame() * Inv(_boneTs[index]);

	Vec3_2_pyVec3(InvRotate(jointFrame, getBodyVelocityGlobal(index, Inv(_boneTs[index]).GetPosition())), pyV);
	return pyV;
}

object VpControlModel::getJointAccelerationLocal( int index )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 pospos = Inv(_boneTs[index]).GetPosition();
	SE3 jointFrame = _nodes[index]->body.GetFrame() * Inv(_boneTs[index]);

	Vec3_2_pyVec3(InvRotate(jointFrame, getBodyAccelerationGlobal(index, &(pospos))), pyV);
	return pyV;
}


bp::list VpControlModel::getJointOrientationsLocal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getJointOrientationLocal(i));
	return ls;
}

bp::list VpControlModel::getJointAngVelocitiesLocal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getJointAngVelocityLocal(i));
	return ls;
}

bp::list VpControlModel::getJointAngAccelerationsLocal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getJointAngAccelerationLocal(i));
	return ls;
}

bp::list VpControlModel::getJointPositionsGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getJointPositionGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointVelocitiesGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getJointVelocityGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointAccelerationsGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getJointAccelerationGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointOrientationsGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getJointOrientationGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointAngVelocitiesGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getJointAngVelocityGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointAngAccelerationsGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		ls.append(getJointAngAccelerationGlobal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointOrientationsLocal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		ls.append(getJointOrientationLocal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointAngVelocitiesLocal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		ls.append(getJointAngVelocityLocal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointAngAccelerationsLocal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		ls.append(getJointAngAccelerationLocal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointPositionsGlobal()
{
	bp::list ls;
//	for(int i=1; i<_jointElementIndexes.size(); ++i)
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		ls.append(getJointPositionGlobal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointOrientationsGlobal()
{
	bp::list ls;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		ls.append(getJointOrientationGlobal(i));
	return ls;
}
void VpControlModel::setJointAngVelocityLocal( int index, const object& angvel )
{
	if(index == 0)
	{
		se3 genVelBodyLocal, genVelJointLocal;
		
		genVelBodyLocal = _nodes[index]->body.GetGenVelocityLocal();

		genVelJointLocal = InvAd(Inv(_boneTs[index]), genVelBodyLocal);
		genVelJointLocal[0] = XD(angvel[0]);
		genVelJointLocal[1] = XD(angvel[1]);
		genVelJointLocal[2] = XD(angvel[2]); 

		genVelBodyLocal = Ad(Inv(_boneTs[index]), genVelJointLocal);;
		_nodes[index]->body.SetGenVelocityLocal(genVelBodyLocal);
	}
	else
		_nodes[index]->joint.SetVelocity(pyVec3_2_Vec3(angvel));
}

void VpControlModel::setJointAngAccelerationLocal( int index, const object& angacc )
{
	if(index == 0)
	{
		se3 genAccBodyLocal, genAccJointLocal;
		
		genAccBodyLocal = _nodes[index]->body.GetGenAccelerationLocal();

		genAccJointLocal = InvAd(Inv(_boneTs[index]), genAccBodyLocal);
		genAccJointLocal[0] = XD(angacc[0]);
		genAccJointLocal[1] = XD(angacc[1]);
		genAccJointLocal[2] = XD(angacc[2]); 

		genAccBodyLocal = Ad(Inv(_boneTs[index]), genAccJointLocal);;
		_nodes[index]->body.SetGenAccelerationLocal(genAccBodyLocal);
	}
	else
		_nodes[index]->joint.SetAcceleration(pyVec3_2_Vec3(angacc));
}

void VpControlModel::setJointAccelerationGlobal( int index, const object& acc )
{
	if(index == 0)
	{
		Vec3 pospos = Inv(_boneTs[index]).GetPosition();
		setBodyAccelerationGlobal(index, pyVec3_2_Vec3(acc), &(pospos));
	}
	else
		cout << "setJointAccelerationGlobal() : not completely implemented" << endl;
}

void VpControlModel::setJointAngAccelerationGlobal( int index, const object& angacc )
{
	setBodyAngAccelerationGlobal(index, angacc);
}

void VpControlModel::setJointAngAccelerationsLocal( const bp::list& angaccs )
{
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		setJointAngAccelerationLocal(i, angaccs[i]);
}

void VpControlModel::setInternalJointAngAccelerationsLocal( const bp::list& angaccs )
{
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		_nodes[i]->joint.SetAcceleration(pyVec3_2_Vec3(angaccs[i-1]));
}

void VpControlModel::SetJointElasticity(int index, scalar Kx, scalar Ky, scalar Kz)
{
    assert(Kx < 0. && "Joint Elasticity must larger than 0");
    if(Ky < 0.)
    {
        SpatialSpring k(Kx);
        _nodes[index]->joint.SetElasticity(k);
    }
    else
    {
        SpatialSpring k(0., Kx, Ky, Kz);
        _nodes[index]->joint.SetElasticity(k);
    }
}
void VpControlModel::SetJointsElasticity(scalar Kx, scalar Ky, scalar Kz)
{
    for (std::vector<int>::size_type i=1; i<_nodes.size(); i++)
        SetJointElasticity(i, Kx, Ky, Kz);
}

void VpControlModel::SetJointDamping(int index, scalar Dx, scalar Dy, scalar Dz)
{
    assert(Dx < 0. && "Joint Damping must larger than 0");

    if(Dy < 0.)
    {
        SpatialDamper d(Dx);
        _nodes[index]->joint.SetDamping(d);
    }
    else
    {
        SpatialDamper d(0., Dx, Dy, Dz);
        _nodes[index]->joint.SetDamping(d);
    }
}

void VpControlModel::SetJointsDamping(scalar Dx, scalar Dy, scalar Dz)
{
    for (std::vector<int>::size_type i=1; i<_nodes.size(); i++)
        SetJointDamping(i, Dx, Dy, Dz);
}


boost::python::object VpControlModel::getJointTorqueLocal( int index )
{
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	if(index==0) return pyV;

	Vec3_2_pyVec3(_nodes[index]->joint.GetTorque(), pyV);
	return pyV;
}

bp::list VpControlModel::getInternalJointTorquesLocal()
{
	bp::list ls;
//	for(int i=1; i<_jointElementIndexes.size(); ++i)
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		ls.append(getJointTorqueLocal(i));
	return ls;
}

void VpControlModel::setJointTorqueLocal( int index, const object& torque )
{
//	int index = _jointElementIndexes[jointIndex];
	_nodes[index]->joint.SetTorque(pyVec3_2_Vec3(torque));
}

void VpControlModel::setInternalJointTorquesLocal( const bp::list& torques )
{
//	int index;
//	for(int i=1; i<_jointElementIndexes.size(); ++i)
//	{
//		index = _jointElementIndexes[i];
//		_nodes[index]->joint.SetTorque(pyVec3_2_Vec3(torques[i]));
//	}
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
		_nodes[i]->joint.SetTorque(pyVec3_2_Vec3(torques[i-1]));
}


void VpControlModel::applyBodyGenForceGlobal( int index, const object& torque, const object& force, const object& positionLocal/*=object()*/ )
{
	Vec3 zero(0,0,0);
	if(positionLocal==object())
		_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), XD(force[0]), XD(force[1]), XD(force[2])), zero);
	else
		_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), XD(force[0]), XD(force[1]), XD(force[2])), pyVec3_2_Vec3(positionLocal));
}

void VpControlModel::applyBodyForceGlobal( int index, const object& force, const object& positionLocal/*=object()*/ )
{
	Vec3 zero(0,0,0);
	if(positionLocal==object())
		_nodes[index]->body.ApplyGlobalForce(dse3(0.,0.,0., XD(force[0]), XD(force[1]), XD(force[2])), zero);
	else
		_nodes[index]->body.ApplyGlobalForce(dse3(0.,0.,0., XD(force[0]), XD(force[1]), XD(force[2])), pyVec3_2_Vec3(positionLocal));
}

void VpControlModel::applyBodyTorqueGlobal( int index, const object& torque )
{
	Vec3 zero(0,0,0);
	_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), 0.,0.,0.), zero);
}

object VpControlModel::getBodyForceLocal( int index )
{
	dse3 genForce;
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genForce = _nodes[index]->body.GetForce();
	pyV[0] = genForce[3];
	pyV[1] = genForce[4];
	pyV[2] = genForce[5];
	return pyV;

}

object VpControlModel::getBodyNetForceLocal( int index )
{
	dse3 genForce;
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genForce = _nodes[index]->body.GetNetForce();
	pyV[0] = genForce[3];
	pyV[1] = genForce[4];
	pyV[2] = genForce[5];
	return pyV;
}

object VpControlModel::getBodyGravityForceLocal( int index )
{
	dse3 genForce;
	ndarray O = np::array(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genForce = _nodes[index]->body.GetGravityForce();
	pyV[0] = genForce[3];
	pyV[1] = genForce[4];
	pyV[2] = genForce[5];
	return pyV;
}

static ublas::vector<double> ToUblasVector(const Vec3 &v_vp)
{
    ublas::vector<double> v(3);
    for(int i=0; i<3; i++)
        v(i) = v_vp[i];
    return v;
}

static ublas::vector<double> ToUblasVector(const Axis &v_vp)
{
    ublas::vector<double> v(3);
    for(int i=0; i<3; i++)
        v(i) = v_vp[i];
    return v;
}

static ublas::matrix<double> SE3ToUblasRotate(const SE3 &T_vp)
{
    ublas::matrix<double> T(3, 3);
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            T(j, i) = T_vp[i*3 + j];
    return T;
}

static ublas::matrix<double> GetCrossMatrix(const ublas::vector<double> &r)
{
    ublas::matrix<double> R(3, 3);
    R(0, 0) = 0;
    R(1, 0) = r[2];
    R(2, 0) = -r[1];
    R(0, 1) = -r[2];
    R(1, 1) = 0;
    R(2, 1) = r[0];
    R(0, 2) = r[1];
    R(1, 2) = -r[0];
    R(2, 2) = 0;

    return R;
}

static ublas::matrix<double> GetBJointJacobian(const Axis &m_rQ)
{
    ublas::matrix<double> J(3, 3);
	ublas::vector<double> m_rQ_ub = ToUblasVector(m_rQ);

	scalar t = Norm(m_rQ), alpha, beta, gamma, t2 = t * t;

    if ( t < BJOINT_EPS )
	{
		alpha = SCALAR_1_6 - SCALAR_1_120 * t2;
		beta = SCALAR_1 - SCALAR_1_6 * t2;
		gamma = SCALAR_1_2 - SCALAR_1_24 * t2;
	} else
	{
		beta = sin(t) / t;
		alpha = (SCALAR_1 - beta) / t2;
		gamma = (SCALAR_1 - cos(t)) / t2;
	}

//	Axis V = (alpha * Inner(m_rQ, m_rDq)) * m_rQ + beta * m_rDq + gamma * Cross(m_rDq, m_rQ);

    J = alpha * outer_prod(m_rQ_ub, m_rQ_ub) + beta * ublas::identity_matrix<double>(3) - gamma * GetCrossMatrix(m_rQ_ub);

//    J(0, 0) = alpha * m_rQ[0] * m_rQ[0] + beta;
//    J(1, 0) = alpha * m_rQ[1] * m_rQ[0] - gamma * m_rQ[2];
//    J(2, 0) = alpha * m_rQ[2] * m_rQ[0] + gamma * m_rQ[1];
//    J(0, 1) = alpha * m_rQ[0] * m_rQ[1] + gamma * m_rQ[2];
//    J(1, 1) = alpha * m_rQ[1] * m_rQ[1] + beta;
//    J(2, 1) = alpha * m_rQ[2] * m_rQ[1] - gamma * m_rQ[0];
//    J(0, 2) = alpha * m_rQ[0] * m_rQ[2] - gamma * m_rQ[1];
//    J(1, 2) = alpha * m_rQ[1] * m_rQ[2] + gamma * m_rQ[0];
//    J(2, 2) = alpha * m_rQ[2] * m_rQ[2] + beta;

//    return alpha * mm.getDyadMatrixForm(q) + beta * np.eye(3) - gamma * mm.getCrossMatrixForm(q)
    return J;
}

static object ToNumpyArray(const ublas::matrix<double> &m)
{
	bp::tuple shape = bp::make_tuple(m.size1(), m.size2());
    np::dtype dtype = np::dtype::get_builtin<float>();
	ndarray m_np = np::empty(shape, dtype);
	for(ublas::matrix<double>::size_type i=0; i<m.size1(); i++)
	{
	    for(ublas::matrix<double>::size_type j=0; j<m.size2(); j++)
	    {
	        m_np[i][j] = m(i, j);
	    }
    }
	return m_np;
}



object VpControlModel::getLocalJacobian(int index)
{
    return ToNumpyArray(GetBJointJacobian(_nodes[index]->joint.GetDisplacement()));
}

object VpControlModel::getLocalJointVelocity(int index)
{
    return Vec3_2_pyVec3(_nodes[index]->joint.GetVelocity());
}

object VpControlModel::getLocalJointDisplacementDerivatives(int index)
{
    Axis vec = _nodes[index]->joint.GetDisplacementDerivate();
    return Vec3_2_pyVec3(Vec3(vec[0], vec[1], vec[2]));
}

object VpControlModel::computeJacobian(int index, const object& positionGlobal)
{
    //TODO:
	Vec3 effector_position = pyVec3_2_Vec3(positionGlobal);
	vpBJoint *joint;
	SE3 joint_frame;

	bp::tuple shape = bp::make_tuple(6, m_total_dof);
    np::dtype dtype = np::dtype::get_builtin<float>();
	ndarray J = np::zeros(shape, dtype);
	ublas::vector<double> offset;
	ublas::matrix<double> _Jw, _Jv;

	//root joint
	J[0][0] = 1.;
	J[1][1] = 1.;
	J[2][2] = 1.;

    joint_frame = _nodes[0]->body.GetFrame() * Inv(_boneTs[0]);
    _Jw = prod(SE3ToUblasRotate(joint_frame), GetBJointJacobian(LogR(joint_frame)));
    for (int dof_index = 0; dof_index < 3; dof_index++)
    {
        for (int j=0; j<3; j++)
        {
            J[j+3][dof_index+3] = _Jw(j, dof_index);
        }
    }

    //internal joint
	for(std::vector<int>::size_type i=1; i<_nodes.size();i++)
	{
        joint = &(_nodes[i]->joint);
        joint_frame = _nodes[i]->body.GetFrame() * Inv(_boneTs[i]);
        offset = ToUblasVector(effector_position - joint_frame.GetPosition());
        _Jw = prod(SE3ToUblasRotate(joint_frame), GetBJointJacobian(joint->GetDisplacement()));
        _Jv = -prod(GetCrossMatrix(offset), _Jw);
        for (int dof_index = 0; dof_index < _nodes[i]->dof; dof_index++)
        {
            for (int j=0; j<3; j++)
            {
                J[j+0][dof_index] = _Jv(j, dof_index);
                J[j+3][dof_index] = _Jw(j, dof_index);
            }
        }
	}

	return J;
}

/////////////////////////////////////////
// Additional
void VpModel::addBody(bool flagControl)
{
	int n = _nodes.size();
	_nodes.resize(n + 1);
	_boneTs.resize(n + 1);
		
	//add body
	Node* pNode = new Node("Left_Toes");
	scalar density = 1000;
	scalar width = 0.1;
	scalar height = 0.1;
	scalar length = 0.1;
	pNode->body.AddGeometry(new vpBox(Vec3(width, height, length)));
	pNode->body.SetInertia(BoxInertia(density, Vec3(width / 2., height / 2., length / 2.)));

	//boneT = boneT * SE3(pyVec3_2_Vec3(cfgNode.attr("offset")));
	//_boneTs[joint_index] = boneT;
	SE3 newT;// = T * boneT;

	pNode->body.SetFrame(newT);
	_nodes[n] = pNode;

	_boneTs[n] = newT;
	
	if (flagControl == true)
		_pWorld->AddBody(&pNode->body);


	//create new joint
	int parent_index = n - 1;
	SE3 invLocalT;
	Node* pParentNode = _nodes[parent_index];

	pParentNode->body.SetJoint(&pNode->joint, Inv(_boneTs[parent_index])*Inv(invLocalT));
	pNode->body.SetJoint(&pNode->joint, Inv(_boneTs[n]));
	pNode->use_joint = true;

}

 int VpModel::id2index(int id)
 {
 	int index = 0;
 	for (int i = 0; i < getBodyNum(); i++)
 	{
 		if (id == index2id(i))
 		{
 			index = i;
 			break;
 		}
 	}
 	return index;
 }

void VpModel::SetBodyColor(int id, unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
	int index = id2index(id);
	Node* pNode = _nodes[index];
	pNode->color[0] = r;
	pNode->color[1] = g;
	pNode->color[2] = b;
	pNode->color[3] = a;
}

void VpControlModel::setSpring(int body1Index, int body2Index, scalar elasticity, scalar damping, const object& p1, const object& p2, scalar initialDistance)
{
	vpSpring* spring = new vpSpring;
	Vec3 v1 = pyVec3_2_Vec3(p1);
	Vec3 v2 = pyVec3_2_Vec3(p2);
	spring->Connect(&(_nodes[body1Index]->body), &(_nodes[body2Index]->body), v1, v2);
	spring->SetElasticity(elasticity);
	spring->SetDamping(damping);
	spring->SetInitialDistance(initialDistance);
	_springs.push_back(spring);
}



// must be called at first:clear all torques and accelerations
bp::list VpControlModel::getInverseEquationOfMotion(object &invM, object &invMb)
{
	bp::list ls;
	ls.append(_nodes.size());


	// M^-1 * tau - M^-1 * b = ddq
	// ddq^T = [rootjointLin^T rootjointAng^T joints^T]^T
	
	
	//vpBody *Hip = &(_nodes.at(0)->body);
	//Hip->ResetForce();
	//for(size_t i=0; i<_nodes.size(); i++)
	//{
		//_nodes.at(i)->body.ResetForce();
		//_nodes.at(i)->joint.SetTorque(Vec3(0,0,0));
	//}
	//Hip->GetSystem()->ForwardDynamics();
	//std::cout << Hip->GetGenAccelerationLocal();
	//for(size_t i=1; i<_nodes.size(); i++)
	//{
		//std::cout << _nodes[i]->joint.GetAcceleration();
	//}



	int n = _nodes.size()-1;
	int N = 6+3*n;
	dse3 zero_dse3(0.0);
	Vec3 zero_Vec3(0.0);

	vpBody *Hip = &(_nodes.at(0)->body);

	//save current ddq and tau
	std::vector<Vec3> accBackup;
	std::vector<Vec3> torBackup;
	for(int i=0; i<n; i++)
	{
		vpBJoint *joint = &(_nodes.at(i+1)->joint);
		accBackup.push_back(joint->GetAcceleration());
		torBackup.push_back(joint->GetTorque());
	}
	// se3 hipAccBackup = Hip->GetGenAcceleration();
	// dse3 hipTorBackup = Hip->GetForce();

	Hip->ResetForce();
	for(std::vector<int>::size_type i=0; i<_nodes.size(); i++)
	{
		_nodes.at(i)->body.ResetForce();
		_nodes.at(i)->joint.SetTorque(Vec3(0,0,0));
	}

	//get invMb
	Hip->ApplyLocalForce(zero_dse3, zero_Vec3);
	for(int i=0; i<n; i++)
	{
		vpBJoint *joint = &(_nodes.at(i+1)->joint);
		joint->SetTorque(zero_Vec3);
	}
	Hip->GetSystem()->ForwardDynamics();
	se3 hipAcc_tmp = Hip->GetGenAccelerationLocal(); // represented in body frame
//	se3 hipAcc_tmp = InvAd((_boneTs[0]), Hip->GetGenAccelerationLocal()); // represented in body frame

//	se3 hipAcc_tmp = InvAd(_boneTs[0], Hip->GetGenAccelerationLocal());
//	se3 hipVelLocal_joint = InvAd(_boneTs[0], Hip->GetGenVelocityLocal());
//	Vec3 hipAngVelLocal_joint(hipVelLocal_joint[0], hipVelLocal_joint[1], hipVelLocal_joint[2]);
//	Vec3 hipLinVelLocal_joint(hipVelLocal_joint[3], hipVelLocal_joint[4], hipVelLocal_joint[5]);
//
//	hipAcc_tmp += Cross(hipAngVelLocal_joint, hipLinVelLocal_joint);

//	Vec3 hipJointPosLocal = Inv(_boneTs[0]).GetPosition();
//	SE3 hipFrame_joint = Hip->GetFrame() * Inv(_boneTs[0]);
//	SE3 hipFrame_body = Hip->GetFrame();
//
//	Vec3 hipAngVelGlobal = Hip->GetAngVelocity();
//
//	se3 hipGenAccLocal_body = Hip->GetGenAccelerationLocal();
//	Vec3 hipAngAccLocal_body(hipGenAccLocal_body[0], hipGenAccLocal_body[1], hipGenAccLocal_body[2]);
//	Vec3 hipLinAccLocal_body(hipGenAccLocal_body[3], hipGenAccLocal_body[4], hipGenAccLocal_body[5]);
//
//	Vec3 hipAngAccGlobal_body = Rotate(hipFrame_body, hipAngAccLocal_body);
//	Vec3 hipLinAccGlobal_body = Rotate(hipFrame_body, hipLinAccLocal_body);
//
//	Vec3 hipAngAccGlobal_joint = hipAngAccGlobal_body;
//	Vec3 hipLinAccGlobal_joint = hipLinAccGlobal_body + Cross(hipAngAccGlobal_body, hipJointPosLocal)
//							+ Cross(hipAngVelGlobal, Cross(hipAngVelGlobal, hipJointPosLocal));
//
//	Vec3 hipAngAccLocal_joint = InvRotate(hipFrame_joint, hipAngAccGlobal_joint);
//	Vec3 hipLinAccLocal_joint = InvRotate(hipFrame_joint, hipLinAccGlobal_joint);
//
////	se3 hipAcc_tmp(hipAngAccGlobal_joint[0], hipAngAccGlobal_joint[1], hipAngAccGlobal_joint[2],
////				   hipLinAccGlobal_joint[0], hipLinAccGlobal_joint[1], hipLinAccGlobal_joint[2]);
//	se3 hipAcc_tmp(hipAngAccLocal_joint[0], hipAngAccLocal_joint[1], hipAngAccLocal_joint[2],
//				   hipLinAccLocal_joint[0], hipLinAccLocal_joint[1], hipLinAccLocal_joint[2]);
	{
		invMb[0] = -hipAcc_tmp[3];
		invMb[1] = -hipAcc_tmp[4];
		invMb[2] = -hipAcc_tmp[5];
		invMb[3] = -hipAcc_tmp[0];
		invMb[4] = -hipAcc_tmp[1];
		invMb[5] = -hipAcc_tmp[2];
	}
	//std::cout << "Hip velocity: " << Hip->GetGenVelocity();
	for(int i=0; i<n; i++)
	{
		Vec3 acc(0,0,0);
		vpBJoint *joint = &(_nodes.at(i+1)->joint);
		acc = joint->GetAcceleration();
		for(int j=0; j<3; j++)
		{
			invMb[6+3*i+j] = -acc[j];
		}
	}

	//get M
	for(int i=0; i<N; i++)
	{
		Hip->ResetForce();
		for(std::vector<int>::size_type i=0; i<_nodes.size(); i++)
		{
			_nodes.at(i)->body.ResetForce();
			_nodes.at(i)->joint.SetTorque(Vec3(0,0,0));
		}

		dse3 genForceLocal(0.0);
		if (i < 3) genForceLocal[i+3] = 1.0;
		else if(i<6) genForceLocal[i-3] = 1.0;
		for(int j=0; j<n; j++)
		{
			Vec3 torque(0., 0., 0.);
			if ( i >= 6 && (i-6)/3 == j )
				torque[ (i-6)%3 ] = 1.;
			vpBJoint *joint = &(_nodes.at(j+1)->joint);
			joint->SetTorque(torque);
		}
		Hip->ApplyLocalForce(genForceLocal, zero_Vec3);

		Hip->GetSystem()->ForwardDynamics();
		 se3 hipAcc_tmp = Hip->GetGenAccelerationLocal();
////		 se3 hipAcc_tmp_1 = Ad((_boneTs[0]), Hip->GetGenAccelerationLocal());
////		se3 hipAcc_tmp_1 = InvAd(_boneTs[0], Hip->GetGenAccelerationLocal());
////		se3 hipVelLocal_joint_1 = InvAd(_boneTs[0], Hip->GetGenVelocityLocal());
////		Vec3 hipAngVelLocal_joint_1(hipVelLocal_joint_1[0], hipVelLocal_joint_1[1], hipVelLocal_joint_1[2]);
////		Vec3 hipLinVelLocal_joint_1(hipVelLocal_joint_1[3], hipVelLocal_joint_1[4], hipVelLocal_joint_1[5]);
////
////		hipAcc_tmp_1 += Cross(hipAngVelLocal_joint_1, hipLinVelLocal_joint_1);
//
//		Vec3 hipJointPosLocal = Inv(_boneTs[0]).GetPosition();
//		SE3 hipFrame_joint = Hip->GetFrame() * Inv(_boneTs[0]);
//		SE3 hipFrame_body = Hip->GetFrame();
//
//		Vec3 hipAngVelGlobal = Hip->GetAngVelocity();
//
//		se3 hipGenAccLocal_body = Hip->GetGenAccelerationLocal();
//		Vec3 hipAngAccLocal_body(hipGenAccLocal_body[0], hipGenAccLocal_body[1], hipGenAccLocal_body[2]);
//		Vec3 hipLinAccLocal_body(hipGenAccLocal_body[3], hipGenAccLocal_body[4], hipGenAccLocal_body[5]);
//
//		Vec3 hipAngAccGlobal_body = Rotate(hipFrame_body, hipAngAccLocal_body);
//		Vec3 hipLinAccGlobal_body = Rotate(hipFrame_body, hipLinAccLocal_body);
//
//		Vec3 hipAngAccGlobal_joint = hipAngAccGlobal_body;
//		Vec3 hipLinAccGlobal_joint = hipLinAccGlobal_body + Cross(hipAngAccGlobal_body, hipJointPosLocal)
//									 + Cross(hipAngVelGlobal, Cross(hipAngVelGlobal, hipJointPosLocal));
//
//		Vec3 hipAngAccLocal_joint = InvRotate(hipFrame_joint, hipAngAccGlobal_joint);
//		Vec3 hipLinAccLocal_joint = InvRotate(hipFrame_joint, hipLinAccGlobal_joint);
//
////	se3 hipAcc_tmp(hipAngAccGlobal_joint[0], hipAngAccGlobal_joint[1], hipAngAccGlobal_joint[2],
////				   hipLinAccGlobal_joint[0], hipLinAccGlobal_joint[1], hipLinAccGlobal_joint[2]);
//		se3 hipAcc_tmp(hipAngAccLocal_joint[0], hipAngAccLocal_joint[1], hipAngAccLocal_joint[2],
//					   hipLinAccLocal_joint[0], hipLinAccLocal_joint[1], hipLinAccLocal_joint[2]);


		for (int j = 0; j < 3; j++)
		{
			invM[j][i] = hipAcc_tmp[j+3] + invMb[j];
		}
		for (int j = 3; j < 6; j++)
		{
			invM[j][i] = hipAcc_tmp[j-3] + invMb[j];
		}
		for(int j=0; j<n; j++)
		{
			Vec3 acc(0,0,0);
			vpBJoint *joint = &(_nodes.at(j+1)->joint);
			acc = joint->GetAcceleration();
			for(int k=0; k<3; k++)
			{
				invM[6+3*j+k][i] = acc[k] + invMb[6+3*j+k];
			}
		}
	}

	// restore ddq and tau
	for(int i=0; i<n; i++)
	{
		vpBJoint *joint = &(_nodes.at(i+1)->joint);
		joint->SetAcceleration(accBackup.at(i));
		joint->SetTorque(torBackup.at(i));
		joint->SetAcceleration(Vec3(0., 0., 0.));
		joint->SetTorque(Vec3(0., 0., 0.));
	}
	//Hip->SetGenAcceleration(hipAccBackup);

	Hip->ResetForce();
//	Hip->ApplyGlobalForce(hipTorBackup, zero_Vec3);
	return ls;
}

bp::list VpControlModel::getEquationOfMotion(object& M, object& b)
{
	bp::list ls;
	ls.append(_nodes.size());
	//for(int i=0; i<_nodes.size(); i++)
	//{
		//ls.append(_nodes.at(i)->name);
		//ls.append(_nodes.at(i)->dof);
	//}
	//
	//RMatrix ddq = get_ddq(), tau = get_tau(); // save current ddq and tau
	//int n = getNumCoordinates();
	//M.ReNew(n,n);
	//set_ddq(Zeros(n,1));
	//GSystem::calcInverseDynamics();
	//b = get_tau();
	//for (int i=0; i<n; i++) {
	//	RMatrix unit = Zeros(n,1);
	//	unit[i] = 1;
	//	set_ddq(unit);
	//	GSystem::calcInverseDynamics();
	//	get_tau(&M[i*n]);
	//	for (int j=0; j<n; j++) {
	//		M[i*n+j] -= b[j];
	//	}
	//}
	//set_ddq(ddq); set_tau(tau); // restore ddq and tau


	// M * ddq + b = tau
	// ddq^T = [rootjointLin^T rootjointAng^T joints^T]^T
	
	
	int n = _nodes.size()-1;
	int N = 6+3*n;

	vpBody *Hip = &(_nodes.at(0)->body);

	//save current ddq and tau
	std::vector<Vec3> accBackup;
	std::vector<Vec3> torBackup;
	for(int i=0; i<n; i++)
	{
		vpBJoint *joint = &(_nodes.at(i+1)->joint);
		accBackup.push_back(joint->GetAcceleration());
		torBackup.push_back(joint->GetTorque());
	}
	se3 hipAccBackup = Hip->GetGenAcceleration();
	dse3 hipTorBackup = Hip->GetForce();

	//Hip->ResetForce();
	//for(int i=0; i<_nodes.size(); i++)
	//{
		////_nodes.at(i)->body.ResetForce();
		//_nodes.at(i)->joint.SetTorque(Vec3(0,0,0));
	//}

	//get b
	se3 zero_se3(0.0);
	Vec3 zero_Vec3(0.0);
	Hip->SetGenAccelerationLocal(zero_se3);
//	Hip->SetGenAcceleration(zero_se3);
	for(int i=0; i<n; i++)
	{
		vpBJoint *joint = &(_nodes.at(i+1)->joint);
		joint->SetAcceleration(zero_Vec3);
	}
	Hip->GetSystem()->InverseDynamics();
	dse3 hipTorque_tmp_b = Hip->GetForce(); // represented in body frame
	SE3 hipFrame = Hip->GetFrame();
	{
		b[0] = hipTorque_tmp_b[3];
		b[1] = hipTorque_tmp_b[4];
		b[2] = hipTorque_tmp_b[5];
		b[3] = hipTorque_tmp_b[0];
		b[4] = hipTorque_tmp_b[1];
		b[5] = hipTorque_tmp_b[2];
	}
	//std::cout << "Hip velocity: " << Hip->GetGenVelocity();
	for(int i=0; i<n; i++)
	{
		Vec3 torque(0,0,0);
		vpBJoint *joint = &(_nodes.at(i+1)->joint);
		torque = joint->GetTorque();
		//std::cout << "name: " << _nodes[i+1]->name <<std::endl;
		//std::cout << "torque: " << joint->GetTorque(); 
		//std::cout << "velocity: " << joint->GetVelocity();
		for(int j=0; j<3; j++)
		{
			b[6+3*i+j] = torque[j];
		}
	}

	//get M
	for(int i=0; i<N; i++)
	{
		//Hip->ResetForce();
		//for(int i=0; i<_nodes.size(); i++)
		//{
			////_nodes.at(i)->body.ResetForce();
			//_nodes.at(i)->joint.SetTorque(Vec3(0,0,0));
		//}
		Vec3 acc_hip(0.0);
		Axis accang_hip(0.0);
		if (i < 3) acc_hip[i] = 1.0;
		else if(i<6) accang_hip[i-3] = 1.0;
		for(int j=0; j<n; j++)
		{
			Vec3 acc(0., 0., 0.);
			if ( i >= 6 && (i-6)/3 == j )
				acc[ (i-6)%3 ] = 1.;
			vpBJoint *joint = &(_nodes.at(j+1)->joint);
			joint->SetAcceleration(acc);
		}
		{
			se3 genAccBodyLocal(accang_hip, acc_hip);
			Hip->SetGenAccelerationLocal(genAccBodyLocal);
			// Hip->SetGenAcceleration(genAccBodyLocal);
		}

		Hip->GetSystem()->InverseDynamics();
		dse3 hipTorque_tmp = Hip->GetForce();
		for (int j = 0; j < 3; j++)
		{
			M[j][i] = hipTorque_tmp[j+3] - hipTorque_tmp_b[j+3];
		}
		for (int j = 3; j < 6; j++)
		{
			M[j][i] = hipTorque_tmp[j-3] - hipTorque_tmp_b[j-3];
		}
		//for (int j = 0; j < 6; j++)
			//M[j][i] = hipTorque_tmp[j] - hipTorque_tmp_b[j];
		for(int j=0; j<n; j++)
		{
			Vec3 torque(0,0,0);
			vpBJoint *joint = &(_nodes.at(j+1)->joint);
			torque = joint->GetTorque();
			for(int k=0; k<3; k++)
			{
				M[6+3*j+k][i] = torque[k] - b[6+3*j+k];
			}
		}
	}

	// restore ddq and tau
	
	for(int i=0; i<n; i++)
	{
		vpBJoint *joint = &(_nodes.at(i+1)->joint);
		joint->SetAcceleration(accBackup.at(i));
		joint->SetTorque(torBackup.at(i));
	}
	Hip->SetGenAcceleration(hipAccBackup);
	Hip->ResetForce();
	Hip->ApplyGlobalForce(hipTorBackup, zero_Vec3);
	return ls;
}

//void VpControlModel::stepKinematics(double dt, const object& acc)
//{
	//Vec3 ddq = pyVec3_2_Vec3(acc);
	//Vec3 dq(0.0);
	//SE3 q;
	//vpBJoint *joint = &(_nodes.at(2)->joint);
	//dq = joint->GetVelocity() + ddq * dt;
	//joint->SetVelocity(dq);
	////q = joint->GetOrientation() * Exp(Axis(dq*dt));
	//q = Exp(Axis(dq*dt))*joint->GetOrientation(); 
	//joint->SetOrientation(q);
//}
#include <VP/vpWorld.h>
void VpControlModel::stepKinematics(double dt, const bp::list& accs)
{
	vpBody *Hip = &(_nodes.at(0)->body);
	Vec3 hipacc = pyVec3_2_Vec3(accs[0].slice(0,3));
	Axis hipangacc(pyVec3_2_Vec3(accs[0].slice(3,6)));
	{
		se3 genAccBodyLocal(hipangacc, hipacc);
		se3 genVelBodyLocal = Hip->GetGenVelocityLocal() + genAccBodyLocal*dt ;
		Hip->SetGenVelocityLocal(genVelBodyLocal);
		SE3 rootFrame = Hip->GetFrame() * Exp(dt*genVelBodyLocal);
		Hip->SetFrame(rootFrame);
	}

	Vec3 ddq(0.0), dq(0.0), zero_Vec3(0.0);
	SE3 q;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
	{
		ddq = pyVec3_2_Vec3(accs[i]);
		vpBJoint *joint = &(_nodes[i]->joint);
		dq = joint->GetVelocity() + ddq * dt;
		joint->SetVelocity(dq);
		q = joint->GetOrientation() * Exp(Axis(dq*dt));
		joint->SetOrientation(q);
	}

	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		_nodes[i]->body.UpdateGeomFrame();
	
	se3 zero_se3(0.0);
	Hip->ResetForce();
	//for(int i=0; i<_nodes.size(); i++)
	//{
		////_nodes.at(i)->body.ResetForce();
		//_nodes.at(i)->body.SetGenAcceleration(zero_se3);
	//}
}
