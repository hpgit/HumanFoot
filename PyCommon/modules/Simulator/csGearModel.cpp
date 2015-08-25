#include "stdafx.h"

#include "../../externalLibs/common/boostPythonUtil.h"

#include "GearUtil.h"
#include "csGearWorld.h"
#include "csGearModel.h"

#include <sstream>
#include <float.h>

#define MAX_X 1	// 0001
#define MAX_Y 2	// 0010
#define MAX_Z 4	// 0100

using std::string;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyPositionGlobal_py_overloads, getBodyPositionGlobal_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyVelocityGlobal_py_overloads, getBodyVelocityGlobal_py, 1, 2);
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_applyBodyGenForceGlobal_overloads, applyBodyGenForceGlobal, 3, 4);
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_applyBodyForceGlobal_overloads, applyBodyForceGlobal, 2, 3);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(initializeHybridDynamics_overloads, initializeHybridDynamics, 0, 1);

BOOST_PYTHON_MODULE(csGearModel)
{
	numeric::array::set_module_and_type("numpy", "ndarray");

	class_<GearModel>("GearModel", init<GearWorld*, object, object>())
		.def("__str__", &GearModel::__str__)
		.def("getBodyNum", &GearModel::getBodyNum)
		.def("getBodyMasses", &GearModel::getBodyMasses)
		.def("getTotalMass", &GearModel::getTotalMass)
		.def("getBodyShape", &GearModel::getBodyShape)
		.def("getBodyVerticesPositionGlobal", &GearModel::getBodyVerticesPositionGlobal)

		.def("index2name", &GearModel::index2name)
		//.def("index2id", &GearModel::index2id)
		.def("name2index", &GearModel::name2index)
		//.def("name2id", &GearModel::name2id)

		.def("getBodyInertiaLocal", &GearModel::getBodyInertiaLocal_py)
		.def("getBodyInertiaGlobal", &GearModel::getBodyInertiaGlobal_py)

		.def("getBodyInertiasLocal", &GearModel::getBodyInertiasLocal)
		.def("getBodyInertiasGlobal", &GearModel::getBodyInertiasGlobal)
		
		.def("getBodyPositionGlobal", &GearModel::getBodyPositionGlobal_py, getBodyPositionGlobal_py_overloads())
		.def("getBodyVelocityGlobal", &GearModel::getBodyVelocityGlobal_py, getBodyVelocityGlobal_py_overloads())
		.def("getBodyAccelerationGlobal", &GearModel::getBodyAccelerationGlobal_py)
		.def("getBodyOrientationGlobal", &GearModel::getBodyOrientationGlobal)
		.def("getBodyAngVelocityGlobal", &GearModel::getBodyAngVelocityGlobal)
		.def("getBodyAngAccelerationGlobal", &GearModel::getBodyAngAccelerationGlobal)

		.def("getBodyPositionsGlobal", &GearModel::getBodyPositionsGlobal)
		.def("getBodyVelocitiesGlobal", &GearModel::getBodyVelocitiesGlobal)
		.def("getBodyAccelerationsGlobal", &GearModel::getBodyAccelerationsGlobal)
		.def("getBodyAngVelocitiesGlobal", &GearModel::getBodyAngVelocitiesGlobal)
		.def("getBodyAngAccelerationsGlobal", &GearModel::getBodyAngAccelerationsGlobal)

		//.def("setBodyPositionGlobal", &GearModel::setBodyPositionGlobal_py)
		//.def("setBodyVelocityGlobal", &GearModel::setBodyVelocityGlobal_py)
		//.def("setBodyAccelerationGlobal", &GearModel::setBodyAccelerationGlobal_py)
		//.def("setBodyAngVelocityGlobal", &GearModel::setBodyAngVelocityGlobal)
		//.def("setBodyAngAccelerationGlobal", &GearModel::setBodyAngAccelerationGlobal)

		.def("translateByOffset", &GearModel::translateByOffset)
		.def("rotate", &GearModel::rotate)

		.def("update", &GearModel::update)
		;

	class_<GearControlModel, bases<GearModel> >("GearControlModel", init<GearWorld*, object, object>())
		.def("__str__", &GearControlModel::__str__)
		.def("getJointNum", &GearControlModel::getJointNum)
		.def("getInternalJointNum", &GearControlModel::getInternalJointNum)
		.def("getInternalJointDOFs", &GearControlModel::getInternalJointDOFs)
		.def("getDOFs", &GearControlModel::getDOFs)
		.def("getTotalDOF", &GearControlModel::getTotalDOF)
		.def("getTotalInternalJointDOF", &GearControlModel::getTotalInternalJointDOF)

		//.def("fixBody", &GearControlModel::fixBody)

		.def("initializeHybridDynamics", &GearControlModel::initializeHybridDynamics, initializeHybridDynamics_overloads())
		.def("solveHybridDynamics", &GearControlModel::solveHybridDynamics)

		.def("getDOFPositions", &GearControlModel::getDOFPositions)
		//.def("getDOFVelocities", &GearControlModel::getDOFVelocities)
		//.def("getDOFAccelerations", &GearControlModel::getDOFAccelerations)
		//.def("getDOFAxeses", &GearControlModel::getDOFAxeses)

		//.def("getDOFPositionsLocal", &GearControlModel::getDOFPositionsLocal)
		//.def("getDOFVelocitiesLocal", &GearControlModel::getDOFVelocitiesLocal)
		//.def("getDOFAccelerationsLocal", &GearControlModel::getDOFAccelerationsLocal)
		//.def("getDOFAxesesLocal", &GearControlModel::getDOFAxesesLocal)

		.def("getDOFPositionsEuler", &GearControlModel::getDOFPositionsEuler)
		.def("getDOFVelocitiesEuler", &GearControlModel::getDOFVelocitiesEuler)
		.def("getDOFAccelerationsEuler", &GearControlModel::getDOFAccelerationsEuler)
		//.def("getDOFAxesesEuler", &GearControlModel::getDOFAxesesEuler)

		//.def("setDOFAccelerations", &GearControlModel::setDOFAccelerations)

		.def("getJointOrientationLocal", &GearControlModel::getJointOrientationLocal)
		//.def("getJointAngVelocityLocal", &GearControlModel::getJointAngVelocityLocal)
		//.def("getJointAngAccelerationLocal", &GearControlModel::getJointAngAccelerationLocal)

		//.def("getJointPositionGlobal", &GearControlModel::getJointPositionGlobal)
		//.def("getJointVelocityGlobal", &GearControlModel::getJointVelocityGlobal)
		//.def("getJointVelocityLocal", &GearControlModel::getJointVelocityLocal)
		//.def("getJointAccelerationGlobal", &GearControlModel::getJointAccelerationGlobal)
		//.def("getJointAccelerationLocal", &GearControlModel::getJointAccelerationLocal)
		.def("getJointOrientationGlobal", &GearControlModel::getJointOrientationGlobal)
		//.def("getJointAngVelocityGlobal", &GearControlModel::getJointAngVelocityGlobal)
		//.def("getJointAngAccelerationGlobal", &GearControlModel::getJointAngAccelerationGlobal)

		//.def("getJointOrientationsLocal", &GearControlModel::getJointOrientationsLocal)
		//.def("getJointAngVelocitiesLocal", &GearControlModel::getJointAngVelocitiesLocal)
		//.def("getJointAngAccelerationsLocal", &GearControlModel::getJointAngAccelerationsLocal)

		//.def("getJointPositionsGlobal", &GearControlModel::getJointPositionsGlobal)
		//.def("getJointVelocitiesGlobal", &GearControlModel::getJointVelocitiesGlobal)
		//.def("getJointAccelerationsGlobal", &GearControlModel::getJointAccelerationsGlobal)
		//.def("getJointOrientationsGlobal", &GearControlModel::getJointOrientationsGlobal)
		//.def("getJointAngVelocitiesGlobal", &GearControlModel::getJointAngVelocitiesGlobal)
		//.def("getJointAngAccelerationsGlobal", &GearControlModel::getJointAngAccelerationsGlobal)

		.def("getInternalJointOrientationsLocal", &GearControlModel::getInternalJointOrientationsLocal)
		//.def("getInternalJointAngVelocitiesLocal", &GearControlModel::getInternalJointAngVelocitiesLocal)
		//.def("getInternalJointAngAccelerationsLocal", &GearControlModel::getInternalJointAngAccelerationsLocal)

		//.def("getInternalJointPositionsGlobal", &GearControlModel::getInternalJointPositionsGlobal)
		//.def("getInternalJointOrientationsGlobal", &GearControlModel::getInternalJointOrientationsGlobal)

		//.def("setJointAngVelocityLocal", &GearControlModel::setJointAngVelocityLocal)
		//.def("setJointAngAccelerationLocal", &GearControlModel::setJointAngAccelerationLocal)

		//.def("setJointAccelerationGlobal", &GearControlModel::setJointAccelerationGlobal)
		//.def("setJointAngAccelerationGlobal", &GearControlModel::setJointAngAccelerationGlobal)

		//.def("setJointAngAccelerationsLocal", &GearControlModel::setJointAngAccelerationsLocal)
		
		//.def("setInternalJointAngAccelerationsLocal", &GearControlModel::setInternalJointAngAccelerationsLocal)


		//.def("applyBodyGenForceGlobal", &GearControlModel::applyBodyGenForceGlobal, VpControlModel_applyBodyGenForceGlobal_overloads())
		//.def("applyBodyForceGlobal", &GearControlModel::applyBodyForceGlobal, VpControlModel_applyBodyForceGlobal_overloads())
		//.def("applyBodyTorqueGlobal", &GearControlModel::applyBodyTorqueGlobal)

		//.def("getBodyForceLocal", &GearControlModel::getBodyForceLocal)
		//.def("getBodyNetForceLocal", &GearControlModel::getBodyNetForceLocal)
		//.def("getBodyGravityForceLocal", &GearControlModel::getBodyGravityForceLocal)

		//.def("getJointTorqueLocal", &GearControlModel::getJointTorqueLocal)
		//.def("getInternalJointTorquesLocal", &GearControlModel::getInternalJointTorquesLocal)

		//.def("setJointTorqueLocal", &GearControlModel::setJointTorqueLocal)
		//.def("setInternalJointTorquesLocal", &GearControlModel::setInternalJointTorquesLocal)

		.def("calcMassMatrix3", &GearControlModel::calcMassMatrix3)
		.def("stepKinematics", &GearControlModel::stepKinematics)
		;
}

 
GearModel::GearModel( GearWorld* pWorld, const object& createPosture, const object& config ) 
:_pWorld(pWorld), _config(config), _skeleton(createPosture.attr("skeleton"))
{
	int num = XI(createPosture.attr("skeleton").attr("getJointNum")());
	_nodes.resize(num, NULL);
	_boneTs.resize(num, SE3());

	//createBodies(createPosture);
	modelJointFree.spherical_joint.coord_chart =
		GJointSpherical::EULER_ZXY;
		//CoordinateChartForSphericalJoint::EULER_ZXY;
	createBodiesAndJoints(createPosture);
	build_name2index();
	_pWorld->_system.buildSystem(&pWorld->_ground);

	update(createPosture);

}

GearModel::~GearModel()
{
	for( NODES_ITOR it=_nodes.begin(); it!=_nodes.end(); ++it)
		if(*it)
			delete *it;
}

void GearModel::createBodiesAndJoints(const object& posture)
{
	object joint = posture.attr("skeleton").attr("root");

	//object rootPos = posture.attr("rootPos");
	//SE3 T = SE3(pyVec3_2_Vec3(rootPos));

	object tpose = posture.attr("getTPose")();
	_createBodyAndJoint(joint, tpose, &(_pWorld->_ground), SE3());

	//rootPos_Vec3 = pyVec3_2_Vec3(rootPos);
	//modelJointFree.setPosition(Vec3(0,0,0), rootPos);
}

static bool hasAttrName(const object& obj, std::string str)
{
	return PyObject_HasAttrString(obj.ptr(), str.c_str())
		&& obj.attr(str.c_str()) != object();
}
		
void GearModel::_createBodyAndJoint(const object& joint, const object& posture, GBody *parentBody, const SE3 &parentBoneTs)
{
	int len_joint_children = len(joint.attr("children")); 
	if (len_joint_children == 0 )
		return;

	SE3 P = SE3(pyVec3_2_Vec3(joint.attr("offset")));

	std::string joint_name = XS(joint.attr("name"));
	int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
	SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));

	if (_config.attr("hasNode")(joint_name))
	{
		Vec3 offset(0.);
		for( int i=0 ; i<len_joint_children; ++i)
			offset += pyVec3_2_Vec3(joint.attr("children")[i].attr("offset"));
		offset *= (1./len_joint_children);

		//offset = Vec3(0,0,0);
		SE3 boneT(offset*.5);
		if(!joint_name.compare("Hips"))
			boneT=SE3();
		//else
			//boneT *=getSE3FromVectors(offset, Vec3(0,1,0));

		//Vec3 defaultBoneV(0,0,1);
		//SE3 boneR = getSE3FromVectors(defaultBoneV, offset);
		//if(joint_name.compare(string("Hips")))
			//boneT = boneT * boneR;

		Node* pNode = new Node(joint_name);
		_nodes[joint_index] = pNode;

		object cfgNode = _config.attr("getNode")(joint_name);
		gReal length;
		if( hasAttrName(cfgNode, string("length")) )
		{
			length = XD(cfgNode.attr("length")) * XD(cfgNode.attr("boneRatio"));
		}
		else
			length = Norm(offset) * XD(cfgNode.attr("boneRatio"));

		gReal density = XD(cfgNode.attr("density"));
		gReal width, height;
		if( hasAttrName(cfgNode, string("width")) )
		{
			width = XD(cfgNode.attr("width"));
			if( hasAttrName(cfgNode, string("mass")) )
				height = (XD(cfgNode.attr("mass")) / (density * length)) / width;
			else
				height = .1;
		}
		else
		{
			if( hasAttrName(cfgNode, string("mass")) )
				width = sqrt( (XD(cfgNode.attr("mass")) / (density * length)) );
			else
				width = .1;
			height = width;
		}

		//TODO:
		//pNode->body.AddGeometry(new vpBox(Vec3(width, height, length)));
		//((HpGBody*)&(pNode->body))->setSize(Vec3(width, length, height), getSE3FromVectors(offset, Vec3(0,1,0)));
		((HpGBody*)&(pNode->body))->setSize(Vec3(width, length, height), getSE3FromVectors(Vec3(0,1,0), offset));
		gReal size[3];
		size[0] = width/2.;
		size[1] = length/2.;
		size[2] = height/2.;
		gReal mass = (gReal)8.0 * density * size[0] * size[1] * size[2];
		gReal ix = mass * (size[1] * size[1] + size[2] * size[2]) / 3.0;
		gReal iy = mass * (size[0] * size[0] + size[2] * size[2]) / 3.0;
		gReal iz = mass * (size[0] * size[0] + size[1] * size[1]) / 3.0;
		pNode->body.setMass(mass, ix, iy, iz, 0, 0, 0);

		_boneTs[joint_index] = boneT;

		//TODO:
		//pNode->body.SetFrame(newT);
		if(!joint_name.compare("Hips"))
		{
			modelJointFree.connectBodies(parentBody, &(pNode->body));
			modelJointFree.setPositionAndOrientation(Inv(parentBoneTs)*P, Inv(boneT));
		}
		else
		{
			pNode->joint.push_back(new GJointRevolute);
			pNode->joint.push_back(new GJointRevolute);
			pNode->joint.push_back(new GJointRevolute);
			
			HpGBody *dummy1 = new HpGBody;
			HpGBody *dummy2 = new HpGBody;
			dummy1->setMass(0);
			dummy2->setMass(0);
			//std::cout << joint_index << " " <<joint_name << Inv(parentBoneTs)*P << Inv(boneT);
			pNode->joint[0]->connectBodies(parentBody, dummy1);
			pNode->joint[0]->setPositionAndOrientation(Inv(parentBoneTs)*P, SE3());
			pNode->joint[0]->setAxis(Vec3(0,0,1));

			pNode->joint[1]->connectBodies(dummy1, dummy2);
			pNode->joint[1]->setPositionAndOrientation(SE3(), SE3());
			pNode->joint[1]->setAxis(Vec3(1,0,0));

			pNode->joint[2]->connectBodies(dummy2, &(pNode->body));
			pNode->joint[2]->setPositionAndOrientation(SE3(), Inv(boneT));
			pNode->joint[2]->setAxis(Vec3(0,1,0));

			//pNode->joint[0]->connectBodies(parentBody, &(pNode->body));
			//pNode->joint[0]->setPositionAndOrientation(Inv(parentBoneTs)*P, Inv(boneT));
		}

		for( int i=0 ; i<len_joint_children; ++i)
			_createBodyAndJoint(joint.attr("children")[i], posture, &(pNode->body), boneT);
	}
	//for( int i=0 ; i<len_joint_children; ++i)
		//_createBody(joint.attr("children")[i], posture);
}


//int GearModel::getParentIndex( int index )
//{
//	object parent = _skeleton.attr("getParentJointIndex")(index);
//	if(parent==object())
//		return -1;
//	else
//		return XI(parent);
//}

std::string GearModel::__str__()
{
	std::stringstream ss;
	ss << "<NODES>" << std::endl;
	for(size_t i=0; i<_nodes.size(); ++i)
	{
		ss << "[" << i << "]:";
		//if(_nodes[i]==NULL)
			//ss << "NULL, ";
		//else
		ss << _nodes[i]->name << ", ";
	}
	ss << std::endl;

	//ss << "<BODIES INDEX:(NODE INDEX) NODE NAME>\n";
	//for(int i=0; i<_bodyElementIndexes.size(); ++i)
		//ss << "[" << i << "]:(" << _bodyElementIndexes[i] << ") " << _nodes[_bodyElementIndexes[i]]->name << ", ";
	//ss << std::endl;

	ss << "<BODIES (,JOINTS)>" << std::endl;
	for(size_t i=0; i<_nodes.size(); ++i)
		ss << "[" << i << "]:" << _nodes[i]->name << ", ";
	ss << std::endl;

	ss << "<BODY MASSES>" << std::endl;
	for(size_t i=0; i<_nodes.size(); ++i)
	//if(_nodes[i])
		ss << "[" << i << "]:" << _nodes[i]->body.I.GetMass() << ", ";
	ss << std::endl;

	//ss << "<BODY INERTIAS>" << endl;
	//ss << "I11 I22 I33 I12 I13 I23 offset.x offset.y offset.z mass" << endl;
	//for(int i=0; i<_nodes.size(); ++i)
		//if(_nodes[i])
		//{
			//ss << "[" << i << "]:";
			//for(int j=0; j<10; ++j)
				//ss << _nodes[i]->body.GetInertia()[j] << " ";
			//ss << endl;
		//}
	//ss << endl;

	return ss.str();
}

bp::list GearModel::getBodyMasses()
{
	bp::list ls;
	for(size_t i=0; i<_nodes.size(); ++i)
		ls.append(_nodes[i]->body.getMass());
	return ls;
}

gReal GearModel::getTotalMass()
{
	gReal mass = 0.;
	for(size_t i=0; i<_nodes.size(); ++i)
		mass += _nodes[i]->body.getMass();
	return mass;
}

object GearModel::getBodyShape(int index)
{
	static numeric::array O(make_tuple(0.,0.,0.));
	char type;
	gReal data[3];

	_nodes[index]->body.getShape(&type, data);

	object pyV = O.copy();
	pyV[0] = data[0];
	pyV[1] = data[1];
	pyV[2] = data[2];

	return pyV;
}

bp::list GearModel::getBodyVerticesPositionGlobal(int index)
{
	static numeric::array O(make_tuple(0.,0.,0.));

	HpGBody *pBody;
	char type;
	gReal data[3];

	bp::list ls_point;

	pBody = &(_nodes[index]->body);
	pBody->getShape(&type, data);
	SE3 geomFrame = pBody->getGeomGlobalFrame();

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

//bp::list GearModel::getBodyPoints()
//{
//	bp::list ls;
//	char type;
//	gReal data[3];
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

//bp::list GearModel::getBodyShapes()
//{
//	bp::list ls;
//	char type;
//	gReal data[3];
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

void GearModel::getBodyInertiaLocal(int index, SE3& Tin)
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

	const Inertia in = _nodes[index]->body.I;

	Tin[0] = in._I[0];
	Tin[4] = in._I[1];
	Tin[8] = in._I[2];
	Tin[3] = Tin[1] = in._I[3];
	Tin[6] = Tin[2] = in._I[4];
	Tin[7] = Tin[5] = in._I[5];
}

boost::python::object GearModel::getBodyInertiaLocal_py( int index )
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static SE3 Tin;
	object pyIn = I.copy();

	getBodyInertiaLocal(index, Tin);
	SE3_2_pySO3(Tin, pyIn);
	return pyIn;
}

boost::python::object GearModel::getBodyInertiaGlobal_py( int index )
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static SE3 Tin_local, bodyFrame;
	object pyIn = I.copy();

	getBodyInertiaLocal(index, Tin_local);
	bodyFrame = _nodes[index]->body.getPoseGlobal();
	SE3_2_pySO3(bodyFrame * Tin_local * Inv(bodyFrame), pyIn);
	return pyIn;
	
}

bp::list GearModel::getBodyInertiasLocal()
{
	bp::list ls;
	for(size_t i=0; i<_nodes.size(); ++i)
		ls.append(getBodyInertiaLocal_py(i));
	return ls;
}

bp::list GearModel::getBodyInertiasGlobal()
{
	bp::list ls;
	for(size_t i=0; i<_nodes.size(); ++i)
		ls.append(getBodyInertiaGlobal_py(i));
	return ls;
}

object GearModel::getBodyPositionGlobal_py( int index, const object& positionLocal/*=object() */ )
{
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	static Vec3 positionLocal_;

	if(positionLocal==object())
		Vec3_2_pyVec3(getBodyPositionGlobal(index), pyV);
	else
	{
		pyVec3_2_Vec3(positionLocal, positionLocal_);
		Vec3_2_pyVec3(getBodyPositionGlobal(index, &positionLocal_), pyV);
	}
	return pyV;
}
object GearModel::getBodyVelocityGlobal_py( int index, const object& positionLocal/*=object() */ )
{
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	static Vec3 positionLocal_;
	
	if(positionLocal==object())
		Vec3_2_pyVec3(getBodyVelocityGlobal(index), pyV);
	else
	{
		pyVec3_2_Vec3(positionLocal, positionLocal_);
		Vec3_2_pyVec3(getBodyVelocityGlobal(index, positionLocal_), pyV);
	}
	return pyV;
}

bp::list GearModel::getBodyVelocitiesGlobal()
{
	bp::list ls;
	for(size_t i=0; i<_nodes.size(); ++i)
		ls.append(getBodyVelocityGlobal_py(i));
	return ls;
}

object GearModel::getBodyAngVelocityGlobal( int index )
{
	static se3 genVel;
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genVel = _nodes[index]->body.getVelocityGlobal();
	pyV[0] = genVel[0];
	pyV[1] = genVel[1];
	pyV[2] = genVel[2];
	return pyV;
}

bp::list GearModel::getBodyAngVelocitiesGlobal()
{
	bp::list ls;
	for(size_t i=0; i<_nodes.size(); ++i)
		ls.append(getBodyAngVelocityGlobal(i));
	return ls;
}

object GearModel::getBodyAccelerationGlobal_py(int index, const object& positionLocal )
{
	static se3 genAcc;
	static Vec3 positionLocal_;
	static numeric::array O(make_tuple(0.,0.,0.));
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

bp::list GearModel::getBodyAccelerationsGlobal()
{
	bp::list ls;
	for(size_t i=0; i<_nodes.size(); ++i)
		ls.append(getBodyAccelerationGlobal_py(i));
	return ls;
}

bp::list GearModel::getBodyPositionsGlobal()
{
	bp::list ls;
	for(size_t i=0; i<_nodes.size(); ++i)
		ls.append(getBodyPositionGlobal_py(i));
	return ls;
}

object GearModel::getBodyAngAccelerationGlobal( int index )
{
	static se3 genAcc;
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genAcc = _nodes[index]->body.getAccelerationGlobal();
	pyV[0] = genAcc[0];
	pyV[1] = genAcc[1];
	pyV[2] = genAcc[2];
	return pyV;
}

bp::list GearModel::getBodyAngAccelerationsGlobal()
{
	bp::list ls;
	for(size_t i=0; i<_nodes.size(); ++i)
		ls.append(getBodyAngAccelerationGlobal(i));
	return ls;
}

void GearModel::translateByOffset( const object& offset )
{
	static Vec3 v;
	pyVec3_2_Vec3(offset, v);
	std::vector <gReal> jointValues;
	
	//is it right?
	Vec3 localv = _nodes[0]->body.getOrientationGlobal() * v;
	getJointData(JOINT_VALUE, jointValues);
	for(int i=0; i<3; i++)
		jointValues[i] += localv[i];
	setJointData(JOINT_VALUE, jointValues);
}

//TODO:
void GearModel::rotate( const object& rotation )
{
	static SE3 R, bodyFrame;
	pySO3_2_SE3(rotation, R);

	//bodyFrame = _nodes[0]->body.GetFrame();
	//_nodes[0]->body.SetFrame(bodyFrame * R);

	// ?Ù²? root body frame?? ???? joint?? ?????? ?????? body?? frame ??????Æ®. ?Ì°?À» ???? ??À¸?? root body ?Ï³??? rotation?? ?????È´?. 
	//_pWorld->UpdateFrame();
}

Vec3 GearModel::getBodyPositionGlobal( int index, const Vec3* pPositionLocal )
{
	if(!pPositionLocal)
		return _nodes[index]->body.getPositionGlobal();
	else
		return _nodes[index]->body.getPositionGlobal((*pPositionLocal));
}

Vec3 GearModel::getBodyVelocityGlobal( int index, const Vec3& positionLocal)
{
	return _nodes[index]->body.getVelocityLinearGlobal(positionLocal);
}

Vec3 GearModel::getBodyAccelerationGlobal( int index, const Vec3* pPositionLocal)
{
	static Vec3 acc;
	if(pPositionLocal)
		acc = _nodes[index]->body.getAccelerationLinearGlobal((*pPositionLocal));
	else
		acc = _nodes[index]->body.getAccelerationLinearGlobal();
	return acc;
	
}
void GearModel::getJointData(JointDataType t, std::vector<gReal> &values)
{
	//values
	//p[0] p[1] p[2] eulerZ eulerX eulerY theta[0] theta[1] .....
	values.clear();
	//values.resize(_nodes.size()*3+3);

	double imag;
	Vec3 v,p;
	{
		GJointFreeTS* j=(GJointFreeTS*)(&modelJointFree);
		double temp[6];
		GBody* body=(GBody*)(j->pRightBody);
		se3& dotSdq = j->dSdq;
		RMatrix ddq(6,1);
		RMatrix Sddq(6,1);
		switch(t)
		{
			case JOINT_VALUE:
				j->get_q(temp);
				//hwangpil
				//v.x : Z angle, v.y: X angle v.z: Y angle
				p[0]=temp[0]; p[1]=temp[1]; p[2]=temp[2];
				v[0]=temp[3]; v[1]=temp[4]; v[2]=temp[5];
				//getOrientation(&j->spherical_joint, &temp[3], q);
				//imag=q.w;
				//v.x=q.x; v.y=q.y; v.z=q.z;
				break;
			case JOINT_VEL:
#ifdef USE_GLOBAL_ANG_VEL
				v=j->spherical_joint.T.GetRotation()*j->Sdq.GetW();
				p=j->spherical_joint.T.GetRotation()*j->Sdq.GetV();
#else
				v=j->Sdq.GetW();
				p=j->Sdq.GetV();
#endif
				break;
			case JOINT_ACC:
				j->get_ddq(ddq.GetPtr());
				imag=0.0;
				//se3& dotSdq=j->get_dSdq();
				Sddq=j->S*ddq;
				v[0]=Sddq(0,0)+dotSdq[0];
				v[1]=Sddq(1,0)+dotSdq[1];
				v[2]=Sddq(2,0)+dotSdq[2];
				p[0]=Sddq(3,0)+dotSdq[3];
				p[1]=Sddq(4,0)+dotSdq[4];
				p[2]=Sddq(5,0)+dotSdq[5];
				break;
			case JOINT_TAU:
				v[0]=body->F[0];
				v[1]=body->F[1];
				v[2]=body->F[2];
				p[0]=body->F[3];
				p[1]=body->F[4];
				p[2]=body->F[5];
				break;
			default:
				assert(0);
		}
		for(size_t k=0; k<3; k++)values.push_back(p[k]);
		for(size_t k=0; k<3; k++)values.push_back(v[k]);
	}
	for(size_t i=1; i<_nodes.size(); i++)
	{
		for(size_t k=0; k<_nodes[i]->joint.size(); k++)
		{
/*
			if(_nodes[i]->joint[j].jointType == GJoint::GJOINT_FREE_TS)
			{
				GJointFreeTS* joint=(GJointFreeTS*)_nodes[i].joint[j];
				double temp[6];
				ASSERT(sRDOF-sTDOF==3);
				switch(t)
				{
					case JOINT_VALUE:
						quater q;
						j->get_q(temp);
						p.x=temp[0]; p.y=temp[1]; p.z=temp[2];
						getOrientation(&j->spherical_joint, &temp[3], q);
						imag=q.w;
						v.x=q.x; v.y=q.y; v.z=q.z;
						break;
					case JOINT_VEL:
						Vec3 p2, v2;
#ifdef USE_GLOBAL_ANG_VEL
						assert(false);
						v2=j->spherical_joint.T.GetRotation()*j->Sdq.GetW();
						p2=j->spherical_joint.T.GetRotation()*j->Sdq.GetV();
						v=toBase(v2);
						p=toBase(p2);
#else
						v=toBase(j->Sdq.GetW());
						p=toBase(j->Sdq.GetV());
#endif
						break;
					case JOINT_ACC:
						RMatrix ddq(6,1);
						RMatrix Sddq(6,1);
						j->get_ddq(ddq.GetPtr());
						imag=0.0;
						se3& dotSdq=j->get_dSdq();
						Sddq=j->S*ddq;
						v.x=Sddq(0,0)+dotSdq[0];
						v.y=Sddq(1,0)+dotSdq[1];
						v.z=Sddq(2,0)+dotSdq[2];
						p.x=Sddq(3,0)+dotSdq[3];
						p.y=Sddq(4,0)+dotSdq[4];
						p.z=Sddq(5,0)+dotSdq[5];
						break;
					case JOINT_TAU:
						GBodyRigid* body=(GBodyRigid*)(j->pRightBody);
						v.x=body->F[0];
						v.y=body->F[1];
						v.z=body->F[2];
						p.x=body->F[3];
						p.y=body->F[4];
						p.z=body->F[5];
						break;
					default:
						ASSERT(0);
				}
				out[sTDOF]=p.x;
				out[sTDOF+1]=p.y;
				out[sTDOF+2]=p.z;
				out[sRDOF]=imag;
				out[sRDOF+1]=v.x;
				out[sRDOF+2]=v.y;
				out[sRDOF+3]=v.z;
			}
			else if(b.mJoint->jointType==HRP_JOINT::BALL)
			{			
				GJointSpherical* j=(GJointSpherical*)cinfo[b.mJoint->jointStartId]->joint;
				int sRDOF=l.dofInfo.startR(i);

				switch(t)
				{
					case DynamicsSimulator::JOINT_VALUE:
						quater q;

						double temp[4];
						j->get_q(temp);
						getOrientation(j, temp, q);

						imag=q.w;
						v.x=q.x; v.y=q.y; v.z=q.z;
						break;
					case DynamicsSimulator::JOINT_VELOCITY:
						Vec3 angVel;
#ifdef USE_GLOBAL_ANG_VEL
						angVel=j->T.GetRotation()*j->Sdq.GetW();
#else
						angVel=j->Sdq.GetW();
#endif
						v=toBase(angVel);
						break;
					case DynamicsSimulator::JOINT_ACCELERATION:
						// incorrect. need to add dotS dq
						double temp[4];
						j->get_ddq(temp);
						imag=0.0;
						v.x=temp[0]; v.y=temp[1]; v.z=temp[2];

						matrix3 jacobian;
						getMat3(jacobian, j->S);
						jacobian.rotate(v);// to twist 
						break;
					case DynamicsSimulator::JOINT_TORQUE:
						double temp[4];
						j->get_tau(temp);
						imag=0.0;
						v.x=temp[0]; v.y=temp[1]; v.z=temp[2];

						matrix3 jacobian;
						getMat3(jacobian, j->S);
						jacobian.rotate(v);// to spatial 
						break;
					default:
						ASSERT(0);
				}


				out[sRDOF]=imag;
				out[sRDOF+1]=v.x;
				out[sRDOF+2]=v.y;
				out[sRDOF+3]=v.z;
			}
			else 
*/
			if(_nodes[i]->joint[k]->jointType == GJoint::GJOINT_REVOLUTE)
			{
				GJointRevolute* j=((GJointRevolute*)(_nodes[i]->joint[k]));
				switch(t)
				{
				case JOINT_VALUE:
					values.push_back(j->coordinate.q);
					break;
				case JOINT_VEL:
					values.push_back(j->coordinate.dq);
					break;
				case JOINT_ACC:
					values.push_back(j->coordinate.ddq);
					break;
				case JOINT_TAU:
					values.push_back(j->coordinate.tau);
					break;
				default:
					assert(0);
				}
			}
/*
			else if(b.mJoint->jointType==HRP_JOINT::SLIDE)
			{			
				int sj=b.mJoint->jointStartId;
				int sDOF=l.dofInfo.startT(i);
				int nDOF=l.dofInfo.endR(i)-sDOF;



				for(int jj=0; jj<nDOF; jj++)
				{

					GJointPrismatic* j=((GJointPrismatic*)cinfo[jj+sj]->joint);
					switch(t)
					{
						case DynamicsSimulator::JOINT_VALUE:
							out[sDOF+jj]=j->coordinate.q;
							break;
						case DynamicsSimulator::JOINT_VELOCITY:
							out[sDOF+jj]=j->coordinate.dq;
							break;
						case DynamicsSimulator::JOINT_ACCELERATION:
							out[sDOF+jj]=j->coordinate.ddq;
							break;
						case DynamicsSimulator::JOINT_TORQUE:
							out[sDOF+jj]=j->coordinate.tau;
							break;

						default:
							ASSERT(0);
					}
				}
			}
*/
		}
	}
}
void GearModel::setJointData(JointDataType t, std::vector<gReal> &values)
{
	int valueIdx=0;

	{
		GJointFreeTS* j=(GJointFreeTS*)(&modelJointFree);
		double zero[]={0,0,0,0,0,0};	
		if (t==JOINT_VALUE)
		{
			// the execution path depends on the current value. 
			// to break such dependency, and to guarantee that the same input yields the same output,
			// invalidate spherical joint first.
			//j->spherical_joint.coord_chart=0;
			j->spherical_joint.coord_chart =
				GJointSpherical::EULER_ZXY;
				//CoordinateChartForSphericalJoint::EULER_ZXY;
			j->set_q(zero);
			j->update_short();
		}
		else if(t==JOINT_VEL) j->set_dq(zero);
	}
	{
		GJointFreeTS* j=(GJointFreeTS*)(&modelJointFree);
		double temp[6];
		Vec3 v, p;
		p[0] = values[valueIdx++];
		p[1] = values[valueIdx++];
		p[2] = values[valueIdx++];
		v[0] = values[valueIdx++];
		v[1] = values[valueIdx++];
		v[2] = values[valueIdx++];
		Vec3 euler;
		Vec3 W, V;
		SO3 invT_global;
		RMatrix Sdq(6,1);
		RMatrix Sddq(6,1);
		RMatrix invS, dq, ddq;
		GBody* body=(GBody*)(&(_nodes[0]->body));
		//se3& dotSdq=j->dSdq;
		switch(t)
		{
			case JOINT_VALUE:

				//calcEulerFromQuater(&j->spherical_joint, q, euler);

				temp[3]=v[0];	temp[4]=v[1];	temp[5]=v[2];
				temp[0]=p[0];	temp[1]=p[1];	temp[2]=p[2];

				j->set_q(temp);
				j->update_short();
				break;

			case JOINT_VEL:
#ifdef USE_GLOBAL_ANG_VEL
				// body velocity
				// w=invR*wo
				// v=invR*vo
				// where [w,v]'=Sdq
				invT_global=j->spherical_joint.inv_T.GetRotation();
				W=invT_global*v;
				V=invT_global*p;
#else
				W=v;
				V=p;
				//printf("W,V=%s,%s\n", in.toVector3(sRDOF+1).output().ptr(), p.output().ptr());
#endif
				Sdq(0,0)=W[0];
				Sdq(1,0)=W[1];
				Sdq(2,0)=W[2];
				Sdq(3,0)=V[0];
				Sdq(4,0)=V[1];
				Sdq(5,0)=V[2];
				invS=Inv(j->S);
				dq=invS*Sdq;
				j->set_dq(dq);

				break;
			case JOINT_ACC:
#ifdef USE_GLOBAL_ANG_VEL
				// not implemented yet!
				// body velocity: v = (w,V)'
				// body acceleration: dv
				// world acceleration: dvo (not spatial acceleration)
				// vo=Rv --(1)
				// -> v=invR vo --(2)
				// derivate both side of (2)
				// dv=invR*dvo + dotinvR* vo
				// dv=invR*dvo + dotR'* vo
				// 	 =invR*dvo + (R*skew(wo))'*vo
				// 	 =invR*dvo - skew(wo)*invR*vo  --(3)
				//
				// verification: derivate both side of (1)
				// dv=invR(dvo - dotR v) 
				//   =invR(dvo - R*skew(wo)*v) 
				//   =invR*dvo - skew(wo)*invR*vo  --(5)
				//					// Of course, (3)==(4)
				//

				SO3 invR=j->spherical_joint.inv_T.GetRotation();
				SO3 R=j->spherical_joint.T.GetRotation();
				j->update(); //calculate dS, dSdq
				se3 vo=mult(R, j->Sdq);
				se3 dvo(v,p);

				se3 dv=mult(invR, dvo);
				dv-=mult(skew(vo.GetW())*invR, vo);
				// ddq=invS*dv - dotS_dq	
				RMatrix ddq=Inv(j->S)*toRMat(dv);
				ddq-=toRMat(j->dSdq);
#else
				W=v;
				V=p;
				// v=S*dq where dq is euler angle speeds
				// dv= dotS*dq+ S*ddq
				j->update(); //calculate dS, dSdq
				{
					RMatrix Sddq(6,1);
					se3 dotSdq=j->dSdq;
					Sddq(0,0)=W[0]-dotSdq[0];
					Sddq(1,0)=W[1]-dotSdq[1];
					Sddq(2,0)=W[2]-dotSdq[2];
					Sddq(3,0)=V[0]-dotSdq[3];
					Sddq(4,0)=V[1]-dotSdq[4];
					Sddq(5,0)=V[2]-dotSdq[5];
					//Sddq(0,0)=V[0];
					//Sddq(1,0)=V[1];
					//Sddq(2,0)=V[2];
					//Sddq(3,0)=W[0];
					//Sddq(4,0)=W[1];
					//Sddq(5,0)=W[2];
					invS=Inv(j->S);
					ddq=invS*Sddq;
					ddq = Sddq;
				}
#endif
				j->set_ddq(ddq.element);
				break;
			case JOINT_TAU:
				for(int k=0; k<6; k++) temp[k]=0;
				j->set_tau(temp);
				body->addExternalForceLocally(dse3(v, p));
				break;
			default:
				assert(0);
		}
	}
	//double imag;
	Vec3 p,v;

	for(size_t i=1; i<_nodes.size(); i++)
	{
		for(size_t k=0; k<_nodes[i]->joint.size(); k++)
		{
/*
			if(b.mJoint->jointType==HRP_JOINT::FREE)
			{
				GJointFreeC2* j=(GJointFreeC2* )cinfo[b.mJoint->jointStartId]->joint;

				ASSERT(l.dofInfo.hasTranslation(i));
				ASSERT(l.dofInfo.hasQuaternion(i));
				int sTDOF=l.dofInfo.startT(i);
				int sRDOF=l.dofInfo.startR(i);
				ASSERT(sRDOF-sTDOF==3);
				p=in.toVector3(sTDOF);

				double temp[6];

				switch(t)
				{
					case DynamicsSimulator::JOINT_VALUE:
						{
							Vec3 euler;
							q=in.toQuater(sRDOF);

							calcEulerFromQuater(&j->spherical_joint, q, euler);

							temp[3]=euler[0];	temp[4]=euler[1];	temp[5]=euler[2];
							temp[0]=p.x;		temp[1]=p.y;		temp[2]=p.z;

							j->set_q(temp);
							j->update_short();
						}
						break;

					case DynamicsSimulator::JOINT_VELOCITY:
						{
							Vec3 W, V;
#ifdef USE_GLOBAL_ANG_VEL
							// body velocity
							// w=invR*wo
							// v=invR*vo
							// where [w,v]'=Sdq
							SO3 invT_global=j->spherical_joint.inv_T.GetRotation();

							W=invT_global*toGMBS(in.toVector3(sRDOF+1));
							V=invT_global*toGMBS(p);
#else
							W=toGMBS(in.toVector3(sRDOF+1));
							V=toGMBS(p);
							//printf("W,V=%s,%s\n", in.toVector3(sRDOF+1).output().ptr(), p.output().ptr());
#endif
							RMatrix Sdq(6,1);
							Sdq(0,0)=W[0];
							Sdq(1,0)=W[1];
							Sdq(2,0)=W[2];
							Sdq(3,0)=V[0];
							Sdq(4,0)=V[1];
							Sdq(5,0)=V[2];
							RMatrix invS=Inv(j->S);
							RMatrix dq=invS*Sdq;
							j->set_dq(dq);

						}
						break;
					case DynamicsSimulator::JOINT_ACCELERATION:
						{
#ifdef USE_GLOBAL_ANG_VEL
							// not implemented yet!
							// body velocity: v = (w,V)'
							// body acceleration: dv
							// world acceleration: dvo (not spatial acceleration)
							// vo=Rv --(1)
							// -> v=invR vo --(2)
							// derivate both side of (2)
							// dv=invR*dvo + dotinvR* vo
							// dv=invR*dvo + dotR'* vo
							// 	 =invR*dvo + (R*skew(wo))'*vo
							// 	 =invR*dvo - skew(wo)*invR*vo  --(3)
							//
							// verification: derivate both side of (1)
							// dv=invR(dvo - dotR v) 
							//   =invR(dvo - R*skew(wo)*v) 
							//   =invR*dvo - skew(wo)*invR*vo  --(5)
							//					// Of course, (3)==(4)
							//

							SO3 invR=j->spherical_joint.inv_T.GetRotation();
							SO3 R=j->spherical_joint.T.GetRotation();
							j->update(); //calculate dS, dSdq
							se3 vo=mult(R, j->Sdq);
							se3 dvo(toGMBS(v),toGMBS(p));

							se3 dv=mult(invR, dvo);
							dv-=mult(skew(vo.GetW())*invR, vo);
							// ddq=invS*dv - dotS_dq	
							RMatrix ddq=Inv(j->S)*toRMat(dv);
							ddq-=toRMat(j->dSdq);
#else
							Vec3 W,V;
							W=toGMBS(in.toVector3(sRDOF+1));
							V=toGMBS(p);
							// v=S*dq where dq is euler angle speeds
							// dv= dotS*dq+ S*ddq
							j->update(); //calculate dS, dSdq
							RMatrix Sddq(6,1);
							se3& dotSdq=j->get_dSdq();
							Sddq(0,0)=W[0]-dotSdq[0];
							Sddq(1,0)=W[1]-dotSdq[1];
							Sddq(2,0)=W[2]-dotSdq[2];
							Sddq(3,0)=V[0]-dotSdq[3];
							Sddq(4,0)=V[1]-dotSdq[4];
							Sddq(5,0)=V[2]-dotSdq[5];
							RMatrix invS=Inv(j->S);
							RMatrix ddq=invS*Sddq;
#endif
							j->set_ddq(ddq.element);
						}
						break;
					case DynamicsSimulator::JOINT_TORQUE:
						{
							v=in.toVector3(sRDOF+1);
							for(int i=0; i<6; i++)
								temp[i]=0;
							j->set_tau(temp);
							GBodyRigid* body=getBody(cinfo,&b);
							body->addExternalForceLocally(dse3(toGMBS(v), toGMBS(p)));
						}
						break;
					default:
						ASSERT(0);
				}
			}
			else if(b.mJoint->jointType==HRP_JOINT::BALL)
			{
				GJointSpherical* j=(GJointSpherical*)cinfo[b.mJoint->jointStartId]->joint;
				ASSERT(l.dofInfo.hasQuaternion(i));
				int sRDOF=l.dofInfo.startR(i);
				imag=in[sRDOF];
				v=in.toVector3(sRDOF+1);

				double temp[3];
				switch(t)
				{
					case DynamicsSimulator::JOINT_VALUE:
						{
							Vec3 euler;
							q=in.toQuater(sRDOF);

							calcEulerFromQuater(j, q, euler);

							temp[0]=euler[0];	temp[1]=euler[1];	temp[2]=euler[2];

							j->set_q(temp);
							j->update_short();
						}
						break;
					case DynamicsSimulator::JOINT_VELOCITY:
						{
							Vec3 W;
#ifdef USE_GLOBAL_ANG_VEL
							SO3 invT_global=j->inv_T.GetRotation();
							W=invT_global*toGMBS(v);
#else
							W=toGMBS(v);
#endif
							RMatrix Sdq(3,1);
							Sdq(0,0)=W[0];
							Sdq(1,0)=W[1];
							Sdq(2,0)=W[2];
							RMatrix invS=Inv(j->S.Sub(0,2,0,2));
							//					cout << "W"<<W<<"\n";
							//					cout << "invS"<<invS<<"\n";
							RMatrix dq=invS*Sdq;
							//					cout << "dq"<<dq<<"\n";
							//					cout << j->S*dq<<Sdq<<"\n";
							j->set_dq(dq);
							   //matrix3 jacobianInv;
							   //getMat3(jacobianInv, j->S);
							   //jacobianInv.inverse();
							   //jacobianInv.rotate(v);
							   //temp[0]=v.x;	temp[1]=v.y;	temp[2]=v.z;					
							   //j->set_dq(temp);

						}
						break;
					case DynamicsSimulator::JOINT_ACCELERATION:
						{
							matrix3 jacobianInv;
							getMat3(jacobianInv, j->S);
							jacobianInv.inverse();
							jacobianInv.rotate(v);
							temp[0]=v.x;	temp[1]=v.y;	temp[2]=v.z;					

							j->set_ddq(temp);
						}
						break;
					case DynamicsSimulator::JOINT_TORQUE:
						{
							matrix3 jacobianInv;
							getMat3(jacobianInv, j->S);
							jacobianInv.inverse();
							jacobianInv.rotate(v);
							temp[0]=v.x;	temp[1]=v.y;	temp[2]=v.z;					

							j->set_tau(temp);
						}
						break;
					default:	
						ASSERT(0);
				}
			}
			else 
*/
			if(_nodes[i]->joint[k]->jointType == GJoint::GJOINT_REVOLUTE)
			{
				GJointRevolute* j=((GJointRevolute*)(_nodes[i]->joint[k]));
				
				switch(t)
				{
					case JOINT_VALUE:
						//std::cout << values[valueIdx] << " " << j->getAxis();
						j->coordinate.q=values[valueIdx++];
						break;
					case JOINT_VEL:
						j->coordinate.dq=values[valueIdx++];
						break;
					case JOINT_ACC:
						j->coordinate.ddq=values[valueIdx++];
						break;
					case JOINT_TAU:
						j->coordinate.tau=values[valueIdx++];
						break;

					default:
						assert(0);
				}
			}
/*
			else if(b.mJoint->jointType==HRP_JOINT::SLIDE)
			{
				int sj=b.mJoint->jointStartId;
				int sDOF=l.dofInfo.startT(i);
				int nDOF=l.dofInfo.endR(i)-sDOF;

				for(int jj=0; jj<nDOF; jj++)
				{
					GJointPrismatic* j=((GJointPrismatic*)cinfo[jj+sj]->joint);
					switch(t)
					{
						case DynamicsSimulator::JOINT_VALUE:
							j->coordinate.q=in[sDOF+jj];
							break;
						case DynamicsSimulator::JOINT_VELOCITY:
							j->coordinate.dq=in[sDOF+jj];
							break;
						case DynamicsSimulator::JOINT_ACCELERATION:
							j->coordinate.ddq=in[sDOF+jj];
							break;
						case DynamicsSimulator::JOINT_TORQUE:
							j->coordinate.tau=in[sDOF+jj];
							break;

						default:
							ASSERT(0);
					}

				}
			}
*/
		}
	}

	if(t!=JOINT_TAU)
	{
		GJoint* j = (GJoint*)(&modelJointFree);
		if( !j->isConstantScrew() ) j->update_short();
		//for(int i=1; i<_nodes.size(); i++)
		//{
			//for(int k=0; k<_nodes[i]->joint.size(); k++)
			//{
				//GJointRevolute *j = (GJointRevolute*)(&_nodes[i]->joint[k]);
				//if( !j->isConstantScrew() ) 
				//{
					//j->update_short();
				//}
			//}
		//}
	}
}

boost::python::object GearModel::getBodyOrientationGlobal( int index )
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static SE3 bodyFrame;
	object pyR = I.copy();

	// body frame?? Inv(boneT)?? ???? joint frame ???Ñ´?
	bodyFrame = _nodes[index]->body.getPoseGlobal();
	SE3_2_pySO3(bodyFrame, pyR);
	return pyR;
}

void GearModel::update( const object& posture)
{
	std::vector<gReal> _values;
	object joint = posture.attr("skeleton").attr("root");
	//object rootPos = posture.attr("rootPos");
	//SE3 T= SE3(pyVec3_2_Vec3(rootPos));
	_updateJoint(joint, posture, _values);
	//for(size_t i=0; i<_values.size(); i++)
		//_values[i] = 0.;
	setJointData(JOINT_VALUE, _values);
	
	_pWorld->_system.updateKinematics();
	//getJointData(JOINT_VALUE, _values);
	//for(size_t i=0; i<_values.size(); i++)
		//std::cout << (180./M_PI) * _values[i] << " ";
	//std::cout << std::endl;

}

static Vec3 invEulerZXY(const SO3 &R)
{
	double sx = R[5];
	double radX = asin(sx);
	double radZ = atan2(-R[3], R[4]);
	double radY = atan2(-R[2], R[8]);

	//return Vec3(atan2(-R._R[3], R._R[4]), atan2(R._R[5], sqrt(R._R[3]*R._R[3]+R._R[4]*R._R[4])), atan2(-R._R[2], R._R[8]));
	//return Vec3(radZ, atan2(sx, cx), atan2(-R[2], R[8]));
	return Vec3(radZ,radX,radY);
}
void GearModel::_updateJoint( const object& joint, const object& posture, std::vector<gReal> &_values)
{
	for(size_t i=0; i<_nodes.size(); i++)
	{
		string joint_name = _nodes[i]->name;
		int joint_index = XI(posture.attr("skeleton").attr("getJointIndex")(joint_name));
		SE3 R = pySO3_2_SE3(posture.attr("getJointOrientationLocal")(joint_index));
		if (_config.attr("hasNode")(joint_name))
		{
			if(!joint_name.compare("Hips"))
			{
				object rootPos = posture.attr("rootPos");
				Vec3 rootPos_Vec3= pyVec3_2_Vec3(rootPos);
				_values.push_back(rootPos_Vec3[0]);
				_values.push_back(rootPos_Vec3[1]);
				_values.push_back(rootPos_Vec3[2]);
			}
			SO3 R_SO3 = R.GetRotation();
			Vec3 eulerZXY = invEulerZXY(R_SO3);
			//std::cout << joint_name << " " << (180./M_PI) * eulerZXY;
			_values.push_back(eulerZXY[0]);
			_values.push_back(eulerZXY[1]);
			_values.push_back(eulerZXY[2]);
		}
	}
}


GearControlModel::GearControlModel( GearWorld* pWorld, const object& createPosture, const object& config )
	:GearModel(pWorld, createPosture, config)
{
	//addBodiesToWorld(createPosture);
	//ignoreCollisionBtwnBodies(); //don't need in gear

	//object tpose = createPosture.attr("getTPose")();
	//createJoints(tpose);

	//update(createPosture);
}

std::string GearControlModel::__str__()
{
	std::string s1 = GearModel::__str__();

	std::stringstream ss;

	ss << "<INTERNAL JOINTS>" << std::endl;
	for(size_t i=1; i<_nodes.size(); ++i)
		ss << "[" << i-1 << "]:" << _nodes[i]->name << ", ";
	ss << std::endl;

	return s1 + ss.str();
}

bp::list GearControlModel::getInternalJointDOFs()
{
	bp::list ls;
	for(size_t i=1; i<_nodes.size(); ++i)
		ls.append(3);
	return ls;
}

int GearControlModel::getTotalInternalJointDOF()
{
	int dof = 0;
	for(size_t i=1; i<_nodes.size(); ++i)
		dof += 3;
	return dof;
}

bp::list GearControlModel::getDOFs()
{
	bp::list ls;
	ls.append(6);
	for(size_t i=1; i<_nodes.size(); ++i)
		ls.append(3);
	return ls;
}
int GearControlModel::getTotalDOF()
{
	int dof = 0;
	dof += 6;
	for(size_t i=1; i<_nodes.size(); ++i)
		dof += 3;
	return dof;
}

void GearControlModel::initializeHybridDynamics(bool floatingBase)
{
	int rootIndex = 0;
	
	for(size_t i=0; i<_nodes.size(); ++i)
	{
		if(i == 0)
		{
			if(floatingBase)
				modelJointFree.setPrescribed(false);
			else
				modelJointFree.setPrescribed(true);
		}
		else
		{
			for(size_t j=0; j<_nodes[i]->joint.size(); j++)
				_nodes[i]->joint[j]->setPrescribed(true);
		}
	}
}

void GearControlModel::solveHybridDynamics()
{
	_pWorld->_system.calcDynamics();	
}

bp::list GearControlModel::getDOFPositions()
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static numeric::array O(make_tuple(0.,0.,0.));
	static SE3 rootFrame;

	object pyR = I.copy();
	object pyV = O.copy();

	bp::list ls = getInternalJointOrientationsLocal();

	rootFrame = _nodes[0]->body.getPoseGlobal() * Inv(_boneTs[0]);

	Vec3_2_pyVec3(rootFrame.GetPosition(), pyV);
	SE3_2_pySO3(rootFrame, pyR);

	ls.insert(0, make_tuple(pyV, pyR));
	return ls;
}

//bp::list GearControlModel::getDOFVelocities()
//{
	//static numeric::array rootGenVel(make_tuple(0.,0.,0.,0.,0.,0.));
	
	//rootGenVel.slice(0,3) = getJointVelocityGlobal(0);
	////rootGenVel.slice(3,6) = getJointAngVelocityGlobal(0);
	//rootGenVel.slice(3,6) = getJointAngVelocityLocal(0);

	//bp::list ls = getInternalJointAngVelocitiesLocal();
	//ls.insert(0, rootGenVel);
	//return ls;
//}

//bp::list GearControlModel::getDOFAccelerations()
//{
	//static numeric::array rootGenAcc(make_tuple(0.,0.,0.,0.,0.,0.));
	
	//rootGenAcc.slice(0,3) = getJointAccelerationGlobal(0);
////	rootGenAcc.slice(3,6) = getJointAngAccelerationGlobal(0);
	//rootGenAcc.slice(3,6) = getJointAngAccelerationLocal(0);

	//bp::list ls = getInternalJointAngAccelerationsLocal();

	//ls.insert(0, rootGenAcc);
	//return ls;
//}

//bp::list GearControlModel::getDOFAxeses()
//{
	//static numeric::array rootAxeses( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.),
										//make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	//numeric::array rootAxesTmp = (numeric::array)getJointOrientationGlobal(0);
	//numeric::array rootAxes = transpose_pySO3(rootAxesTmp);
	//rootAxeses[3] = rootAxes[0];
	//rootAxeses[4] = rootAxes[1];
	//rootAxeses[5] = rootAxes[2];

	//bp::list ls = getInternalJointOrientationsGlobal();
	//for(int i=0; i<len(ls); ++i)
	//{
		//numeric::array lsTmp = (numeric::array)ls[i];
		//ls[i] = transpose_pySO3(lsTmp);
	//}

	//ls.insert(0, rootAxeses);
	//return ls;
//}

//bp::list GearControlModel::getDOFPositionsLocal()
//{
	//static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	//static numeric::array O(make_tuple(0.,0.,0.));
	//static SE3 rootFrame;

	//object pyR = I.copy();
	//object pyV = O.copy();

	//bp::list ls = getInternalJointOrientationsLocal();

	//rootFrame = _nodes[0]->body.GetFrame() * Inv(_boneTs[0]);

	//Vec3_2_pyVec3(-Inv(rootFrame).GetPosition(), pyV);
	//SE3_2_pySO3(rootFrame, pyR);

	//ls.insert(0, make_tuple(pyV, pyR));
	//return ls;
//}

//bp::list GearControlModel::getDOFVelocitiesLocal()
//{
	//static numeric::array rootGenVel(make_tuple(0.,0.,0.,0.,0.,0.));
	
	////rootGenVel.slice(0,3) = getJointVelocityGlobal(0);
	////rootGenVel.slice(3,6) = getJointAngVelocityGlobal(0);
	//rootGenVel.slice(0,3) = getJointVelocityLocal(0);
	//rootGenVel.slice(3,6) = getJointAngVelocityLocal(0);

	//bp::list ls = getInternalJointAngVelocitiesLocal();
	//ls.insert(0, rootGenVel);
	//return ls;
//}

//bp::list GearControlModel::getDOFAccelerationsLocal()
//{
	//static numeric::array rootGenAcc(make_tuple(0.,0.,0.,0.,0.,0.));
	
	////rootGenAcc.slice(0,3) = getJointAccelerationGlobal(0);
	////rootGenAcc.slice(3,6) = getJointAngAccelerationGlobal(0);
	//rootGenAcc.slice(0,3) = getJointAccelerationLocal(0);
	//rootGenAcc.slice(3,6) = getJointAngAccelerationLocal(0);

	//bp::list ls = getInternalJointAngAccelerationsLocal();

	//ls.insert(0, rootGenAcc);
	//return ls;
//}

//bp::list GearControlModel::getDOFAxesesLocal()
//{
	//static numeric::array rootAxeses( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.),
										//make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	//numeric::array rootAxesTmp = (numeric::array)getJointOrientationGlobal(0);
	//numeric::array rootAxes = transpose_pySO3(rootAxesTmp);
	//rootAxeses[0] = rootAxes[0];
	//rootAxeses[1] = rootAxes[1];
	//rootAxeses[2] = rootAxes[2];
	//rootAxeses[3] = rootAxes[0];
	//rootAxeses[4] = rootAxes[1];
	//rootAxeses[5] = rootAxes[2];

	//bp::list ls = getInternalJointOrientationsGlobal();
	//for(int i=0; i<len(ls); ++i)
	//{
		//numeric::array lsTmp = (numeric::array)ls[i];
		//ls[i] = transpose_pySO3(lsTmp);
	//}

	//ls.insert(0, rootAxeses);
	//return ls;
//}

bp::list GearControlModel::getDOFPositionsEuler()
{
	//static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static numeric::array O(make_tuple(0.,0.,0.));
	static numeric::array rootAcc(make_tuple(0.,0.,0.,0.,0.,0.));
	//static SE3 rootFrame;

	bp::list ls;
	std::vector<double> _values;
	getJointData(JOINT_VALUE, _values);
	object pyV = O.copy();
	double3_2_pyVec3(_values[0], _values[1], _values[2], pyV);
	rootAcc.slice(0,3) = pyV;
	pyV = O.copy();
	double3_2_pyVec3(_values[3], _values[4], _values[5], pyV);
	rootAcc.slice(3,6) = pyV;

	ls.append(rootAcc);
	for(size_t i=6; i<_values.size(); i+=3)
	{
		pyV = O.copy();
		double3_2_pyVec3(_values[i], _values[i+1], _values[i+2], pyV);
		ls.append(pyV);
	}
	
	return ls;
}

bp::list GearControlModel::getDOFVelocitiesEuler()
{
	//static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static numeric::array O(make_tuple(0.,0.,0.));
	static numeric::array rootAcc(make_tuple(0.,0.,0.,0.,0.,0.));
	//static SE3 rootFrame;

	bp::list ls;
	std::vector<double> _values;
	getJointData(JOINT_VEL, _values);
	object pyV = O.copy();
	double3_2_pyVec3(_values[0], _values[1], _values[2], pyV);
	rootAcc.slice(0,3) = pyV;
	pyV = O.copy();
	double3_2_pyVec3(_values[3], _values[4], _values[5], pyV);
	rootAcc.slice(3,6) = pyV;

	ls.append(rootAcc);
	for(size_t i=6; i<_values.size(); i+=3)
	{
		pyV = O.copy();
		double3_2_pyVec3(_values[i], _values[i+1], _values[i+2], pyV);
		ls.append(pyV);
	}
	
	return ls;
}

bp::list GearControlModel::getDOFAccelerationsEuler()
{
	//static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static numeric::array O(make_tuple(0.,0.,0.));
	//static SE3 rootFrame;

	bp::list ls;
	std::vector<double> _values;
	getJointData(JOINT_ACC, _values);
	object pyV = O.copy();
	double3_2_pyVec3(_values[0], _values[1], _values[2], pyV);
	ls.append(pyV);
	pyV = O.copy();
	double3_2_pyVec3(_values[3], _values[4], _values[5], pyV);
	ls.append(pyV);
	for(size_t i=6; i<_values.size(); i+=3)
	{
		pyV = O.copy();
		double3_2_pyVec3(_values[i], _values[i+1], _values[i+2], pyV);
		ls.append(pyV);
	}
	
	return ls;
}

//bp::list GearControlModel::getDOFAxesesEuler()
//{
	//static numeric::array rootAxeses( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.),
										//make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	//numeric::array rootAxesTmp = (numeric::array)getJointOrientationGlobal(0);
	//numeric::array rootAxes = transpose_pySO3(rootAxesTmp);
	//rootAxeses[0] = rootAxes[0];
	//rootAxeses[1] = rootAxes[1];
	//rootAxeses[2] = rootAxes[2];
	//rootAxeses[3] = rootAxes[0];
	//rootAxeses[4] = rootAxes[1];
	//rootAxeses[5] = rootAxes[2];

	//bp::list ls = getInternalJointOrientationsGlobal();
	//for(int i=0; i<len(ls); ++i)
	//{
		//numeric::array lsTmp = (numeric::array)ls[i];
		//ls[i] = transpose_pySO3(lsTmp);
	//}

	//ls.insert(0, rootAxeses);
	//return ls;
//}

//void GearControlModel::setDOFAccelerations( const bp::list& dofaccs)
//{
	//static numeric::array O(make_tuple(0.,0.,0.));

	//setJointAccelerationGlobal(0, dofaccs[0].slice(0,3));

////	setJointAngAccelerationGlobal(0, dofaccs[0].slice(3,6));
	//setJointAngAccelerationLocal(0, dofaccs[0].slice(3,6));

	//setInternalJointAngAccelerationsLocal( ((bp::list)dofaccs.slice(1,_)) );
//}

boost::python::object GearControlModel::getJointOrientationLocal( int index )
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	if(index == 0)
		return getJointOrientationGlobal(index);
	else
	{
		double angles[3];
		for(int i=0; i<3; i++)
			_nodes[index]->joint[i]->get_q(&angles[i]);
		Vec3 EulerAngles(angles);
		SO3 ori_SO3 = EulerZXY(EulerAngles);
		object pyR = I.copy();
		SE3_2_pySO3(SE3(ori_SO3), pyR);
		return pyR;
	}
}

//boost::python::object GearControlModel::getJointAngVelocityLocal( int index )
//{
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();

	//if(index == 0)
	//{
		//static se3 genVelBodyLocal, genVelJointLocal;
		//static SE3 bodyFrame;
		//bodyFrame = _nodes[index]->body.getPoseGlobal();

		////is it right?
		//genVelBodyLocal = InvRotate(bodyFrame, _nodes[index]->body.getVelocityGlobal());
		//genVelJointLocal = InvAd(Inv(_boneTs[index]), genVelBodyLocal);
////		genVelJointLocal = Ad(_boneTs[index], genVelBodyLocal);	// À­ ???Î°? ??Àº ????
		//pyV[0] = genVelJointLocal[0];
		//pyV[1] = genVelJointLocal[1];
		//pyV[2] = genVelJointLocal[2]; 
	//}
	//else
		////Vec3_2_pyVec3(_nodes[index]->joint.GetVelocity(), pyV);
	
	//return pyV;
//}

//boost::python::object GearControlModel::getJointAngAccelerationLocal( int index )
//{
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();

	//if(index == 0)
	//{
		//static se3 genAccBodyLocal, genAccJointLocal;

		//genAccBodyLocal = _nodes[index]->body.GetGenAccelerationLocal();
		//genAccJointLocal = InvAd(Inv(_boneTs[index]), genAccBodyLocal);
		//pyV[0] = genAccJointLocal[0]; 
		//pyV[1] = genAccJointLocal[1];
		//pyV[2] = genAccJointLocal[2]; 
	//}
	//else
		//Vec3_2_pyVec3(_nodes[index]->joint.GetAcceleration(), pyV);
	
	//return pyV;
//}

//object GearControlModel::getJointPositionGlobal( int index )
//{
	//static numeric::array O(make_tuple(0.,0.,0.));
	//static SE3 bodyFrame;
	//object pyV = O.copy();

	//// body frame?? Inv(boneT)?? ???? joint 'Ä¡ Ã£?Â´?.
	//bodyFrame = _nodes[index]->body.GetFrame();
	//Vec3_2_pyVec3((bodyFrame * Inv(_boneTs[index])).GetPosition(), pyV);
	//return pyV;

////	if(!_nodes[index])	// ?????? ???? parent joint frameÀ» Ã£?? offset??Å­ transformation ??Å²??.
////	{
////		static SE3 parentJointFrame;
////		static Vec3 offset;
//////		int parent = XI(_skeleton.attr("getParentIndex")(index));
////		int parent = XI(_skeleton.attr("getParentJointIndex")(index));
////		parentJointFrame = _nodes[parent]->body.GetFrame() * Inv(_boneTs[parent]);
////		offset = pyVec3_2_Vec3(_skeleton.attr("getOffset")(index));
////		Vec3_2_pyVec3(parentJointFrame * offset, pyV);
////	}
////	else	// ?????? ?Æ´? ???? body frame?? Inv(boneT)?? ???? joint À§Ä¡ Ã£?Â´?.
////	{
////		static SE3 bodyFrame;
////		bodyFrame = _nodes[index]->body.GetFrame();
////		Vec3_2_pyVec3((bodyFrame * Inv(_boneTs[index])).GetPosition(), pyV);
////	}
////	return pyV;
//}
//object GearControlModel::getJointVelocityGlobal( int index )
//{
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();

	//Vec3_2_pyVec3(getBodyVelocityGlobal(index, Inv(_boneTs[index]).GetPosition()), pyV);
	//return pyV;
//}
//object GearControlModel::getJointVelocityLocal( int index )
//{
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();
	//SE3 jointFrame = _nodes[index]->body.GetFrame() * Inv(_boneTs[index]);

	//Vec3_2_pyVec3(InvRotate(jointFrame, getBodyVelocityGlobal(index, Inv(_boneTs[index]).GetPosition())), pyV);
	//return pyV;
//}

//object GearControlModel::getJointAccelerationGlobal( int index )
//{
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();
	//Vec3 pospos = Inv(_boneTs[index]).GetPosition();

	//Vec3_2_pyVec3(getBodyAccelerationGlobal(index, &(pospos)), pyV);
	//return pyV;
//}

//object GearControlModel::getJointAccelerationLocal( int index )
//{
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();
	//Vec3 pospos = Inv(_boneTs[index]).GetPosition();
	//SE3 jointFrame = _nodes[index]->body.GetFrame() * Inv(_boneTs[index]);

	//Vec3_2_pyVec3(InvRotate(jointFrame, getBodyAccelerationGlobal(index, &(pospos))), pyV);
	//return pyV;
//}

boost::python::object GearControlModel::getJointOrientationGlobal( int index )
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static SE3 bodyFrame;
	object pyR = I.copy();

	bodyFrame = _nodes[index]->body.getPoseGlobal();
	SE3_2_pySO3(bodyFrame * Inv(_boneTs[index]), pyR);
	return pyR;
}

//boost::python::object GearControlModel::getJointAngVelocityGlobal( int index )
//{
	//return getBodyAngVelocityGlobal(index);

////	static numeric::array O(make_tuple(0.,0.,0.));
////	static Vec3 angVel, parentAngVel;
////	object pyV = O.copy();
////
////	angVel = _nodes[index]->body.GetAngVelocity();
////
////	int parentIndex = getParentIndex(index);
////	if(parentIndex==-1)
////		parentAngVel = Vec3(0.,0.,0.);
////	else
////		parentAngVel = _nodes[parentIndex]->body.GetAngVelocity();
////
////	Vec3_2_pyVec3(angVel - parentAngVel, pyV);
////	return pyV;
//}

//boost::python::object GearControlModel::getJointAngAccelerationGlobal( int index )
//{
	//return getBodyAngAccelerationGlobal(index);
//}

//bp::list GearControlModel::getJointOrientationsLocal()
//{
	//bp::list ls;
	//for(int i=0; i<_nodes.size(); ++i)
		//ls.append(getJointOrientationLocal(i));
	//return ls;
//}

//bp::list GearControlModel::getJointAngVelocitiesLocal()
//{
	//bp::list ls;
	//for(int i=0; i<_nodes.size(); ++i)
		//ls.append(getJointAngVelocityLocal(i));
	//return ls;
//}

//bp::list GearControlModel::getJointAngAccelerationsLocal()
//{
	//bp::list ls;
	//for(int i=0; i<_nodes.size(); ++i)
		//ls.append(getJointAngAccelerationLocal(i));
	//return ls;
//}

//bp::list GearControlModel::getJointPositionsGlobal()
//{
	//bp::list ls;
	//for(int i=0; i<_nodes.size(); ++i)
		//ls.append(getJointPositionGlobal(i));
	//return ls;
//}

//bp::list GearControlModel::getJointVelocitiesGlobal()
//{
	//bp::list ls;
	//for(int i=0; i<_nodes.size(); ++i)
		//ls.append(getJointVelocityGlobal(i));
	//return ls;
//}

//bp::list GearControlModel::getJointAccelerationsGlobal()
//{
	//bp::list ls;
	//for(int i=0; i<_nodes.size(); ++i)
		//ls.append(getJointAccelerationGlobal(i));
	//return ls;
//}

//bp::list GearControlModel::getJointOrientationsGlobal()
//{
	//bp::list ls;
	//for(int i=0; i<_nodes.size(); ++i)
		//ls.append(getJointOrientationGlobal(i));
	//return ls;
//}

//bp::list GearControlModel::getJointAngVelocitiesGlobal()
//{
	//bp::list ls;
	//for(int i=0; i<_nodes.size(); ++i)
		//ls.append(getJointAngVelocityGlobal(i));
	//return ls;
//}

//bp::list GearControlModel::getJointAngAccelerationsGlobal()
//{
	//bp::list ls;
	//for(int i=0; i<_nodes.size(); ++i)
		//ls.append(getJointAngAccelerationGlobal(i));
	//return ls;
//}

bp::list GearControlModel::getInternalJointOrientationsLocal()
{
	bp::list ls;
	for(size_t i=1; i<_nodes.size(); ++i)
		ls.append(getJointOrientationLocal(i));
	return ls;
}

//bp::list GearControlModel::getInternalJointAngVelocitiesLocal()
//{
	//bp::list ls;
	//for(int i=1; i<_nodes.size(); ++i)
		//ls.append(getJointAngVelocityLocal(i));
	//return ls;
//}

//bp::list GearControlModel::getInternalJointAngAccelerationsLocal()
//{
	//bp::list ls;
	//for(int i=1; i<_nodes.size(); ++i)
		//ls.append(getJointAngAccelerationLocal(i));
	//return ls;
//}

//bp::list GearControlModel::getInternalJointPositionsGlobal()
//{
	//bp::list ls;
////	for(int i=1; i<_jointElementIndexes.size(); ++i)
	//for(int i=1; i<_nodes.size(); ++i)
		//ls.append(getJointPositionGlobal(i));
	//return ls;
//}

//bp::list GearControlModel::getInternalJointOrientationsGlobal()
//{
	//bp::list ls;
	//for(int i=1; i<_nodes.size(); ++i)
		//ls.append(getJointOrientationGlobal(i));
	//return ls;
//}
//void GearControlModel::setJointAngVelocityLocal( int index, const object& angvel )
//{
	//if(index == 0)
	//{
		//static se3 genVelBodyLocal, genVelJointLocal;
		
		//genVelBodyLocal = _nodes[index]->body.GetGenVelocityLocal();

		//genVelJointLocal = InvAd(Inv(_boneTs[index]), genVelBodyLocal);
		//genVelJointLocal[0] = XD(angvel[0]);
		//genVelJointLocal[1] = XD(angvel[1]);
		//genVelJointLocal[2] = XD(angvel[2]); 

		//genVelBodyLocal = Ad(Inv(_boneTs[index]), genVelJointLocal);;
		//_nodes[index]->body.SetGenVelocityLocal(genVelBodyLocal);
	//}
	//else
		//_nodes[index]->joint.SetVelocity(pyVec3_2_Vec3(angvel));
//}

//void GearControlModel::setJointAngAccelerationLocal( int index, const object& angacc )
//{
	//if(index == 0)
	//{
		//static se3 genAccBodyLocal, genAccJointLocal;
		
		//genAccBodyLocal = _nodes[index]->body.GetGenAccelerationLocal();

		//genAccJointLocal = InvAd(Inv(_boneTs[index]), genAccBodyLocal);
		//genAccJointLocal[0] = XD(angacc[0]);
		//genAccJointLocal[1] = XD(angacc[1]);
		//genAccJointLocal[2] = XD(angacc[2]); 

		//genAccBodyLocal = Ad(Inv(_boneTs[index]), genAccJointLocal);;
		//_nodes[index]->body.SetGenAccelerationLocal(genAccBodyLocal);
	//}
	//else
		//_nodes[index]->joint.SetAcceleration(pyVec3_2_Vec3(angacc));
//}

//void GearControlModel::setJointAccelerationGlobal( int index, const object& acc )
//{
	//if(index == 0)
	//{
		//Vec3 pospos = Inv(_boneTs[index]).GetPosition();
		//setBodyAccelerationGlobal(index, pyVec3_2_Vec3(acc), &(pospos));
	//}
	//else
		//cout << "setJointAccelerationGlobal() : not completely implemented" << endl;
//}

//void GearControlModel::setJointAngAccelerationGlobal( int index, const object& angacc )
//{
	//setBodyAngAccelerationGlobal(index, angacc);
//}

//void GearControlModel::setJointAngAccelerationsLocal( const bp::list& angaccs )
//{
	//for(int i=0; i<_nodes.size(); ++i)
		//setJointAngAccelerationLocal(i, angaccs[i]);
//}

//void GearControlModel::setInternalJointAngAccelerationsLocal( const bp::list& angaccs )
//{
	//for(int i=1; i<_nodes.size(); ++i)
		//_nodes[i]->joint.SetAcceleration(pyVec3_2_Vec3(angaccs[i-1]));
//}

//boost::python::object GearControlModel::getJointTorqueLocal( int index )
//{
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();

	//if(index==0) return object();

	//Vec3_2_pyVec3(_nodes[index]->joint.GetTorque(), pyV);
	//return pyV;
//}

//bp::list GearControlModel::getInternalJointTorquesLocal()
//{
	//bp::list ls;
////	for(int i=1; i<_jointElementIndexes.size(); ++i)
	//for(int i=1; i<_nodes.size(); ++i)
		//ls.append(getJointTorqueLocal(i));
	//return ls;
//}

//void GearControlModel::setJointTorqueLocal( int index, const object& torque )
//{
////	int index = _jointElementIndexes[jointIndex];
	//_nodes[index]->joint.SetTorque(pyVec3_2_Vec3(torque));
//}

//void GearControlModel::setInternalJointTorquesLocal( const bp::list& torques )
//{
////	int index;
////	for(int i=1; i<_jointElementIndexes.size(); ++i)
////	{
////		index = _jointElementIndexes[i];
////		_nodes[index]->joint.SetTorque(pyVec3_2_Vec3(torques[i]));
////	}
	//for(int i=1; i<_nodes.size(); ++i)
		//_nodes[i]->joint.SetTorque(pyVec3_2_Vec3(torques[i-1]));
//}


//void GearControlModel::applyBodyGenForceGlobal( int index, const object& torque, const object& force, const object& positionLocal[>=object()<] )
//{
	//static Vec3 zero(0,0,0);
	//if(positionLocal==object())
		//_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), XD(force[0]), XD(force[1]), XD(force[2])), zero);
	//else
		//_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), XD(force[0]), XD(force[1]), XD(force[2])), pyVec3_2_Vec3(positionLocal));
//}

//void GearControlModel::applyBodyForceGlobal( int index, const object& force, const object& positionLocal[>=object()<] )
//{
	//static Vec3 zero(0,0,0);
	//if(positionLocal==object())
		//_nodes[index]->body.ApplyGlobalForce(dse3(0.,0.,0., XD(force[0]), XD(force[1]), XD(force[2])), zero);
	//else
		//_nodes[index]->body.ApplyGlobalForce(dse3(0.,0.,0., XD(force[0]), XD(force[1]), XD(force[2])), pyVec3_2_Vec3(positionLocal));
//}

//void GearControlModel::applyBodyTorqueGlobal( int index, const object& torque )
//{
	//static Vec3 zero(0,0,0);
	//_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), 0.,0.,0.), zero);
//}

//object GearControlModel::getBodyForceLocal( int index )
//{
	//static dse3 genForce;
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();

	//genForce = _nodes[index]->body.GetForce();
	//pyV[0] = genForce[3];
	//pyV[1] = genForce[4];
	//pyV[2] = genForce[5];
	//return pyV;

//}

//object GearControlModel::getBodyNetForceLocal( int index )
//{
	//static dse3 genForce;
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();

	//genForce = _nodes[index]->body.GetNetForce();
	//pyV[0] = genForce[3];
	//pyV[1] = genForce[4];
	//pyV[2] = genForce[5];
	//return pyV;
//}

//object GearControlModel::getBodyGravityForceLocal( int index )
//{
	//static dse3 genForce;
	//static numeric::array O(make_tuple(0.,0.,0.));
	//object pyV = O.copy();

	//genForce = _nodes[index]->body.GetGravityForce();
	//pyV[0] = genForce[3];
	//pyV[1] = genForce[4];
	//pyV[2] = genForce[5];
	//return pyV;
//}

static void ID(GearControlModel& sim, std::vector<gReal>& desiredacceleration, std::vector<gReal>& controlforce)
{
	sim.setJointData(GearModel::JOINT_ACC, desiredacceleration);
	int ichara=0;

	//sim._cinfo[ichara]->_system->initExternalForce();
	sim._pWorld->_system.update_joint_local_info();
	sim._pWorld->_system.fsFwdRecursion_a();
	sim._pWorld->_system.fsBwdRecursion_b();
	sim._pWorld->_system.fsFwdRecursion_c();

	//sim._system->updateGlobalLocationsOfBodiesAndJoints();
	sim.getJointData(GearModel::JOINT_TAU, controlforce);

	//vectorn theta,dtheta;
	//sim.getLinkData(0, DynamicsSimulator::JOINT_VALUE, theta);
	//sim.getLinkData(0, DynamicsSimulator::JOINT_VELOCITY, dtheta);
	//printf("%s\n%s\n:thetadtheta\n", theta.output().ptr(), dtheta.output().ptr());
	//printf("cf: %s\n", controlforce.output().ptr());
}

inline void packTau(int McolIdx, object &M, std::vector<gReal>& in)
{
	//out.setSize(in.size()-1);
	//out.range(0,3).assign(in.range(4,7));
	//out.range(3,6).assign(in.range(0,3));
	//out.range(6,out.size()).assign(in.range(7, in.size()));
	for(size_t i=0; i<in.size(); i++)
		M[i][McolIdx] = in[i];
}

void GearControlModel::calcMassMatrix3(object& M, object& b)
{
	//differs from calcMassMatrix !!! all dw, dv, tauext, fext are w.r.t body local coordinate!
		  //|       |   | dw   |   |    |   | tauext    |
		  //| out_M | * | dv   | + | b1 | = | fext      |
		  //|       |   |ddq   |   |    |   | u         |

	_pWorld->_system.initBodyForces();

	int numActualDOF=getTotalDOF();

	std::vector<gReal> desiredacceleration;
	std::vector<gReal> controlforce;
	//M.setSize(numActualDOF, numActualDOF);

	for( int i=0; i<numActualDOF; i++)
		desiredacceleration.push_back(0.0);
	for( int i=0; i<numActualDOF; i++)
		controlforce.push_back(0.0);

	std::vector<gReal> controlforce_backup;
	getJointData(JOINT_TAU, controlforce_backup);

	//for (int j=sj;j<ej;j++)
		//cinfo[j]->joint->setPrescribed(true);
	modelJointFree.setPrescribed(true);
	for(size_t i=1; i<_nodes.size(); i++)
		for(size_t j=0; j<_nodes[i]->joint.size(); j++)
			_nodes[i]->joint[j]->setPrescribed(true);

	for(int i=0; i<3; i++)
	{
		desiredacceleration[i+3]=1;
		ID(*this, desiredacceleration, controlforce);
		desiredacceleration[i+3]=0;
		packTau(i, M, controlforce);
	}
	for(int i=3; i<6; i++)
	{
		desiredacceleration[i-3]=1;
		ID(*this, desiredacceleration, controlforce);
		desiredacceleration[i-3]=0;
		packTau(i, M, controlforce);
	}
	for(int i=6; i<numActualDOF; i++)
	{
		desiredacceleration[i]=1;
		ID(*this, desiredacceleration, controlforce);
		desiredacceleration[i]=0;
		packTau(i, M, controlforce);
	}
	// printf("%s\n", desiredacceleration.output().ptr());
	ID(*this, desiredacceleration, controlforce);
	for(size_t i=0; i<controlforce.size(); i++)b[i] = controlforce[i];

	for(int c=0; c<numActualDOF; c++)
		for(int r=0;r<numActualDOF; r++)
			M[r][c] = b[r];

	//for (int j=sj;j<ej;j++)
		//cinfo[j]->joint->setPrescribed(false);
	modelJointFree.setPrescribed(false);
	for(size_t i=1; i<_nodes.size(); i++)
		for( size_t j=0; j<_nodes[i]->joint.size(); j++)
			_nodes[i]->joint[j]->setPrescribed(false);
	setJointData(JOINT_TAU, controlforce_backup);
}

void GearControlModel::stepKinematics(double dt, const bp::list& accs)
{
	std::vector <gReal> _values;
	Vec3 hipacc(pyVec3_2_Vec3(accs[0].slice(0,3)));
	Vec3 hipangacc(pyVec3_2_Vec3(accs[0].slice(3,6)));
	//Vec3 hipacc(0.0);
	//Vec3 hipangacc(0.0);
	_values.push_back(hipacc[0]);
	_values.push_back(hipacc[1]);
	_values.push_back(hipacc[2]);
	_values.push_back(hipangacc[0]);
	_values.push_back(hipangacc[1]);
	_values.push_back(hipangacc[2]);
	Vec3 ddq(0.0);
	for(size_t i=1; i<_nodes.size(); i++)
	{	
		ddq = pyVec3_2_Vec3(accs[i]);
		_values.push_back(ddq[0]);
		_values.push_back(ddq[1]);
		_values.push_back(ddq[2]);
	}
	setJointData(JOINT_ACC, _values);

	std::list<GCoordinate *>::iterator iter_pcoord;
	for (iter_pcoord = _pWorld->_system.pCoordinates.begin(); iter_pcoord != _pWorld->_system.pCoordinates.end(); iter_pcoord++) 
	{
		(*iter_pcoord)->dq += (*iter_pcoord)->ddq * dt;
		(*iter_pcoord)->q += (*iter_pcoord)->dq * dt;
	}

	_pWorld->_system.updateKinematics();
}
