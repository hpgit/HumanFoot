#include "stdafx.h"

#include "../../externalLibs/common/boostPythonUtil.h"

#include "VPUtil.h"
#include "csVpModel.h"
#include "csVpWorld.h"

#define MAX_X 1	// 0001
#define MAX_Y 2	// 0010
#define MAX_Z 4	// 0100

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyPositionGlobal_py_overloads, getBodyPositionGlobal_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyVelocityGlobal_py_overloads, getBodyVelocityGlobal_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpMotionModel_recordVelByFiniteDiff_overloads, recordVelByFiniteDiff, 0, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_applyBodyGenForceGlobal_overloads, applyBodyGenForceGlobal, 3, 4);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(VpControlModel_applyBodyForceGlobal_overloads, applyBodyForceGlobal, 2, 3);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(initializeHybridDynamics_overloads, initializeHybridDynamics, 0, 1);

BOOST_PYTHON_MODULE(csVpModel)
{
	numeric::array::set_module_and_type("numpy", "ndarray");

	class_<VpModel>("VpModel", init<VpWorld*, object, object>())
		.def("__str__", &VpModel::__str__)
		.def("getBodyNum", &VpModel::getBodyNum)
		.def("getBodyMasses", &VpModel::getBodyMasses)
		.def("getTotalMass", &VpModel::getTotalMass)
		.def("getBodyShape", &VpModel::getBodyShape)
		.def("getBodyVerticesPositionGlobal", &VpModel::getBodyVerticesPositionGlobal)

		.def("index2name", &VpModel::index2name)
		.def("index2id", &VpModel::index2id)
		.def("name2index", &VpModel::name2index)
		.def("name2id", &VpModel::name2id)

		.def("getBodyInertiaLocal", &VpModel::getBodyInertiaLocal_py)
		.def("getBodyInertiaGlobal", &VpModel::getBodyInertiaGlobal_py)

		.def("getBodyInertiasLocal", &VpModel::getBodyInertiasLocal)
		.def("getBodyInertiasGlobal", &VpModel::getBodyInertiasGlobal)
		
		.def("getBodyPositionGlobal", &VpModel::getBodyPositionGlobal_py, getBodyPositionGlobal_py_overloads())
		.def("getBodyVelocityGlobal", &VpModel::getBodyVelocityGlobal_py, getBodyVelocityGlobal_py_overloads())
		.def("getBodyAccelerationGlobal", &VpModel::getBodyAccelerationGlobal_py)
		.def("getBodyOrientationGlobal", &VpModel::getBodyOrientationGlobal)
		.def("getBodyAngVelocityGlobal", &VpModel::getBodyAngVelocityGlobal)
		.def("getBodyAngAccelerationGlobal", &VpModel::getBodyAngAccelerationGlobal)

		.def("getBodyPositionsGlobal", &VpModel::getBodyPositionsGlobal)
		.def("getBodyVelocitiesGlobal", &VpModel::getBodyVelocitiesGlobal)
		.def("getBodyAccelerationsGlobal", &VpModel::getBodyAccelerationsGlobal)
		.def("getBodyAngVelocitiesGlobal", &VpModel::getBodyAngVelocitiesGlobal)
		.def("getBodyAngAccelerationsGlobal", &VpModel::getBodyAngAccelerationsGlobal)

		.def("setBodyPositionGlobal", &VpModel::setBodyPositionGlobal_py)
		.def("setBodyVelocityGlobal", &VpModel::setBodyVelocityGlobal_py)
		.def("setBodyAccelerationGlobal", &VpModel::setBodyAccelerationGlobal_py)
		.def("setBodyAngVelocityGlobal", &VpModel::setBodyAngVelocityGlobal)
		.def("setBodyAngAccelerationGlobal", &VpModel::setBodyAngAccelerationGlobal)

		.def("translateByOffset", &VpModel::translateByOffset)
		.def("rotate", &VpModel::rotate)
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

		.def("update", &VpControlModel::update)
		.def("fixBody", &VpControlModel::fixBody)

		.def("initializeHybridDynamics", &VpControlModel::initializeHybridDynamics, initializeHybridDynamics_overloads())
		.def("solveHybridDynamics", &VpControlModel::solveHybridDynamics)

		.def("getDOFPositions", &VpControlModel::getDOFPositions)
		.def("getDOFVelocities", &VpControlModel::getDOFVelocities)
		.def("getDOFAccelerations", &VpControlModel::getDOFAccelerations)
		.def("getDOFAxeses", &VpControlModel::getDOFAxeses)

		.def("getDOFPositionsLocal", &VpControlModel::getDOFPositionsLocal)
		.def("getDOFVelocitiesLocal", &VpControlModel::getDOFVelocitiesLocal)
		.def("getDOFAccelerationsLocal", &VpControlModel::getDOFAccelerationsLocal)
		.def("getDOFAxesesLocal", &VpControlModel::getDOFAxesesLocal)

		.def("setDOFAccelerations", &VpControlModel::setDOFAccelerations)

		.def("getJointOrientationLocal", &VpControlModel::getJointOrientationLocal)
		.def("getJointAngVelocityLocal", &VpControlModel::getJointAngVelocityLocal)
		.def("getJointAngAccelerationLocal", &VpControlModel::getJointAngAccelerationLocal)

		.def("getJointPositionGlobal", &VpControlModel::getJointPositionGlobal)
		.def("getJointVelocityGlobal", &VpControlModel::getJointVelocityGlobal)
		.def("getJointVelocityLocal", &VpControlModel::getJointVelocityLocal)
		.def("getJointAccelerationGlobal", &VpControlModel::getJointAccelerationGlobal)
		.def("getJointAccelerationLocal", &VpControlModel::getJointAccelerationLocal)
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

		.def("getJointTorqueLocal", &VpControlModel::getJointTorqueLocal)
		.def("getInternalJointTorquesLocal", &VpControlModel::getInternalJointTorquesLocal)

		.def("setJointTorqueLocal", &VpControlModel::setJointTorqueLocal)
		.def("setInternalJointTorquesLocal", &VpControlModel::setInternalJointTorquesLocal)

		.def("getEquationOfMotion", &VpControlModel::getEquationOfMotion)
		.def("stepKinematics", &VpControlModel::stepKinematics)
		;
}

 
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
		offset *= (1./len_joint_children);

		SE3 boneT(offset*.5);

		Vec3 defaultBoneV(0,0,1);
		
		SE3 boneR = getSE3FromVectors(defaultBoneV, offset);
		if(joint_name.compare(string("Hips")))
			boneT = boneT * boneR;

		Node* pNode = new Node(joint_name);
		_nodes[joint_index] = pNode;

		object cfgNode = _config.attr("getNode")(joint_name);
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

		pNode->body.AddGeometry(new vpBox(Vec3(width, height, length)));
//		pNode->body.SetInertia(BoxInertia(density, Vec3(width,height,length)));
		pNode->body.SetInertia(BoxInertia(density, Vec3(width/2.,height/2.,length/2.)));

		boneT = boneT * SE3(pyVec3_2_Vec3(cfgNode.attr("offset")));
		_boneTs[joint_index] = boneT;
		SE3 newT = T * boneT;

		pNode->body.SetFrame(newT);
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
	for(int i=0; i<_nodes.size(); ++i)
		ss << "[" << i << "]:" << _nodes[i]->name << ", ";
	ss << endl;

	ss << "<BODY MASSES>" << endl;
	for(int i=0; i<_nodes.size(); ++i)
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
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(_nodes[i]->body.GetInertia().GetMass());
	return ls;
}

scalar VpModel::getTotalMass()
{
	scalar mass = 0.;
	for(int i=0; i<_nodes.size(); ++i)
		mass += _nodes[i]->body.GetInertia().GetMass();
	return mass;
}

object VpModel::getBodyShape(int index)
{
	static numeric::array O(make_tuple(0.,0.,0.));
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
	static numeric::array O(make_tuple(0.,0.,0.));

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
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static SE3 Tin;
	object pyIn = I.copy();

	getBodyInertiaLocal(index, Tin);
	SE3_2_pySO3(Tin, pyIn);
	return pyIn;
}

boost::python::object VpModel::getBodyInertiaGlobal_py( int index )
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static SE3 Tin_local, bodyFrame;
	object pyIn = I.copy();

	getBodyInertiaLocal(index, Tin_local);
	bodyFrame = _nodes[index]->body.GetFrame();
	SE3_2_pySO3(bodyFrame * Tin_local * Inv(bodyFrame), pyIn);
	return pyIn;
	
}

bp::list VpModel::getBodyInertiasLocal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getBodyInertiaLocal_py(i));
	return ls;
}

bp::list VpModel::getBodyInertiasGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getBodyInertiaGlobal_py(i));
	return ls;
}

object VpModel::getBodyPositionGlobal_py( int index, const object& positionLocal/*=object() */ )
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
object VpModel::getBodyVelocityGlobal_py( int index, const object& positionLocal/*=object() */ )
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

bp::list VpModel::getBodyVelocitiesGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getBodyVelocityGlobal_py(i));
	return ls;
}

object VpModel::getBodyAngVelocityGlobal( int index )
{
	static se3 genVel;
	static numeric::array O(make_tuple(0.,0.,0.));
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
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getBodyAngVelocityGlobal(i));
	return ls;
}

object VpModel::getBodyAccelerationGlobal_py(int index, const object& positionLocal )
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

bp::list VpModel::getBodyAccelerationsGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getBodyAccelerationGlobal_py(i));
	return ls;
}

void VpModel::setBodyPositionGlobal_py( int index, const object& pos )
{
	static Vec3 position;

	pyVec3_2_Vec3(pos, position);
	setBodyPositionGlobal(index, position); 
}

void VpModel::setBodyVelocityGlobal_py( int index, const object& vel )
{
	static se3 genVel;
	genVel = _nodes[index]->body.GetGenVelocity();
	genVel[3] = XD(vel[0]);
	genVel[4] = XD(vel[1]);
	genVel[5] = XD(vel[2]);
	_nodes[index]->body.SetGenVelocity(genVel);
}

void VpModel::setBodyAccelerationGlobal_py( int index, const object& acc )
{
	static se3 genAcc;
	genAcc = _nodes[index]->body.GetGenAcceleration();
	genAcc[3] = XD(acc[0]);
	genAcc[4] = XD(acc[1]);
	genAcc[5] = XD(acc[2]);
	_nodes[index]->body.SetGenAcceleration(genAcc);
}

void VpModel::setBodyAngVelocityGlobal( int index, const object& angvel )
{
	static se3 genVel;
	genVel = _nodes[index]->body.GetGenVelocity();
	genVel[0] = XD(angvel[0]);
	genVel[1] = XD(angvel[1]);
	genVel[2] = XD(angvel[2]);
	_nodes[index]->body.SetGenVelocity(genVel);
}

void VpModel::setBodyAngAccelerationGlobal( int index, const object& angacc )
{
	static se3 genAcc;
	genAcc = _nodes[index]->body.GetGenAcceleration();
	genAcc[0] = XD(angacc[0]);
	genAcc[1] = XD(angacc[1]);
	genAcc[2] = XD(angacc[2]);
	_nodes[index]->body.SetGenAcceleration(genAcc);
}

bp::list VpModel::getBodyPositionsGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getBodyPositionGlobal_py(i));
	return ls;
}

object VpModel::getBodyAngAccelerationGlobal( int index )
{
	static se3 genAcc;
	static numeric::array O(make_tuple(0.,0.,0.));
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
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getBodyAngAccelerationGlobal(i));
	return ls;
}

void VpModel::translateByOffset( const object& offset )
{
	static Vec3 v;
	pyVec3_2_Vec3(offset, v);

	for(int i=0; i<_nodes.size(); ++i)
		setBodyPositionGlobal(i, getBodyPositionGlobal(i) + v);
}

void VpModel::rotate( const object& rotation )
{
	static SE3 R, bodyFrame;
	pySO3_2_SE3(rotation, R);

	bodyFrame = _nodes[0]->body.GetFrame();
	_nodes[0]->body.SetFrame(bodyFrame * R);

	// ?ٲ? root body frame?? ???? joint?? ?????? ?????? body?? frame ??????Ʈ. ?̰?�� ???? ??��?? root body ?ϳ??? rotation?? ?????ȴ?. 
	_pWorld->UpdateFrame();
}

Vec3 VpModel::getBodyPositionGlobal( int index, const Vec3* pPositionLocal )
{
	static SE3 bodyFrame;
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
	static se3 genAccLocal, genAccGlobal;

	genAccLocal = _nodes[index]->body.GetGenAccelerationLocal();
	if(pPositionLocal)
		genAccLocal = MinusLinearAd(*pPositionLocal, genAccLocal);
 
	genAccGlobal = Rotate(_nodes[index]->body.GetFrame(), genAccLocal);

	return Vec3(genAccGlobal[3], genAccGlobal[4], genAccGlobal[5]);
}

boost::python::object VpModel::getBodyOrientationGlobal( int index )
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static SE3 bodyFrame;
	object pyR = I.copy();

	// body frame?? Inv(boneT)?? ???? joint frame ???Ѵ?
	bodyFrame = _nodes[index]->body.GetFrame();
	SE3_2_pySO3(bodyFrame, pyR);
	return pyR;
}


void VpModel::setBodyPositionGlobal( int index, const Vec3& position )
{
	static SE3 bodyFrame;
	bodyFrame = _nodes[index]->body.GetFrame();
	bodyFrame.SetPosition(position);
	_nodes[index]->body.SetFrame(bodyFrame);
}

void VpModel::setBodyAccelerationGlobal( int index, const Vec3& acc, const Vec3* pPositionLocal)
{
//	if(pPositionLocal)
//		cout << "pPositionLocal : not implemented functionality yet" << endl;

	static se3 genAcc;
	genAcc = _nodes[index]->body.GetGenAcceleration();
	genAcc[3] = acc[0];
	genAcc[4] = acc[1];
	genAcc[5] = acc[2];

	_nodes[index]->body.SetGenAcceleration(genAcc);
}

VpMotionModel::VpMotionModel( VpWorld* pWorld, const object& createPosture, const object& config )
	:VpModel(pWorld, createPosture, config), _recordVelByFiniteDiff(false), _inverseMotionTimeStep(30.)
{
	// OdeMotionModel?? node.body.disable()�� VpMotionModel?????? pWorld->AddBody()??
	// ?? ???ִ? ??��?? ?? ????�� ?ϵ??? ?Ѵ?.

	update(createPosture);
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
			static SE3 oldT, diffT;
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
}

std::string VpControlModel::__str__()
{
	string s1 = VpModel::__str__();

	stringstream ss;

	ss << "<INTERNAL JOINTS>" << endl;
	for(int i=1; i<_nodes.size(); ++i)
		ss << "[" << i-1 << "]:" << _nodes[i]->name << ", ";
	ss << endl;

	return s1 + ss.str();
}

bp::list VpControlModel::getInternalJointDOFs()
{
	bp::list ls;
	for(int i=1; i<_nodes.size(); ++i)
		ls.append(3);
	return ls;
}

int VpControlModel::getTotalInternalJointDOF()
{
	int dof = 0;
	for(int i=1; i<_nodes.size(); ++i)
		dof += 3;
	return dof;
}

bp::list VpControlModel::getDOFs()
{
	bp::list ls;
	ls.append(6);
	for(int i=1; i<_nodes.size(); ++i)
		ls.append(3);
	return ls;
}
int VpControlModel::getTotalDOF()
{
	int dof = 0;
	dof += 6;
	for(int i=1; i<_nodes.size(); ++i)
		dof += 3;
	return dof;
}

void VpControlModel::createJoints( const object& posture )
{
	object joint = posture.attr("skeleton").attr("root");
	_createJoint(joint, posture);
}

void VpControlModel::_createJoint( const object& joint, const object& posture )
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
	// ��?? ??Ÿ???????? ???⿡?? while loop?? ???? ?????? back tracking?? ???��?
	// L4_M = Inv( Inv(R4)*Inv(P4) * Inv(R3)*Inv(P3) * ...)
	// ��?? ?ڵ???��.

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

		object offset = cfgNode.attr("offset");
		SE3 offsetT = SE3(pyVec3_2_Vec3(offset));

		object parentOffset = parentCfgNode.attr("offset");
		SE3 parentOffsetT = SE3(pyVec3_2_Vec3(parentOffset));

		pParentNode->body.SetJoint(&pNode->joint, Inv(_boneTs[parent_index])*Inv(invLocalT));
		pNode->body.SetJoint(&pNode->joint, Inv(_boneTs[joint_index]));
		pNode->use_joint = true;
	}

	for( int i=0 ; i<len_joint_children; ++i)
		_createJoint(joint.attr("children")[i], posture);
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
	// ��?? ??Ÿ???????? ???⿡?? while loop?? ???? ?????? back tracking?? ???��?
	// L4_M = Inv( Inv(R4)*Inv(P4) * Inv(R3)*Inv(P3) * ...)
	// ��?? ?ڵ???��.

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
			// root?? ?????? body?? ??�� SetFrame() ???ش?.
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
	int rootIndex = 0;
	
	for(int i=0; i<_nodes.size(); ++i)
	{
		if(i == 0)
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

void VpControlModel::solveHybridDynamics()
{
	_nodes[0]->body.GetSystem()->HybridDynamics();	
}

bp::list VpControlModel::getDOFPositions()
{
//	static numeric::array rootFrame( make_tuple(make_tuple(1.,0.,0.,0.), make_tuple(0.,1.,0.,0.), make_tuple(0.,0.,1.,0.), make_tuple(0.,0.,0.,1.)) );
//
//	bp::list ls = getInternalJointOrientationsLocal();
//	SE3_2_pySE3(_nodes[0]->body.GetFrame() * Inv(_boneTs[0]), rootFrame);
//	ls.insert(0, rootFrame );
//	return ls;

	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static numeric::array O(make_tuple(0.,0.,0.));
	static SE3 rootFrame;

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
	static numeric::array rootGenVel(make_tuple(0.,0.,0.,0.,0.,0.));
	
	rootGenVel.slice(0,3) = getJointVelocityGlobal(0);
//	rootGenVel.slice(3,6) = getJointAngVelocityGlobal(0);
	rootGenVel.slice(3,6) = getJointAngVelocityLocal(0);

	bp::list ls = getInternalJointAngVelocitiesLocal();
	ls.insert(0, rootGenVel);
	return ls;
}

bp::list VpControlModel::getDOFAccelerations()
{
	static numeric::array rootGenAcc(make_tuple(0.,0.,0.,0.,0.,0.));
	
	rootGenAcc.slice(0,3) = getJointAccelerationGlobal(0);
//	rootGenAcc.slice(3,6) = getJointAngAccelerationGlobal(0);
	rootGenAcc.slice(3,6) = getJointAngAccelerationLocal(0);

	bp::list ls = getInternalJointAngAccelerationsLocal();

	ls.insert(0, rootGenAcc);
	return ls;
}

bp::list VpControlModel::getDOFAxeses()
{
	static numeric::array rootAxeses( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.),
										make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	numeric::array rootAxesTmp = (numeric::array)getJointOrientationGlobal(0);
	numeric::array rootAxes = transpose_pySO3(rootAxesTmp);
	rootAxeses[3] = rootAxes[0];
	rootAxeses[4] = rootAxes[1];
	rootAxeses[5] = rootAxes[2];

	bp::list ls = getInternalJointOrientationsGlobal();
	for(int i=0; i<len(ls); ++i)
	{
		numeric::array lsTmp = (numeric::array)ls[i];
		ls[i] = transpose_pySO3(lsTmp);
	}

	ls.insert(0, rootAxeses);
	return ls;
}

bp::list VpControlModel::getDOFPositionsLocal()
{
//	static numeric::array rootFrame( make_tuple(make_tuple(1.,0.,0.,0.), make_tuple(0.,1.,0.,0.), make_tuple(0.,0.,1.,0.), make_tuple(0.,0.,0.,1.)) );
//
//	bp::list ls = getInternalJointOrientationsLocal();
//	SE3_2_pySE3(_nodes[0]->body.GetFrame() * Inv(_boneTs[0]), rootFrame);
//	ls.insert(0, rootFrame );
//	return ls;

	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static numeric::array O(make_tuple(0.,0.,0.));
	static SE3 rootFrame;

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
	static numeric::array rootGenVel(make_tuple(0.,0.,0.,0.,0.,0.));
	
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
	static numeric::array rootGenAcc(make_tuple(0.,0.,0.,0.,0.,0.));
	
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
	static numeric::array rootAxeses( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.),
										make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

	numeric::array rootAxesTmp = (numeric::array)getJointOrientationGlobal(0);
	numeric::array rootAxes = transpose_pySO3(rootAxesTmp);
	rootAxeses[0] = rootAxes[0];
	rootAxeses[1] = rootAxes[1];
	rootAxeses[2] = rootAxes[2];
	rootAxeses[3] = rootAxes[0];
	rootAxeses[4] = rootAxes[1];
	rootAxeses[5] = rootAxes[2];

	bp::list ls = getInternalJointOrientationsGlobal();
	for(int i=0; i<len(ls); ++i)
	{
		numeric::array lsTmp = (numeric::array)ls[i];
		ls[i] = transpose_pySO3(lsTmp);
	}

	ls.insert(0, rootAxeses);
	return ls;
}

void VpControlModel::setDOFAccelerations( const bp::list& dofaccs)
{
	static numeric::array O(make_tuple(0.,0.,0.));

	setJointAccelerationGlobal(0, dofaccs[0].slice(0,3));

//	setJointAngAccelerationGlobal(0, dofaccs[0].slice(3,6));
	setJointAngAccelerationLocal(0, dofaccs[0].slice(3,6));

	setInternalJointAngAccelerationsLocal( ((bp::list)dofaccs.slice(1,_)) );
}

boost::python::object VpControlModel::getJointOrientationLocal( int index )
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );

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
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	if(index == 0)
	{
		static se3 genVelBodyLocal, genVelJointLocal;

		genVelBodyLocal = _nodes[index]->body.GetGenVelocityLocal();
		genVelJointLocal = InvAd(Inv(_boneTs[index]), genVelBodyLocal);
//		genVelJointLocal = Ad(_boneTs[index], genVelBodyLocal);	// �� ???ΰ? ??�� ????
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
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	if(index == 0)
	{
		static se3 genAccBodyLocal, genAccJointLocal;

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

object VpControlModel::getJointPositionGlobal( int index )
{
	static numeric::array O(make_tuple(0.,0.,0.));
	static SE3 bodyFrame;
	object pyV = O.copy();

	// body frame?? Inv(boneT)?? ???? joint ��ġ ã?´?.
	bodyFrame = _nodes[index]->body.GetFrame();
	Vec3_2_pyVec3((bodyFrame * Inv(_boneTs[index])).GetPosition(), pyV);
	return pyV;

//	if(!_nodes[index])	// ?????? ???? parent joint frame�� ã?? offset??ŭ transformation ??Ų??.
//	{
//		static SE3 parentJointFrame;
//		static Vec3 offset;
////		int parent = XI(_skeleton.attr("getParentIndex")(index));
//		int parent = XI(_skeleton.attr("getParentJointIndex")(index));
//		parentJointFrame = _nodes[parent]->body.GetFrame() * Inv(_boneTs[parent]);
//		offset = pyVec3_2_Vec3(_skeleton.attr("getOffset")(index));
//		Vec3_2_pyVec3(parentJointFrame * offset, pyV);
//	}
//	else	// ?????? ?ƴ? ???? body frame?? Inv(boneT)?? ???? joint ��ġ ã?´?.
//	{
//		static SE3 bodyFrame;
//		bodyFrame = _nodes[index]->body.GetFrame();
//		Vec3_2_pyVec3((bodyFrame * Inv(_boneTs[index])).GetPosition(), pyV);
//	}
//	return pyV;
}
object VpControlModel::getJointVelocityGlobal( int index )
{
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	Vec3_2_pyVec3(getBodyVelocityGlobal(index, Inv(_boneTs[index]).GetPosition()), pyV);
	return pyV;
}
object VpControlModel::getJointVelocityLocal( int index )
{
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	SE3 jointFrame = _nodes[index]->body.GetFrame() * Inv(_boneTs[index]);

	Vec3_2_pyVec3(InvRotate(jointFrame, getBodyVelocityGlobal(index, Inv(_boneTs[index]).GetPosition())), pyV);
	return pyV;
}

object VpControlModel::getJointAccelerationGlobal( int index )
{
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 pospos = Inv(_boneTs[index]).GetPosition();

	Vec3_2_pyVec3(getBodyAccelerationGlobal(index, &(pospos)), pyV);
	return pyV;
}

object VpControlModel::getJointAccelerationLocal( int index )
{
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 pospos = Inv(_boneTs[index]).GetPosition();
	SE3 jointFrame = _nodes[index]->body.GetFrame() * Inv(_boneTs[index]);

	Vec3_2_pyVec3(InvRotate(jointFrame, getBodyAccelerationGlobal(index, &(pospos))), pyV);
	return pyV;
}

boost::python::object VpControlModel::getJointOrientationGlobal( int index )
{
	static numeric::array I( make_tuple(make_tuple(1.,0.,0.), make_tuple(0.,1.,0.), make_tuple(0.,0.,1.)) );
	static SE3 bodyFrame;
	object pyR = I.copy();

	// body frame?? Inv(boneT)?? ???? joint frame ???Ѵ?
	bodyFrame = _nodes[index]->body.GetFrame();
	SE3_2_pySO3(bodyFrame * Inv(_boneTs[index]), pyR);
	return pyR;
}

boost::python::object VpControlModel::getJointAngVelocityGlobal( int index )
{
	return getBodyAngVelocityGlobal(index);

//	static numeric::array O(make_tuple(0.,0.,0.));
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

bp::list VpControlModel::getJointOrientationsLocal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getJointOrientationLocal(i));
	return ls;
}

bp::list VpControlModel::getJointAngVelocitiesLocal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getJointAngVelocityLocal(i));
	return ls;
}

bp::list VpControlModel::getJointAngAccelerationsLocal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getJointAngAccelerationLocal(i));
	return ls;
}

bp::list VpControlModel::getJointPositionsGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getJointPositionGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointVelocitiesGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getJointVelocityGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointAccelerationsGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getJointAccelerationGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointOrientationsGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getJointOrientationGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointAngVelocitiesGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getJointAngVelocityGlobal(i));
	return ls;
}

bp::list VpControlModel::getJointAngAccelerationsGlobal()
{
	bp::list ls;
	for(int i=0; i<_nodes.size(); ++i)
		ls.append(getJointAngAccelerationGlobal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointOrientationsLocal()
{
	bp::list ls;
	for(int i=1; i<_nodes.size(); ++i)
		ls.append(getJointOrientationLocal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointAngVelocitiesLocal()
{
	bp::list ls;
	for(int i=1; i<_nodes.size(); ++i)
		ls.append(getJointAngVelocityLocal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointAngAccelerationsLocal()
{
	bp::list ls;
	for(int i=1; i<_nodes.size(); ++i)
		ls.append(getJointAngAccelerationLocal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointPositionsGlobal()
{
	bp::list ls;
//	for(int i=1; i<_jointElementIndexes.size(); ++i)
	for(int i=1; i<_nodes.size(); ++i)
		ls.append(getJointPositionGlobal(i));
	return ls;
}

bp::list VpControlModel::getInternalJointOrientationsGlobal()
{
	bp::list ls;
	for(int i=1; i<_nodes.size(); ++i)
		ls.append(getJointOrientationGlobal(i));
	return ls;
}
void VpControlModel::setJointAngVelocityLocal( int index, const object& angvel )
{
	if(index == 0)
	{
		static se3 genVelBodyLocal, genVelJointLocal;
		
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
		static se3 genAccBodyLocal, genAccJointLocal;
		
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
	for(int i=0; i<_nodes.size(); ++i)
		setJointAngAccelerationLocal(i, angaccs[i]);
}

void VpControlModel::setInternalJointAngAccelerationsLocal( const bp::list& angaccs )
{
	for(int i=1; i<_nodes.size(); ++i)
		_nodes[i]->joint.SetAcceleration(pyVec3_2_Vec3(angaccs[i-1]));
}

boost::python::object VpControlModel::getJointTorqueLocal( int index )
{
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	if(index==0) return object();

	Vec3_2_pyVec3(_nodes[index]->joint.GetTorque(), pyV);
	return pyV;
}

bp::list VpControlModel::getInternalJointTorquesLocal()
{
	bp::list ls;
//	for(int i=1; i<_jointElementIndexes.size(); ++i)
	for(int i=1; i<_nodes.size(); ++i)
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
	for(int i=1; i<_nodes.size(); ++i)
		_nodes[i]->joint.SetTorque(pyVec3_2_Vec3(torques[i-1]));
}


void VpControlModel::applyBodyGenForceGlobal( int index, const object& torque, const object& force, const object& positionLocal/*=object()*/ )
{
	static Vec3 zero(0,0,0);
	if(positionLocal==object())
		_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), XD(force[0]), XD(force[1]), XD(force[2])), zero);
	else
		_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), XD(force[0]), XD(force[1]), XD(force[2])), pyVec3_2_Vec3(positionLocal));
}

void VpControlModel::applyBodyForceGlobal( int index, const object& force, const object& positionLocal/*=object()*/ )
{
	static Vec3 zero(0,0,0);
	if(positionLocal==object())
		_nodes[index]->body.ApplyGlobalForce(dse3(0.,0.,0., XD(force[0]), XD(force[1]), XD(force[2])), zero);
	else
		_nodes[index]->body.ApplyGlobalForce(dse3(0.,0.,0., XD(force[0]), XD(force[1]), XD(force[2])), pyVec3_2_Vec3(positionLocal));
}

void VpControlModel::applyBodyTorqueGlobal( int index, const object& torque )
{
	static Vec3 zero(0,0,0);
	_nodes[index]->body.ApplyGlobalForce(dse3(XD(torque[0]), XD(torque[1]), XD(torque[2]), 0.,0.,0.), zero);
}

object VpControlModel::getBodyForceLocal( int index )
{
	static dse3 genForce;
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genForce = _nodes[index]->body.GetForce();
	pyV[0] = genForce[3];
	pyV[1] = genForce[4];
	pyV[2] = genForce[5];
	return pyV;

}

object VpControlModel::getBodyNetForceLocal( int index )
{
	static dse3 genForce;
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genForce = _nodes[index]->body.GetNetForce();
	pyV[0] = genForce[3];
	pyV[1] = genForce[4];
	pyV[2] = genForce[5];
	return pyV;
}

object VpControlModel::getBodyGravityForceLocal( int index )
{
	static dse3 genForce;
	static numeric::array O(make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genForce = _nodes[index]->body.GetGravityForce();
	pyV[0] = genForce[3];
	pyV[1] = genForce[4];
	pyV[2] = genForce[5];
	return pyV;
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

	Hip->ResetForce();
	for(int i=0; i<_nodes.size(); i++)
	{
		//_nodes.at(i)->body.ResetForce();
		_nodes.at(i)->joint.SetTorque(Vec3(0,0,0));
	}

	//get b
	se3 zero_se3(0.0);
	Vec3 zero_Vec3(0.0);
	//is it right?
	Hip->SetGenAccelerationLocal(zero_se3);
	SE3 jointToGlobal = Hip->GetFrame() * Inv(_boneTs[0]);
	for(int i=0; i<n; i++)
	{
		Vec3 acc(0,0,0);
		vpBJoint *joint = &(_nodes.at(i+1)->joint);
		joint->SetAcceleration(acc);
	}
	//Hip->GetSystem()->InverseDynamics();
	Hip->GetSystem()->HybridDynamics();
	dse3 hipTorque_tmp = Hip->GetForce(); // represented in body frame
	//std::cout << "hipTorque_tmp:" <<hipTorque_tmp;
	{
		//b[0] = hipTorque_tmp[3];
		//b[1] = hipTorque_tmp[4];
		//b[2] = hipTorque_tmp[5];
		//b[3] = hipTorque_tmp[0];
		//b[4] = hipTorque_tmp[1];
		//b[5] = hipTorque_tmp[2];
		//is it right?
		dse3 hipTorque_global = InvdAd(Hip->GetFrame(), hipTorque_tmp);
		dse3 hipTorque_joint = dAd(Inv(_boneTs[0]), hipTorque_tmp);
		b[0] = hipTorque_joint[3];
		b[1] = hipTorque_joint[4];
		b[2] = hipTorque_joint[5];
		b[3] = hipTorque_joint[0];
		b[4] = hipTorque_joint[1];
		b[5] = hipTorque_joint[2];
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
		Hip->ResetForce();
		for(int i=0; i<_nodes.size(); i++)
		{
			//_nodes.at(i)->body.ResetForce();
			_nodes.at(i)->joint.SetTorque(Vec3(0,0,0));
		}
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
			//Vec3 hipacc_joint = InvRotate(jointToGlobal, acc_hip);
			se3 genAccJointLocal(accang_hip, acc_hip);
			//se3 genAccBodyLocal = Rotate(Inv(_boneTs[0]), genAccJointLocal);
			se3 genAccBodyLocal = InvAd(_boneTs[0], genAccJointLocal);
			Hip->SetGenAccelerationLocal(genAccBodyLocal);
		}
		//Hip->SetGenAcceleration(se3_tmp);

		Hip->GetSystem()->InverseDynamics();
		//Hip->GetSystem()->HybridDynamics();
		hipTorque_tmp = Hip->GetForce();
		//std::cout << "hipTorque_tmp:" <<hipTorque_tmp;
		//is it right?
		dse3 hipTorque_global = InvdAd(Hip->GetFrame(), hipTorque_tmp);
		dse3 hipTorque_joint = dAd(Inv(_boneTs[0]), hipTorque_tmp);
		for (int j = 0; j < 3; j++)
			//M[j][i] = hipTorque_tmp[j+3] - b[j];
			//M[j][i] = hipTorque_global[j+3] - b[j];
			M[j][i] = hipTorque_joint[j+3] - b[j];
		for (int j = 3; j < 6; j++)
			//M[j][i] = hipTorque_tmp[j-3] - b[j];
			//M[j][i] = hipTorque_global[j+3] - b[j];
			M[j][i] = hipTorque_joint[j-3] - b[j];
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
	//SE3 jointToGlobal = Hip->GetFrame() * Inv(_boneTs[0]);

	{
		se3 genAccJointLocal(hipangacc, hipacc);
		se3 genVelBodyLocal = Hip->GetGenVelocityLocal();
		//se3 genVelJointLocal = InvRotate(Inv(_boneTs[0]), genVelBodyLocal) + genAccJointLocal * dt;
		se3 genVelJointLocal = InvAd(Inv(_boneTs[0]), genVelBodyLocal) + genAccJointLocal * dt;
		//genVelBodyLocal = Rotate(Inv(_boneTs[0]), genVelJointLocal);
		genVelBodyLocal = InvAd(_boneTs[0], genVelJointLocal);
		Hip->SetGenVelocityLocal(genVelBodyLocal);
		SE3 rootFrame = Hip->GetFrame() * Exp(dt*genVelBodyLocal);
		Hip->SetFrame(rootFrame);
	}

	Vec3 ddq(0.0), dq(0.0), zero_Vec3(0.0);
	SE3 q;
	for(int i=1; i<_nodes.size(); ++i)
	{
		ddq = pyVec3_2_Vec3(accs[i]);
		vpBJoint *joint = &(_nodes[i]->joint);
		dq = joint->GetVelocity() + ddq * dt;
		//joint->SetVelocity(zero_Vec3);
		joint->SetVelocity(dq);
		q = joint->GetOrientation() * Exp(Axis(dq*dt));
		//q = Exp(Axis(dq*dt))*joint->GetOrientation(); 
		//std::cout << q << std::endl;
		joint->SetOrientation(q);
	}
	se3 zero_se3(0.0);
	Hip->ResetForce();
	for(int i=0; i<_nodes.size(); i++)
	{
		//_nodes.at(i)->body.ResetForce();
		_nodes.at(i)->body.SetGenAcceleration(zero_se3);
	}
}

