#include "stdafx.h"
#include <algorithm>

#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

//#define make_tuple boost::python::make_tuple

//using boost::python::make_tuple;
namespace bp = boost::python;
namespace np = boost::python::numpy;
using boost::python::numpy::ndarray;

namespace ublas = boost::numeric::ublas;

#define MAX_X 1	// 0001
#define MAX_Y 2	// 0010
#define MAX_Z 4	// 0100

#include <tinyxml2.h>
#include <sstream>
#include <string>
#include "csVpDartModel.h"

using namespace tinyxml2;

#ifndef XMLCheckResult
	#define XMLCheckResult(a_eResult) if (a_eResult != XML_SUCCESS) { printf("Error: %i\n", a_eResult); return;}
#endif

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyPositionGlobal_py_overloads, getBodyPositionGlobal_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyVelocityGlobal_py_overloads, getBodyVelocityGlobal_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getBodyAccelerationGlobal_py_overloads, getBodyAccelerationGlobal_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getJointPositionGlobal_py_overloads, getJointPositionGlobal, 1, 2);

BOOST_PYTHON_MODULE(csVpDartModel)
{
	class_<VpDartModel>("VpDartModel", init<const char *>())
		.def("step", &VpDartModel::step)

		.def("getTimeStep", &VpDartModel::getTimeStep)
		.def("getGravity", &VpDartModel::getGravity)

		.def("calcPenaltyForce", &VpDartModel::calcPenaltyForce)
		.def("applyPenaltyForce", &VpDartModel::applyPenaltyForce)

		.def("initializeHybridDynamics", &VpDartModel::initializeHybridDynamics)
		.def("initializeForwardDynamics", &VpDartModel::initializeForwardDynamics)
		.def("setHybridDynamics", &VpDartModel::setHybridDynamics)
		.def("solveHybridDynamics", &VpDartModel::solveHybridDynamics)
		.def("solveForwardDynamics", &VpDartModel::solveForwardDynamics)
		.def("solveInverseDynamics", &VpDartModel::solveInverseDynamics)

		.def("translateByOffset", &VpDartModel::translateByOffset)

		.def("index2name", &VpDartModel::index2name)
		.def("index2vpid", &VpDartModel::index2vpid)
		.def("name2index", &VpDartModel::name2index)

		.def("getTotalDOF", &VpDartModel::getTotalDOF)
		.def("getDOFs", &VpDartModel::getDOFs)
		.def("getJointDOFIndexes", &VpDartModel::getJointDOFIndexes)
		.def("getJointNum", &VpDartModel::getJointNum)
		.def("getBodyNum", &VpDartModel::getBodyNum)
		.def("getBodyMasses", &VpDartModel::getBodyMasses)
		.def("getTotalMass", &VpDartModel::getTotalMass)

		.def("getBodyInertiaLocal", &VpDartModel::getBodyInertiaLocal_py)
		.def("getBodyInertiaGlobal", &VpDartModel::getBodyInertiaGlobal_py)

		.def("getCOM", &VpDartModel::getCOM)
		.def("getBodyComPositionGlobal", &VpDartModel::getBodyComPositionGlobal)
		.def("getBodyComVelocityGlobal", &VpDartModel::getBodyComVelocityGlobal)

		.def("getBodyFrame", &VpDartModel::getBodyFrame)
		.def("getBodyPositionGlobal", &VpDartModel::getBodyPositionGlobal_py, getBodyPositionGlobal_py_overloads())
		.def("getBodyOrientationGlobal", &VpDartModel::getBodyOrientationGlobal)
		.def("getBodyVelocityGlobal", &VpDartModel::getBodyVelocityGlobal_py, getBodyVelocityGlobal_py_overloads())
		.def("getBodyAccelerationGlobal", &VpDartModel::getBodyAccelerationGlobal_py, getBodyAccelerationGlobal_py_overloads())
		.def("getBodyAngVelocityGlobal", &VpDartModel::getBodyAngVelocityGlobal)
		.def("getBodyAngAccelerationGlobal", &VpDartModel::getBodyAngAccelerationGlobal)

		.def("getBodyGeomNum", &VpDartModel::getBodyGeomNum)
		.def("getBodyGeomsType", &VpDartModel::getBodyGeomsType)
		.def("getBodyGeomsSize", &VpDartModel::getBodyGeomsSize)
		.def("getBodyGeomsLocalFrame", &VpDartModel::getBodyGeomsLocalFrame)
		.def("getBodyGeomsGlobalFrame", &VpDartModel::getBodyGeomsGlobalFrame)

		.def("getJointOrientationGlobal", &VpDartModel::getJointOrientationGlobal)
		.def("getJointPositionGlobal", &VpDartModel::getJointPositionGlobal, getJointPositionGlobal_py_overloads())
		.def("getJointVelocityGlobal", &VpDartModel::getJointVelocityGlobal)
		.def("getJointAngVelocityGlobal", &VpDartModel::getJointAngVelocityGlobal)
		.def("getJointAccelerationGlobal", &VpDartModel::getJointAccelerationGlobal)
		.def("getJointAngAccelerationGlobal", &VpDartModel::getJointAngAccelerationGlobal)

		.def("getJointOrientationLocal", &VpDartModel::getJointOrientationLocal)
		.def("getJointAngVelocityLocal", &VpDartModel::getJointAngVelocityLocal)
		.def("getJointAngAccelerationLocal", &VpDartModel::getJointAngAccelerationLocal)

		.def("setJointAngAccelerationLocal", &VpDartModel::setJointAngAccelerationLocal)
		.def("setJointAccelerationGlobal", &VpDartModel::setJointAccelerationGlobal)

		.def("getDOFPositions", &VpDartModel::getDOFPositions)
		.def("getDOFVelocities", &VpDartModel::getDOFVelocities)
		.def("getDOFAccelerations", &VpDartModel::getDOFAccelerations)

		.def("setDOFAccelerations", &VpDartModel::setDOFAccelerations)
		.def("setDOFTorques", &VpDartModel::setDOFTorques)

        .def("computeCom_J_dJdq", &VpDartModel::computeCom_J_dJdq)
	    ;
}


VpDartModel::~VpDartModel()
{
//    delete _pWorld;
//	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
//	    delete _nodes[i];
}

static void getFloats(const char* text, int num, std::vector<float> &floats)
{
    string text_str(text);
    std::istringstream in(text_str);      //make a stream for the line itself
    float x, y, z, u, v, w;
    floats.clear();
    if (num == 2)
    {
        in >> x >> y;
        floats.push_back(x);
        floats.push_back(y);
    }
    if (num == 3)
    {
        in >> x >> y >> z;
        floats.push_back(x);
        floats.push_back(y);
        floats.push_back(z);
    }
    else if (num == 6)
    {
        in >> x >> y >> z >> u >> v >> w;
        floats.push_back(x);
        floats.push_back(y);
        floats.push_back(z);
        floats.push_back(u);
        floats.push_back(v);
        floats.push_back(w);
    }
}

static SE3 dof6_to_SE3(std::vector<float> &floats)
{
    return EulerXYZ(Vec3(floats[3], floats[4], floats[5]), Vec3(floats[0], floats[1], floats[2]));
}


void VpDartModel::skel_init(const char *skel_path)
{
    boost::python::numpy::initialize();
    XMLDocument xmlDoc;
    XMLError eResult = xmlDoc.LoadFile(skel_path);
    XMLCheckResult(eResult);

    XMLElement *pSkel = xmlDoc.FirstChildElement();

    XMLElement *pWorld = pSkel->FirstChildElement();
    this->_pWorld = new vpWorld;

    // physics
    XMLElement *pPhysics = pWorld->FirstChildElement("physics");

    XMLElement *pTime_step = pPhysics->FirstChildElement("time_step");
    float time_step;
    pTime_step->QueryFloatText(&time_step);
    XMLElement *pGravity = pPhysics->FirstChildElement("gravity");
    this->_pWorld->SetTimeStep(time_step);

    std::vector<float> gravity;
    getFloats(pGravity->GetText(), 3, gravity);
    this->_pWorld->SetGravity(Vec3(gravity[0], gravity[1], gravity[2]));

    // skeleton
    XMLElement *pSkeleton = pWorld->FirstChildElement("skeleton");
    this->name = pSkeleton->Attribute("name");
    if(this->name.find(string("ground")) != std::string::npos)
    {
        pSkeleton = pSkeleton->NextSiblingElement("skeleton");
        this->name = pSkeleton->Attribute("name");
    }


//    Node* pNode = new Node(joint_name);
//    _nodes[joint_index] = pNode;

    // skeleton - body
    XMLElement *pBody = pSkeleton->FirstChildElement("body");
    int body_idx = 0;
    std::vector<Node *> nodes;
    std::vector<string> body_name;
    std::vector<SE3> body_frame;
    std::vector<float> body_mass;
    std::vector<Vec3> body_com_offset;
    std::vector< std::vector<vpGeom*> > body_geom;
    std::vector< std::vector<SE3> > body_geom_transform;

    while(pBody != nullptr)
    {
//        std::cout << "TinyXml Debug: body "<< body_idx << std::endl;
        string name = pBody->Attribute("name");
        body_name.push_back(name);
//        std::cout << "TinyXml Debug: "<< name << std::endl;
        Node* pNode = new Node(name);
        nodes.push_back(pNode);

        // skeleton - body - transform
        std::vector<float> body_transform;
        getFloats(pBody->FirstChildElement("transformation")->GetText(), 6, body_transform);
        body_frame.push_back(dof6_to_SE3(body_transform));

        // skeleton - body - inertia
        XMLElement *pInertia = pBody->FirstChildElement("inertia");
        float mass;
        std::vector<float> offset;
        pInertia->FirstChildElement("mass")->QueryFloatText(&mass);
        getFloats(pInertia->FirstChildElement("offset")->GetText(), 3, offset);
        body_mass.push_back(mass);
        body_com_offset.push_back(Vec3(offset[0], offset[1], offset[2]));

        // skeleton - body - collision_shape
        // ignoring visual shape because it does not affect simulation
        std::vector<vpGeom* > _body_geom;
        std::vector<SE3> _body_geom_transform;
        XMLElement *pShape = pBody->FirstChildElement("collision_shape");
        while(pShape != nullptr)
        {
            string geom_type = pShape->FirstChildElement("geometry")->FirstChildElement()->Value();
            vpGeom *pGeom;
            if(geom_type == "box")
            {
                std::vector<float> geom_size;
                getFloats(pShape->FirstChildElement("geometry")->FirstChildElement("box")->FirstChildElement("size")->GetText(), 3, geom_size);
                pGeom = new vpBox(Vec3(geom_size[0], geom_size[1], geom_size[2]));
            }
            else if(geom_type == "sphere")
            {
                float geom_size;
                pShape->FirstChildElement("geometry")->FirstChildElement("sphere")->FirstChildElement("size")->QueryFloatText(&geom_size);
                pGeom = new vpSphere();
            }
            else if(geom_type == "capsule")
            {
                float radius, height;
                pShape->FirstChildElement("geometry")->FirstChildElement("capsule")->FirstChildElement("radius")->QueryFloatText(&radius);
                pShape->FirstChildElement("geometry")->FirstChildElement("capsule")->FirstChildElement("height")->QueryFloatText(&height);
                pGeom = new vpCapsule(radius, height);
            }
            else if(geom_type == "cylinder")
            {
                float radius, height;
                pShape->FirstChildElement("geometry")->FirstChildElement("cylinder")->FirstChildElement("radius")->QueryFloatText(&radius);
                pShape->FirstChildElement("geometry")->FirstChildElement("cylinder")->FirstChildElement("height")->QueryFloatText(&height);
                pGeom = new vpCylinder(radius, height);
            }
            else
            {
                std::cout << "WARNING!! : " << geom_type << " is not implemented or not supported!" << std::endl;
                continue;
            }

            std::vector<float> geom_transform;
            getFloats(pShape->FirstChildElement("transformation")->GetText(), 6, geom_transform);
            _body_geom_transform.push_back(dof6_to_SE3(geom_transform));
            pNode->body.AddGeometry(pGeom, dof6_to_SE3(geom_transform));

            pShape = pShape->NextSiblingElement("collision_shape");
        }

        pBody = pBody->NextSiblingElement("body");
        body_idx++;
    }

    _nodes.resize(body_idx, NULL);
    _boneTs.resize(body_idx, SE3());
    for(int i=0; i<body_idx; i++)
    {
        _nodes[i] = nodes[i];
        _nodes[i]->body.SetFrame(body_frame[i]);
        this->_name2index[body_name[i]] = i;
        this->_id2index[_nodes[i]->body.GetID()] = i;
//        _pWorld->AddBody(&(_nodes[i]->body));
        _nodes[i]->body.SetInertia(Inertia(body_mass[i]));
//        std::cout << body_name[i] << " " << _nodes[i]->body.GetInertia().GetMass() << " " << body_mass[i] << std::endl;
    }

    // skeleton - joint
    XMLElement *pJoint = pSkeleton->FirstChildElement("joint");
    int joint_idx = 0;
    std::vector<string> joint_name;
    std::vector<string> joint_type;
    std::vector<SE3> joint_frame;
    std::vector<string> joint_parent;
    std::vector<string> joint_child;

    while(pJoint != nullptr)
    {
        string name = pJoint->Attribute("name");
        joint_name.push_back(name);

        string type = pJoint->Attribute("type");
        joint_type.push_back(type);

        string parent = pJoint->FirstChildElement("parent")->GetText();
        joint_parent.push_back(parent);

        string child = pJoint->FirstChildElement("child")->GetText();
        joint_child.push_back(child);

        std::vector<float> joint_transform;
        if(pJoint->FirstChildElement("transformation") != nullptr)
        {
            getFloats(pJoint->FirstChildElement("transformation")->GetText(), 6, joint_transform);
            joint_frame.push_back(dof6_to_SE3(joint_transform));
        }
        else
        {
            joint_frame.push_back(SE3());
        }

//        std::cout << "TinyXml Debug: "<< name << type << parent << child << std::endl;

        pJoint = pJoint->NextSiblingElement("joint");
        joint_idx++;
    }

    int joint_dof_index = 0;

    for(int i=0; i<joint_idx; i++)
    {
        std::vector<Node*>::size_type child_node_idx = 0, parent_node_idx = 0;
        while(child_node_idx < _nodes.size())
        {
            if(_nodes[child_node_idx]->name == joint_child[i])
            {
                break;
            }
            child_node_idx++;
        }
        if(child_node_idx == _nodes.size())
            continue;

        if(joint_type[i] == "free")
        {
            _nodes[child_node_idx]->dof = 6;
            _nodes[child_node_idx]->dof_start_index = 0;
            _nodes[child_node_idx]->parent_index = -1;
            joint_dof_index += 6;
            _boneTs[child_node_idx] = Inv(joint_frame[i]);
            this->_pWorld->AddBody(&(_nodes[child_node_idx]->body));
            this->m_total_dof += 6;
            continue;
        }

        while(parent_node_idx < _nodes.size())
        {
            if(_nodes[parent_node_idx]->name == joint_parent[i])
            {
                break;
            }
            parent_node_idx++;
        }

        if(parent_node_idx == _nodes.size())
            continue;
//        std::cout << "TinyXml Debug: "<< joint_name[i] << joint_type[i] << joint_parent[i] << joint_child[i] << std::endl;
//        std::cout << "TinyXml Debug: "<< parent_node_idx << " " << child_node_idx << std::endl;

        SE3 child_body_to_joint = joint_frame[i];
        SE3 joint_to_child_body = Inv(joint_frame[i]);
        SE3 parent_body_to_joint = Inv(_nodes[parent_node_idx]->body.GetFrame()) * _nodes[child_node_idx]->body.GetFrame() * Inv(child_body_to_joint);
//        std::cout << _nodes[parent_node_idx]->body.GetFrame() << std::endl;
//        std::cout << _nodes[child_node_idx]->body.GetFrame() << std::endl;
//        std::cout << joint_frame[i] << std::endl;
//        std::cout << _nodes[parent_node_idx]->body.GetFrame() * parent_body_to_joint << std::endl;
//        std::cout << _nodes[child_node_idx]->body.GetFrame() * child_body_to_joint << std::endl;

        if(joint_type[i] == "ball")
        {
            _nodes[child_node_idx]->m_pJoint = new vpBJoint();
            _nodes[child_node_idx]->m_pJoint->m_szName = joint_name[i];
            _nodes[child_node_idx]->dof = 3;
            _nodes[child_node_idx]->dof_start_index = joint_dof_index;
            _nodes[child_node_idx]->use_joint = true;
            _nodes[child_node_idx]->parent_index = parent_node_idx;
            joint_dof_index += 3;
            _nodes[parent_node_idx]->body.SetJoint(_nodes[child_node_idx]->m_pJoint, parent_body_to_joint);
            _nodes[child_node_idx]->body.SetJoint(_nodes[child_node_idx]->m_pJoint, child_body_to_joint);
            _boneTs[child_node_idx] = joint_to_child_body;
            this->m_total_dof += 3;
            this->m_joint_num++;

        }
        else if(joint_type[i] == "weld")
        {
            _nodes[child_node_idx]->m_pJoint = new vpWJoint();
            _nodes[child_node_idx]->m_pJoint->m_szName = joint_name[i];
            _nodes[child_node_idx]->dof = 0;
            _nodes[child_node_idx]->dof_start_index = joint_dof_index;
            _nodes[child_node_idx]->use_joint = false;
            _nodes[child_node_idx]->parent_index = parent_node_idx;
            _nodes[parent_node_idx]->body.SetJoint(_nodes[child_node_idx]->m_pJoint, parent_body_to_joint);
            _nodes[child_node_idx]->body.SetJoint(_nodes[child_node_idx]->m_pJoint, child_body_to_joint);
            _boneTs[child_node_idx] = joint_to_child_body;
            this->m_joint_num++;
        }
        else
        {
            _nodes[child_node_idx]->parent_index = -1;
            std::cout << "WARNING!! : " << joint_type[i] << " is not implemented or not supported!" << std::endl;
            continue;
        }
    }

	for( auto pNode0 : _nodes)
	    for (auto pNode1 : _nodes)
            _pWorld->IgnoreCollision(&pNode0->body, &pNode1->body);

    this->_pWorld->SetGlobalDamping(0.999);
    this->_pWorld->Initialize();
	this->_pWorld->SetIntegrator(VP::IMPLICIT_EULER_FAST);
//	this->_pWorld->SetIntegrator(VP::RK4);
//	this->_pWorld->SetIntegrator(VP::EULER);

    for(int i=0; i<body_idx; i++)
    {
        this->_id2index[_nodes[i]->body.GetID()] = i;
    }

    int k = 0;
    for(std::vector<Node*>::size_type i=0; i < _nodes.size(); i++)
    {
        k = (int)i;
        while(k >= 0)
        {
            _nodes[i]->ancestors.push_back(k);
            k = _nodes[k]->parent_index;
        }
    }
    for(std::vector<int>::size_type body_idx=0; body_idx<_nodes.size(); body_idx++)
    {
        std::vector<int> &ancestors = _nodes[body_idx]->ancestors;
        for(std::vector<int>::size_type j=0; j<_nodes.size(); j++)
        {
            if(std::find(ancestors.begin(), ancestors.end(), j) != ancestors.end())
            {
                _nodes[body_idx]->is_ancestor.push_back(true);
            }
            else
            {
                _nodes[body_idx]->is_ancestor.push_back(false);
            }
        }
    }
}


void VpDartModel::step()
{
    this->_pWorld->StepAhead();
}

object VpDartModel::getGravity()
{
	object pyV;
	make_pyVec3(pyV);
	Vec3_2_pyVec3(_pWorld->GetGravity(), pyV);
	return pyV;
}

bool VpDartModel::_calcPenaltyForce( const vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu )
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
		frictionForce = mu * normalForce;

		if(tangentialRelVel < _lockingVel)
			frictionForce *= tangentialRelVel/_lockingVel;

		vFrictionForce = frictionForce * -Normalize(vTangentialRelVel);

		force = vNormalForce + vFrictionForce;
		return true;
	}
}

// @return ( bodyIDs, positions, postionLocals, forces)
boost::python::tuple VpDartModel::calcPenaltyForce( const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds )
{
	bp::list bodyIDs, positions, forces, positionLocals;
	int bodyID;
	static ndarray O_Vec3 = np::array(bp::make_tuple(0.,0.,0.));
	const vpBody* pBody;
	const vpGeom* pGeom;
	char type;
	scalar data[3];
	static Vec3 position, velocity, force, positionLocal, _positionLocal;

	for(int i=0; i<len(bodyIDsToCheck); ++i)
	{
		bodyID = XI(bodyIDsToCheck[i]);
		pBody = _pWorld->GetBody(bodyID);

		for(int j=0; j<pBody->GetNumGeometry(); ++j)
		{
			pGeom = pBody->GetGeometry(j);
			pGeom->GetShape(&type, data);
			const SE3& geomFrame = pGeom->GetGlobalFrame();

			for( int p=0; p<8; ++p)
			{
				_positionLocal[0] = (p & MAX_X) ? data[0]/2. : -data[0]/2.;
				_positionLocal[1] = (p & MAX_Y) ? data[1]/2. : -data[1]/2.;
				_positionLocal[2] = (p & MAX_Z) ? data[2]/2. : -data[2]/2.;

				SE3 body_frame = pBody->GetFrame();

				position = geomFrame * _positionLocal;
				positionLocal = Inv(body_frame) * position;

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
	return bp::make_tuple(bodyIDs, positions, positionLocals, forces);
}


void VpDartModel::applyPenaltyForce( const bp::list& bodyIDs, const bp::list& positionLocals, const bp::list& forces )
{
	int bodyID;
	vpBody* pBody;
	Vec3 position, force;

	for(int i=0; i<len(bodyIDs); ++i)
	{
		bodyID = XI(bodyIDs[i]);
		pyVec3_2_Vec3(positionLocals[i], position);
		pyVec3_2_Vec3(forces[i], force);

		pBody = _pWorld->GetBody(bodyID);
		pBody->ApplyGlobalForce(force, position);
	}
}

void VpDartModel::translateByOffset( const object& offset )
{
	Vec3 v;
	pyVec3_2_Vec3(offset, v);

	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		setBodyPositionGlobal(i, getBodyPositionGlobal(i) + v);
}

void VpDartModel::initializeHybridDynamics()
{
	std::vector<int>::size_type rootIndex = 0;

	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
	{
		if(i == rootIndex)
            _nodes[i]->body.SetHybridDynamicsType(VP::DYNAMIC);
		else
			_nodes[i]->m_pJoint->SetHybridDynamicsType(VP::KINEMATIC);
	}
}

void VpDartModel::initializeForwardDynamics()
{
    std::vector<int>::size_type rootIndex = 0;
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
	{
        _nodes[i]->body.SetHybridDynamicsType(VP::DYNAMIC);
	    if(i != rootIndex)
            _nodes[i]->m_pJoint->SetHybridDynamicsType(VP::DYNAMIC);
    }
}

void VpDartModel::setHybridDynamics(int jointIndex, std::string dynamicsType)
{
    if(dynamicsType == "DYNAMIC")
    {
        _nodes[jointIndex]->body.SetHybridDynamicsType(VP::DYNAMIC);
        _nodes[jointIndex]->m_pJoint->SetHybridDynamicsType(VP::DYNAMIC);
    }
    else if(dynamicsType == "KINEMATIC")
        _nodes[jointIndex]->m_pJoint->SetHybridDynamicsType(VP::KINEMATIC);
}

void VpDartModel::solveHybridDynamics()
{
	_nodes[0]->body.GetSystem()->HybridDynamics();
}

void VpDartModel::solveForwardDynamics()
{
	_nodes[0]->body.GetSystem()->ForwardDynamics();
}

void VpDartModel::solveInverseDynamics()
{
	_nodes[0]->body.GetSystem()->InverseDynamics();
}

bp::list VpDartModel::getDOFs()
{
    bp::list ls;
    for(auto node : _nodes)
        ls.append(node->dof);
    return ls;
}

scalar VpDartModel::getTotalMass()
{
    scalar mass = 0.;
    for (auto node : _nodes)
        mass += node->body.GetInertia().GetMass();
    return mass;
}

bp::list VpDartModel::getJointDOFIndexes(int index)
{
    bp::list ls;
    int dof_index = _nodes[index]->dof_start_index;
    for(int i=0; i<_nodes[index]->dof; i++)
        ls.append(dof_index++);
    return ls;

}

bp::list VpDartModel::getBodyMasses()
{
    bp::list ls;
    for(auto node : _nodes)
        ls.append(node->body.GetInertia().GetMass());
    return ls;
}

void VpDartModel::setBodyPositionGlobal( int index, const Vec3& position )
{
	SE3 bodyFrame = _nodes[index]->body.GetFrame();
	bodyFrame.SetPosition(position);
	_nodes[index]->body.SetFrame(bodyFrame);
}

void VpDartModel::getBodyInertiaLocal(int index, SE3& Tin)
{
//	if(!_nodes[index]) return;

//	ss << "I11 I22 I33 I12 I13 I23 offset.x offset.y offset.z mass" << endl;

//	pyIn[bp::make_tuple(0,0)] = in[0];
//	pyIn[bp::make_tuple(1,1)] = in[1];
//	pyIn[bp::make_tuple(2,2)] = in[2];
//	pyIn[bp::make_tuple(0,1)] = pyIn[bp::make_tuple(1,0)] = in[3];
//	pyIn[bp::make_tuple(0,2)] = pyIn[bp::make_tuple(2,0)] = in[4];
//	pyIn[bp::make_tuple(1,2)] = pyIn[bp::make_tuple(2,1)] = in[5];

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

boost::python::object VpDartModel::getBodyInertiaLocal_py( int index )
{
	ndarray I = np::array( bp::make_tuple(bp::make_tuple(1.,0.,0.), bp::make_tuple(0.,1.,0.), bp::make_tuple(0.,0.,1.)) );
	SE3 Tin;
	object pyIn = I.copy();

	getBodyInertiaLocal(index, Tin);
	SE3_2_pySO3(Tin, pyIn);
	return pyIn;
}

boost::python::object VpDartModel::getBodyInertiaGlobal_py( int index )
{
	ndarray I = np::array( bp::make_tuple(bp::make_tuple(1.,0.,0.), bp::make_tuple(0.,1.,0.), bp::make_tuple(0.,0.,1.)) );
	SE3 Tin_local, bodyFrame;
	object pyIn = I.copy();

	getBodyInertiaLocal(index, Tin_local);
	bodyFrame = _nodes[index]->body.GetFrame() * SE3(_nodes[index]->body.GetCenterOfMass());
	SE3_2_pySO3(bodyFrame * Tin_local * Inv(bodyFrame), pyIn);
	return pyIn;

}

Vec3 VpDartModel::getBodyPositionGlobal( int index, const Vec3* pPositionLocal )
{
	SE3 bodyFrame = _nodes[index]->body.GetFrame();
	if(!pPositionLocal)
		return bodyFrame.GetPosition();
	else
		return bodyFrame * (*pPositionLocal);
}

Vec3 VpDartModel::getBodyVelocityGlobal( int index, const Vec3& positionLocal)
{
	return _nodes[index]->body.GetLinVelocity(positionLocal);
}

Vec3 VpDartModel::getBodyAccelerationGlobal( int index, const Vec3* pPositionLocal)
{
	se3 genAccLocal, genAccGlobal;

	genAccLocal = _nodes[index]->body.GetGenAccelerationLocal();
	if(pPositionLocal)
		genAccLocal = MinusLinearAd(*pPositionLocal, genAccLocal);

	genAccGlobal = Rotate(_nodes[index]->body.GetFrame(), genAccLocal);

	return Vec3(genAccGlobal[3], genAccGlobal[4], genAccGlobal[5]);
}

object VpDartModel::getCOM()
{
	object pyV;
	make_pyVec3(pyV);
	Vec3 com(0., 0., 0.);
	for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i)
		com += _nodes[i]->body.GetInertia().GetMass() * _nodes[i]->get_com_position();
	com *= 1./getTotalMass();

	Vec3_2_pyVec3(com, pyV);
	return pyV;

}

object VpDartModel::getBodyComPositionGlobal( int index )
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 positionLocal_ = _nodes[index]->body.GetCenterOfMass();
    Vec3_2_pyVec3(getBodyPositionGlobal(index, &positionLocal_), pyV);
	return pyV;
}

object VpDartModel::getBodyComVelocityGlobal( int index )
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 positionLocal_ = _nodes[index]->body.GetCenterOfMass();
    Vec3_2_pyVec3(getBodyVelocityGlobal(index, positionLocal_), pyV);
	return pyV;
}

object VpDartModel::getBodyFrame(int index)
{
    object pyT;
    make_pySE3(pyT);
	SE3 bodyFrame = _nodes[index]->body.GetFrame();
    SE3_2_pySE3(bodyFrame, pyT);
	return pyT;
}

object VpDartModel::getBodyPositionGlobal_py( int index, const object& positionLocal/*=object() */ )
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 positionLocal_;

	if(positionLocal.is_none())
		Vec3_2_pyVec3(getBodyPositionGlobal(index), pyV);
	else
	{
		pyVec3_2_Vec3(positionLocal, positionLocal_);
		Vec3_2_pyVec3(getBodyPositionGlobal(index, &positionLocal_), pyV);
	}
	return pyV;
}

object VpDartModel::getBodyOrientationGlobal(int index)
{
	ndarray I = np::array(bp::make_tuple(bp::make_tuple(1., 0., 0.), bp::make_tuple(0., 1., 0.), bp::make_tuple(0., 0., 1.)));
	SE3 bodyFrame;
	object pyR = I.copy();

	bodyFrame = _nodes[index]->body.GetFrame();
	SE3_2_pySO3(bodyFrame, pyR);
	return pyR;
}

object VpDartModel::getBodyVelocityGlobal_py( int index, const object& positionLocal/*=object() */ )
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 positionLocal_;

	if(positionLocal.is_none())
		Vec3_2_pyVec3(getBodyVelocityGlobal(index), pyV);
	else
	{
		pyVec3_2_Vec3(positionLocal, positionLocal_);
		Vec3_2_pyVec3(getBodyVelocityGlobal(index, positionLocal_), pyV);
	}
	return pyV;
}

object VpDartModel::getBodyAccelerationGlobal_py(int index, const object& positionLocal )
{
	se3 genAcc;
	Vec3 positionLocal_;
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	object pyV = O.copy();

	if(positionLocal.is_none())
		Vec3_2_pyVec3(getBodyAccelerationGlobal(index), pyV);
	else
	{
		pyVec3_2_Vec3(positionLocal, positionLocal_);
		Vec3_2_pyVec3(getBodyAccelerationGlobal(index, &positionLocal_), pyV);
	}
	return pyV;
}


object VpDartModel::getBodyAngVelocityGlobal( int index )
{
	se3 genVel;
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genVel = _nodes[index]->body.GetGenVelocity();
	pyV[0] = genVel[0];
	pyV[1] = genVel[1];
	pyV[2] = genVel[2];
	return pyV;
}

object VpDartModel::getBodyAngAccelerationGlobal( int index )
{
	se3 genAcc;
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	object pyV = O.copy();

	genAcc = _nodes[index]->body.GetGenAcceleration();
	pyV[0] = genAcc[0];
	pyV[1] = genAcc[1];
	pyV[2] = genAcc[2];
	return pyV;
}

void VpDartModel::setBodyAccelerationGlobal( int index, const Vec3& acc, const Vec3* pPositionLocal)
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

void VpDartModel::setBodyAngAccelerationGlobal( int index, const object& angacc )
{
	se3 genAcc;
	genAcc = _nodes[index]->body.GetGenAcceleration();
	genAcc[0] = XD(angacc[0]);
	genAcc[1] = XD(angacc[1]);
	genAcc[2] = XD(angacc[2]);
	_nodes[index]->body.SetGenAcceleration(genAcc);
}

int VpDartModel::getBodyGeomNum(int index)
{
    return _nodes[index]->body.GetNumGeometry();
}

bp::list VpDartModel::getBodyGeomsType(int index)
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

bp::list VpDartModel::getBodyGeomsSize(int index)
{
	char type;
	scalar data[3];
	bp::list ls;

	for(int i=0; i<_nodes[index]->body.GetNumGeometry(); ++i)
    {
        ndarray O = np::array(bp::make_tuple(0.,0.,0.));
        _nodes[index]->body.GetGeometry(i)->GetShape(&type, data);
        object pyV = O.copy();
        pyV[0] = data[0];
        pyV[1] = data[1];
        pyV[2] = data[2];
        ls.append(pyV);
    }

	return ls;
}

bp::list VpDartModel::getBodyGeomsLocalFrame(int index)
{
	bp::list ls;

	for(int i=0; i<_nodes[index]->body.GetNumGeometry(); ++i)
    {
        ndarray O = np::array(bp::make_tuple(
                            bp::make_tuple(0.,0.,0.,0.),
                            bp::make_tuple(0.,0.,0.,0.),
                            bp::make_tuple(0.,0.,0.,0.),
                            bp::make_tuple(0.,0.,0.,1.)
                            )
                        );
        const SE3& geomFrame = _nodes[index]->body.GetGeometry(i)->GetLocalFrame();
        object pyT = O.copy();

        for(int j=0; j<12; j++)
            pyT[bp::make_tuple(j%3, j/3)] = geomFrame[j];

        ls.append(pyT);
    }

	return ls;
}

bp::list VpDartModel::getBodyGeomsGlobalFrame(int index)
{
	bp::list ls;

	for(int i=0; i<_nodes[index]->body.GetNumGeometry(); ++i)
    {
        ndarray O = np::array(bp::make_tuple(
                            bp::make_tuple(0.,0.,0.,0.),
                            bp::make_tuple(0.,0.,0.,0.),
                            bp::make_tuple(0.,0.,0.,0.),
                            bp::make_tuple(0.,0.,0.,1.)
                            )
                        );
        const SE3& geomFrame = _nodes[index]->body.GetGeometry(i)->GetGlobalFrame();
        object pyT = O.copy();

        for(int j=0; j<12; j++)
            pyT[bp::make_tuple(j%3, j/3)] = geomFrame[j];

        ls.append(pyT);
    }

	return ls;
}

object VpDartModel::getJointPositionGlobal( int index, const object& positionLocal/*=object() */ )
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
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
}

boost::python::object VpDartModel::getJointOrientationGlobal( int index )
{
	ndarray I = np::array( bp::make_tuple(bp::make_tuple(1.,0.,0.), bp::make_tuple(0.,1.,0.), bp::make_tuple(0.,0.,1.)) );
	SE3 bodyFrame;
	object pyR = I.copy();

	// body frame�� Inv(boneT)�� ���� joint frame ���Ѵ�
	bodyFrame = _nodes[index]->body.GetFrame();
	SE3_2_pySO3(bodyFrame * Inv(_boneTs[index]), pyR);
	return pyR;
}

object VpDartModel::getJointVelocityGlobal( int index )
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	object pyV = O.copy();

	Vec3_2_pyVec3(getBodyVelocityGlobal(index, Inv(_boneTs[index]).GetPosition()), pyV);
	return pyV;
}

boost::python::object VpDartModel::getJointAngVelocityGlobal( int index )
{
	return getBodyAngVelocityGlobal(index);
}

object VpDartModel::getJointAccelerationGlobal( int index )
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	object pyV = O.copy();
	Vec3 pospos = Inv(_boneTs[index]).GetPosition();

	Vec3_2_pyVec3(getBodyAccelerationGlobal(index, &(pospos)), pyV);
	return pyV;
}

boost::python::object VpDartModel::getJointAngAccelerationGlobal( int index )
{
	return getBodyAngAccelerationGlobal(index);
}

boost::python::object VpDartModel::getJointOrientationLocal( int index )
{
	ndarray I = np::array( bp::make_tuple(bp::make_tuple(1.,0.,0.), bp::make_tuple(0.,1.,0.), bp::make_tuple(0.,0.,1.)) );

	if(index == 0)
		return getJointOrientationGlobal(index);
	else
	{
		object pyR = I.copy();
		if(_nodes[index]->dof == 3)
		{
		    vpBJoint *joint = static_cast<vpBJoint*>(_nodes[index]->m_pJoint);
            SE3_2_pySO3(joint->GetOrientation(), pyR);
		}
		else if(_nodes[index]->dof == 1)
		{
		    vpRJoint *joint = static_cast<vpRJoint*>(_nodes[index]->m_pJoint);
		    SE3_2_pySO3(Exp(joint->GetAngle() * Vec3_2_Axis(joint->GetAxis())), pyR);
		}
		return pyR;
	}
}

boost::python::object VpDartModel::getJointAngVelocityLocal( int index )
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
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
	{
	    if( _nodes[index]->dof == 3)
	    {
		    vpBJoint *joint = static_cast<vpBJoint*>(_nodes[index]->m_pJoint);
            Vec3_2_pyVec3(joint->GetVelocity(), pyV);
		}
	    if( _nodes[index]->dof == 1)
	    {
		    vpRJoint *joint = static_cast<vpRJoint*>(_nodes[index]->m_pJoint);
            Vec3_2_pyVec3(joint->GetVelocity() * joint->GetAxis(), pyV);
	    }
    }

	return pyV;
}

boost::python::object VpDartModel::getJointAngAccelerationLocal( int index )
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
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
	{
        vpBJoint *joint = static_cast<vpBJoint*>(_nodes[index]->m_pJoint);
		Vec3_2_pyVec3(joint->GetAcceleration(), pyV);
	}

	return pyV;
}

void VpDartModel::setJointAngAccelerationLocal( int index, const object& angacc )
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
		static_cast<vpBJoint*>(_nodes[index]->m_pJoint)->SetAcceleration(pyVec3_2_Vec3(angacc));
}

void VpDartModel::setJointAccelerationGlobal( int index, const object& acc )
{
	if(index == 0)
	{
		Vec3 pospos = Inv(_boneTs[index]).GetPosition();
		setBodyAccelerationGlobal(index, pyVec3_2_Vec3(acc), &(pospos));
	}
	else
		cout << "setJointAccelerationGlobal() : not completely implemented" << endl;
}

bp::list VpDartModel::getDOFPositions()
{
//	static ndarray rootFrame( bp::make_tuple(bp::make_tuple(1.,0.,0.,0.), bp::make_tuple(0.,1.,0.,0.), bp::make_tuple(0.,0.,1.,0.), bp::make_tuple(0.,0.,0.,1.)) );
//
//	bp::list ls = getInternalJointOrientationsLocal();
//	SE3_2_pySE3(_nodes[0]->body.GetFrame() * Inv(_boneTs[0]), rootFrame);
//	ls.insert(0, rootFrame );
//	return ls;

	ndarray I = np::array( bp::make_tuple(bp::make_tuple(1.,0.,0.), bp::make_tuple(0.,1.,0.), bp::make_tuple(0.,0.,1.)) );
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));
	SE3 rootFrame;

	object pyR = I.copy();
	object pyV = O.copy();

	bp::list ls;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
	{
	    if (_nodes[i]->dof == 3)
            ls.append(getJointOrientationLocal(i));
        else if (_nodes[i]->dof == 0)
        {
            bp::tuple shape = bp::make_tuple(0);
            np::dtype dtype = np::dtype::get_builtin<float>();
            ndarray m_np = np::empty(shape, dtype);
            ls.append(m_np);
        }
	}

	rootFrame = _nodes[0]->body.GetFrame() * Inv(_boneTs[0]);

	Vec3_2_pyVec3(rootFrame.GetPosition(), pyV);
	SE3_2_pySO3(rootFrame, pyR);

	ls.insert(0, bp::make_tuple(pyV, pyR));
	return ls;
}

bp::list VpDartModel::getDOFVelocities()
{
	ndarray rootGenVel = np::array(bp::make_tuple(0.,0.,0.,0.,0.,0.));

	rootGenVel.slice(0,3) = getJointVelocityGlobal(0);
//	rootGenVel.slice(3,6) = getJointAngVelocityGlobal(0);
	rootGenVel.slice(3,6) = getJointAngVelocityLocal(0);

	bp::list ls;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
	{
	    if (_nodes[i]->dof == 3)
            ls.append(getJointAngVelocityLocal(i));
        else if (_nodes[i]->dof == 0)
        {
            bp::tuple shape = bp::make_tuple(0);
            np::dtype dtype = np::dtype::get_builtin<float>();
            ndarray m_np = np::empty(shape, dtype);
            ls.append(m_np);
        }
	}
	ls.insert(0, rootGenVel);

	return ls;
}

bp::list VpDartModel::getDOFAccelerations()
{
	ndarray rootGenAcc = np::array(bp::make_tuple(0.,0.,0.,0.,0.,0.));

	rootGenAcc.slice(0,3) = getJointAccelerationGlobal(0);
//	rootGenAcc.slice(3,6) = getJointAngAccelerationGlobal(0);
	rootGenAcc.slice(3,6) = getJointAngAccelerationLocal(0);

	bp::list ls;
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
	{
	    if (_nodes[i]->dof == 3)
            ls.append(getJointAngAccelerationLocal(i));
        else if (_nodes[i]->dof == 0)
            ls.append(bp::list());
	}

	ls.insert(0, rootGenAcc);
	return ls;
}

void VpDartModel::setDOFAccelerations( const bp::list& dofaccs)
{
	ndarray O = np::array(bp::make_tuple(0.,0.,0.));

	setJointAccelerationGlobal(0, dofaccs[0].slice(0,3));
	setJointAngAccelerationLocal(0, dofaccs[0].slice(3,6));

//	setInternalJointAngAccelerationsLocal( ((bp::list)dofaccs.slice(1,_)) );
	for(std::vector<int>::size_type i=1; i<_nodes.size(); ++i)
	{
	    if (_nodes[i]->dof == 3)
            static_cast<vpBJoint*>(_nodes[i]->m_pJoint)->SetAcceleration(pyVec3_2_Vec3(dofaccs.slice(1,_)[i-1]));
        else if(_nodes[i]->dof == 1)
            static_cast<vpRJoint*>(_nodes[i]->m_pJoint)->SetAcceleration(XD(dofaccs.slice(1, _)[i-1][0]));
    }
}

void VpDartModel::setDOFTorques(const bp::list& dofTorque)
{
	for(std::vector<Node*>::size_type i=1; i<_nodes.size(); ++i)
	{
	    //std::cout << _nodes[i]->name << std::endl;
	    //std::cout << pyVec3_2_Vec3(dofTorque[i-1]) << std::endl;
		static_cast<vpBJoint*>(_nodes[i]->m_pJoint)->SetTorque(pyVec3_2_Vec3(dofTorque[i-1]));
	}
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

static ublas::matrix<double> ToUblasMatrix(const Vec3 &v_vp)
{
    ublas::matrix<double> m(3, 1);
    for(int i=0; i<3; i++)
        m(i, 0) = v_vp[i];
    return m;
}

static ublas::matrix<double> ToUblasMatrix(const Axis &v_vp)
{
    ublas::matrix<double> m(3, 1);
    for(int i=0; i<3; i++)
        m(i, 0) = v_vp[i];
    return m;
}

static ublas::matrix<double> SE3ToUblasRotate(const SE3 &T_vp)
{
    ublas::matrix<double> T(3, 3);
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            T(j, i) = T_vp[i*3 + j];
    return T;
}

static ublas::vector<double> cross(const ublas::vector<double> &v1, const ublas::vector<double> &v2)
{
    ublas::vector<double> v(3);
    v(0) = v1(1)*v2(2) - v1(2) * v2(1);
    v(1) = v1(2)*v2(0) - v1(0) * v2(2);
    v(2) = v1(0)*v2(1) - v1(1) * v2(0);
    return v;
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

Axis GetBJointDq(const Axis &m_rQ, const Vec3 &V)
{
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

static Axis GetBJointJDQ(const Axis &m_rQ, const Axis &m_rDq)
{
    ublas::vector<double> JDQ(3);
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

	Axis V = (alpha * Inner(m_rQ, m_rDq)) * m_rQ + beta * m_rDq + gamma * Cross(m_rDq, m_rQ);
	JDQ(0) = V[0];
	JDQ(1) = V[1];
	JDQ(2) = V[2];

	return V;
}

static Axis GetBJointDJDQ(const Axis &m_rQ, const Axis &m_rDq)
{
	ublas::vector<double> DJDQ(3);
	ublas::vector<double> m_rQ_ub = ToUblasVector(m_rQ);
	scalar t = Norm(m_rQ), alpha, beta, gamma, d_alpha, d_beta, d_gamma, q_dq = Inner(m_rQ, m_rDq), t2 = t * t;
    if ( t < BJOINT_EPS )
	{
		alpha = SCALAR_1_6 - SCALAR_1_120 * t2;
		beta = SCALAR_1 - SCALAR_1_6 * t2;
		gamma = SCALAR_1_2 - SCALAR_1_24 * t2;

		d_alpha = (SCALAR_1_1260 * t2 - SCALAR_1_60) * q_dq;
		d_beta = (SCALAR_1_30 * t2 - SCALAR_1_3) * q_dq;
		d_gamma = (SCALAR_1_180 * t2 - SCALAR_1_12) * q_dq;
	} else
	{
		beta = sin(t) / t;
		alpha = (SCALAR_1 - beta) / t2;
		gamma = (SCALAR_1 - cos(t)) / t2;

		d_alpha = (gamma - SCALAR_3 * alpha) / t2 * q_dq;
		d_beta = (alpha - gamma) * q_dq;
		d_gamma = (beta - SCALAR_2 * gamma) / t2 * q_dq;
	}

    Axis dJdq = (d_alpha * q_dq + alpha * SquareSum(m_rDq)) * m_rQ
        + (alpha*q_dq + d_beta) * m_rDq
        + Cross(d_gamma*m_rDq, m_rQ);
    DJDQ(0) = dJdq[0];
    DJDQ(1) = dJdq[1];
    DJDQ(2) = dJdq[2];

    return dJdq;
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

bp::tuple VpDartModel::computeCom_J_dJdq()
{
    int body_num = this->getBodyNum();

	bp::tuple shape_J = bp::make_tuple(6*body_num, m_total_dof);
	bp::tuple shape_dJdq = bp::make_tuple(6*body_num);
    np::dtype dtype = np::dtype::get_builtin<float>();
	ndarray J = np::zeros(shape_J, dtype);
	ndarray dJdq = np::zeros(shape_dJdq, dtype);
	ublas::vector<double> offset, offset_velocity;
	ublas::matrix<double> _Jw, _Jv;
	ublas::vector<double> _dJdqw, _dJdqv;

	//preprocessing : get joint frames
	std::vector<SE3> joint_frames;
	for (std::vector<Node*>::size_type i=0; i < _nodes.size(); i++)
	    joint_frames.push_back(_nodes[i]->body.GetFrame() * Inv(_boneTs[i]));

	//root joint
	// jacobian
	Vec3 effector_position, effector_velocity;
    _Jw = SE3ToUblasRotate(joint_frames[0]);
    for(std::vector<Node*>::size_type body_idx=0; body_idx < _nodes.size(); body_idx++)
    {
        effector_position = _nodes[body_idx]->get_com_position();
        offset = ToUblasVector(effector_position - joint_frames[0].GetPosition());
        _Jv = -prod(GetCrossMatrix(offset), _Jw);
        for (int dof_index = 0; dof_index < 3; dof_index++)
        {
            J[6*body_idx + dof_index][dof_index] = 1.;
            for (int j=0; j<3; j++)
            {
//                J[6*body_idx + 0 + j][3 + dof_index] = joint_frames[0][3*dof_index + j];
                J[6*body_idx + 0 + j][3+dof_index] = _Jv(j, dof_index);
                J[6*body_idx + 3 + j][3+dof_index] = _Jw(j, dof_index);

            }
        }
    }

    // jacobian derivative
    Vec3 joint_global_pos = joint_frames[0].GetPosition();
    Vec3 joint_global_velocity = _nodes[0]->body.GetLinVelocity(Inv(_boneTs[0]).GetPosition());
    Vec3 joint_global_ang_vel = _nodes[0]->body.GetAngVelocity();

    _dJdqw = ToUblasVector(Vec3(0.));
    for(std::vector<Node*>::size_type body_idx=0; body_idx < _nodes.size(); body_idx++)
    {
        effector_position = _nodes[body_idx]->get_com_position();
        offset = ToUblasVector(effector_position - joint_global_pos);
        effector_velocity = _nodes[body_idx]->get_body_com_velocity();
        offset_velocity = ToUblasVector(effector_velocity - joint_global_velocity);
        _dJdqv = -cross(offset, _dJdqw) - cross(offset_velocity, ToUblasVector(joint_global_ang_vel));
        for (int j=0; j<3; j++)
        {
            dJdq[6*body_idx + 0 + j] += _dJdqv(j);
            dJdq[6*body_idx + 3 + j] += _dJdqw(j);
        }
    }

    //internal joint
    for(std::vector<Node*>::size_type i=1; i<_nodes.size(); i++)
    {
        int parent_joint_index = _nodes[i]->parent_index;
        int dof_start_index = _nodes[i]->dof_start_index;
        joint_global_pos = joint_frames[i].GetPosition();
        joint_global_velocity = _nodes[i]->body.GetLinVelocity(Inv(_boneTs[i]).GetPosition());
        joint_global_ang_vel = _nodes[i]->body.GetAngVelocity() - _nodes[parent_joint_index]->body.GetAngVelocity();

        if (_nodes[i]->dof == 3)
        {
            _Jw = SE3ToUblasRotate(joint_frames[i]);
            // joint_global_ang_vel = Rotate(joint_frames[i], _nodes[i]->joint.GetVelocity());
        }
        else if(_nodes[i]->dof == 1)
        {
            // revolute joint
            _Jw = prod(SE3ToUblasRotate(joint_frames[i]), ToUblasMatrix(static_cast<vpRJoint*>(_nodes[i]->m_pJoint)->GetAxis()));
            // joint_global_ang_vel = Rotate(joint_frames[i], _nodes[i]->joint_revolute.GetVelocity() * _nodes[i]->joint_revolute.GetAxis());
        }

        _dJdqw = ToUblasVector(Cross(_nodes[parent_joint_index]->body.GetAngVelocity(), joint_global_ang_vel));

        for(std::vector<Node*>::size_type body_idx=1; body_idx < _nodes.size(); body_idx++)
        {
            std::vector<bool> &is_body_ancestors = _nodes[body_idx]->is_ancestor;
            effector_position = _nodes[body_idx]->get_com_position();
            effector_velocity = _nodes[body_idx]->get_body_com_velocity();
            if(is_body_ancestors[i])
            {
                // jacobian
                offset = ToUblasVector(effector_position - joint_global_pos);
                _Jv = -prod(GetCrossMatrix(offset), _Jw);

                dof_start_index = _nodes[i]->dof_start_index;
                for (int dof_index = 0; dof_index < _nodes[i]->dof; dof_index++)
                {
                    for (int j=0; j<3; j++)
                    {
                        J[6*body_idx + 0+j][dof_start_index + dof_index] = _Jv(j, dof_index);
                        J[6*body_idx + 3+j][dof_start_index + dof_index] = _Jw(j, dof_index);
                    }
                }

                // jacobian derivatives
                offset_velocity = ToUblasVector(effector_velocity - joint_global_velocity);
                _dJdqv = -cross(offset, _dJdqw) - cross(offset_velocity, ToUblasVector(joint_global_ang_vel));
                for (int j=0; j<3; j++)
                {
                    dJdq[6*body_idx + 0 + j] += _dJdqv(j);
                    dJdq[6*body_idx + 3 + j] += _dJdqw(j);
                }
            }
        }
	}

	return bp::make_tuple(J, dJdq);
}
