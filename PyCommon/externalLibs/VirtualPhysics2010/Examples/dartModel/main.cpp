//
// Created by trif on 2018. 8. 30..
//

#include <VP/vphysics.h>
#include "../../vpRenderer/vpBasicRenderer.h"
#include <tinyxml2.h>
#include <vector>
#include <string>
#include <sstream>

using namespace tinyxml2;

class Node
{
public:
    string name;
    vpBody body;
    vpMaterial material;
    vpBJoint joint;
    vpRJoint joint_revolute;
    vpWJoint joint_weld;
    int dof;
    int dof_start_index;
    bool use_joint;
    unsigned char color[4];
    std::vector<int> ancestors;  // contains itself
    std::vector<bool> is_ancestor;  // is_ancestor[ancestor_idx] = true;
    int parent_index;
    Vec3 offset_from_parent;

    Node(string name_):name(name_), use_joint(false)
    {
        parent_index = -1;
        body.SetMaterial(&material);
        dof = 3;
        color[0] = 0;
        color[1] = 0;
        color[2] = 0;
        color[3] = 255;
    }
    SE3 get_body_frame(){return this->body.GetFrame();}
    Vec3 get_body_position(){return this->body.GetFrame().GetPosition();}
    Vec3 get_body_com_velocity(){return this->body.GetLinVelocity(Vec3(0.));}
};

vpWorld		*world;
string name;
std::vector<Node *> _nodes;
std::vector<SE3> _boneTs;

void skel_init(const char*);

void initialize(void)
{
    skel_init("cart_pole_blade.skel");
    VP_BASIC_RENDERER_WORLD(*world);
    set_view(1.00, -0.00, 0.01, 0.00, 0.01, 0.09, -1.00, 0.00, 0.00, 1.00, 0.09, 0.00, 0.00, 0.00, -14.51, 1.00);
}

void frame(void)
{
//    world->StepAhead();
//    printf(10, 10, "t = %3.2f   E = %3.2f(Ek = %3.2f   Ev = %3.2f)", world.GetSimulationTime(), world.GetKineticEnergy() + world.GetPotentialEnergy(), world.GetKineticEnergy(), world.GetPotentialEnergy());
//    printf(10, 30, "angle = %3.2f   g = (%3.2f, %3.2f, %3.2f)", joint.GetAngle(), world.GetGravity()[0], world.GetGravity()[1], world.GetGravity()[2]);
//    printf(10, 480, "press SPACEBAR to change gravity direction");
}

void keyboard(unsigned char key, int x, int y)
{
//    if ( key == ' ' ) world.SetGravity(-1.0 * world.GetGravity());
    if ( key == ' ' ) for(int i=0; i<40; i++)world->StepAhead();
}

void ignoreCollisionBtwnBodies()
{
    for (auto pNode0 : _nodes)
        for (auto pNode1 : _nodes)
            world->IgnoreCollision(&pNode0->body, &pNode1->body);
}


void getFloats(const char* text, int num, std::vector<float> &floats)
{
    string text_str(text);
    std::istringstream _in(text_str);      //make a stream for the line itself
    float x, y, z, u, v, w;
    floats.clear();
    if (num == 2)
    {
        _in >> x >> y;
        floats.push_back(x);
        floats.push_back(y);
    }
    if (num == 3)
    {
        _in >> x >> y >> z;
        floats.push_back(x);
        floats.push_back(y);
        floats.push_back(z);
    }
    else if (num == 6)
    {
        _in >> x >> y >> z >> u >> v >> w;
        floats.push_back(x);
        floats.push_back(y);
        floats.push_back(z);
        floats.push_back(u);
        floats.push_back(v);
        floats.push_back(w);
    }
}

SE3 dof6_to_SE3(std::vector<float> &floats)
{
    return EulerXYZ(Vec3(floats[3], floats[4], floats[5]), Vec3(floats[0], floats[1], floats[2]));
}


void skel_init(const char *skel_path)
{
    XMLDocument xmlDoc;
    XMLError eResult = xmlDoc.LoadFile(skel_path);

    XMLElement *pSkel = xmlDoc.FirstChildElement();

    XMLElement *pWorld = pSkel->FirstChildElement();
    world = new vpWorld;

    // physics
    XMLElement *pPhysics = pWorld->FirstChildElement("physics");

    XMLElement *pTime_step = pPhysics->FirstChildElement("time_step");
    float time_step;
    pTime_step->QueryFloatText(&time_step);
    XMLElement *pGravity = pPhysics->FirstChildElement("gravity");
    world->SetTimeStep(time_step);

    std::vector<float> gravity;
    getFloats(pGravity->GetText(), 3, gravity);
    world->SetGravity(Vec3(gravity[0], gravity[1], gravity[2]));

    // skeleton
    XMLElement *pSkeleton = pWorld->FirstChildElement("skeleton");
    name = pSkeleton->Attribute("name");
    if(name.find(string("ground")) != std::string::npos)
    {
        pSkeleton = pSkeleton->NextSiblingElement("skeleton");
        name = pSkeleton->Attribute("name");
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
//        world->AddBody(&(_nodes[i]->body));
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
        int child_node_idx = 0, parent_node_idx = 0;
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
            joint_dof_index += 6;
            _boneTs[child_node_idx] = Inv(joint_frame[i]);
            world->AddBody(&(_nodes[child_node_idx]->body));
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

        std::cout << "TinyXml Debug: "<< joint_name[i] << joint_type[i] << joint_parent[i] << joint_child[i] << std::endl;
        std::cout << "TinyXml Debug: "<< parent_node_idx << " " << child_node_idx << std::endl;

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
            _nodes[child_node_idx]->joint.m_szName = joint_name[i];
            _nodes[child_node_idx]->dof = 3;
            _nodes[child_node_idx]->dof_start_index = joint_dof_index;
            _nodes[child_node_idx]->use_joint = true;
            joint_dof_index += 3;
            _nodes[parent_node_idx]->body.SetJoint(&(_nodes[child_node_idx]->joint), parent_body_to_joint);
            _nodes[child_node_idx]->body.SetJoint(&(_nodes[child_node_idx]->joint), child_body_to_joint);
            _boneTs[child_node_idx] = joint_to_child_body;

        }
        else if(joint_type[i] == "weld")
        {
            _nodes[child_node_idx]->joint_weld.m_szName = joint_name[i];
            _nodes[child_node_idx]->dof = 0;
            _nodes[child_node_idx]->dof_start_index = joint_dof_index;
            _nodes[child_node_idx]->use_joint = false;
            _nodes[parent_node_idx]->body.SetJoint(&(_nodes[child_node_idx]->joint_weld), parent_body_to_joint);
            _nodes[child_node_idx]->body.SetJoint(&(_nodes[child_node_idx]->joint_weld), child_body_to_joint);
            _boneTs[child_node_idx] = joint_to_child_body;
        }
        else
        {
            std::cout << "WARNING!! : " << joint_type[i] << " is not implemented or not supported!" << std::endl;
            continue;
        }
    }

    ignoreCollisionBtwnBodies();
    std::cout << "TinyXml Debug: world init" << std::endl;
    world->Initialize();
    std::cout << "TinyXml Debug: world init end" << std::endl;
//	world->SetIntegrator(VP::IMPLICIT_EULER_FAST);
    world->SetIntegrator(VP::EULER);
}
