#ifndef _CS_VP_DART_MODEL_H_
#define _CS_VP_DART_MODEL_H_

#include <vector>
#include <map>
#include <VP/vphysics.h>

class Node
{
public:
    string name;
    vpBody body;
    vpMaterial material;
//    vpBJoint joint;
//    vpRJoint joint_revolute;
//    vpWJoint joint_weld;
    vpJoint *m_pJoint;
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
        m_pJoint = nullptr;
    }
    ~Node()
    {
        if(m_pJoint)
            delete m_pJoint;
    }
    SE3 get_body_frame(){return body.GetFrame();}
    Vec3 get_body_position(){return body.GetFrame().GetPosition();}
    Vec3 get_body_velocity(){return body.GetLinVelocity(Vec3(0.));}

    SE3 get_com_frame()
    {
        Vec3 com = body.GetCenterOfMass();
        return body.GetFrame() * SE3(com);
    }
    Vec3 get_com_position()
    {
        Vec3 com = body.GetCenterOfMass();
        return body.GetFrame() * com;
    }
    Vec3 get_body_com_velocity()
    {
        Vec3 com = body.GetCenterOfMass();
        return body.GetLinVelocity(com);
    }
};

class VpDartModel
{
private:
	vpWorld* _pWorld;
    string name;
	int m_total_dof;
	int m_joint_num;
    scalar _planeHeight;
    scalar _lockingVel;
	object _config;
	object _skeleton;

	vector<Node*> _nodes;

	std::map<std::string, int> _name2index;
	std::map<int, int> _id2index;

	vector<SE3> _boneTs;	// joint position, orientation -> body position, orientation (body�� �߽����� body�� position)

    void skel_init(const char *skel_path);

public:
    VpDartModel(const char *skel_path) : name(""), m_total_dof(0), m_joint_num(0), _planeHeight(0.), _lockingVel(0.) {skel_init(skel_path);}
    ~VpDartModel();

    void step();

    scalar getTimeStep(){return _pWorld->GetTimeStep();}
    object getGravity();

	bp::tuple calcPenaltyForce(const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds);
	bool _calcPenaltyForce(const vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu);
	void applyPenaltyForce(const bp::list& bodyIDs, const bp::list& positions, const bp::list& forces);

	void initializeHybridDynamics();
	void initializeForwardDynamics();
	void setHybridDynamics(int jointIndex, std::string dynamicsType);
	void solveHybridDynamics();
	void solveForwardDynamics();
	void solveInverseDynamics();

	void translateByOffset(const object& offset);

	std::string index2name(int index){return _nodes[index]->name;}
	int index2vpid(int index){return _nodes[index]->body.GetID();}
	int getJointIndex(std::string name){return _name2index[name];}
	int vpid2index(int vpid){return _id2index[vpid];}

	int getBodyNum() { return _nodes.size(); }
    int getTotalDOF(){ return m_total_dof; }
    bp::list getDOFs();
    bp::list getJointDOFIndexes(int index);
    bp::list getJointDOFIndexesByName(std::string name){return getJointDOFIndexes(_name2index[name]);}
    bp::list getJointDOFInfo();
    int getJointNum(){ return (int)_nodes.size();}
    scalar getTotalMass();
    bp::list getBodyMasses();

	void getBodyInertiaLocal(int index, SE3& inT);
	object getBodyInertiaLocal_py(int index);
	object getBodyInertiaGlobal_py(int index);

	void setBodyPositionGlobal(int index, const Vec3& position);
	Vec3 getBodyPositionGlobal(int index, const Vec3* pPositionLocal=NULL);
	Vec3 getBodyVelocityGlobal( int index, const Vec3& positionLocal=Vec3(0.,0.,0.));
	Vec3 getBodyAccelerationGlobal( int index, const Vec3* pPositionLocal=NULL);

    object getCOM();
	object getBodyComPositionGlobal( int index );
	object getBodyComVelocityGlobal( int index );

	object getBodyFrame(int index);
	object getBodyPositionGlobal_py( int index, const object& positionLocal=object() );
	object getBodyOrientationGlobal(int index);
	object getBodyVelocityGlobal_py( int index, const object& positionLocal=object() );
	object getBodyAccelerationGlobal_py(int index, const object& positionLocal=object() );
	object getBodyAngVelocityGlobal( int index );
	object getBodyAngAccelerationGlobal( int index );

    void setBodyAngAccelerationGlobal( int index, const object& angacc );
    void setBodyAccelerationGlobal( int index, const Vec3& acc, const Vec3* pPositionLocal);

	int getBodyGeomNum(int index);
	bp::list getBodyGeomsType(int index);
	bp::list getBodyGeomsSize(int index);
	bp::list getBodyGeomsLocalFrame(int index);
	bp::list getBodyGeomsGlobalFrame(int index);

	object getJointPositionGlobal( int index, const object& positionLocal=object() );
	object getJointOrientationGlobal( int index );
	object getJointVelocityGlobal( int index );
	object getJointAngVelocityGlobal( int index );
	object getJointAccelerationGlobal( int index );
	object getJointAngAccelerationGlobal( int index );

	object getJointOrientationLocal( int index );
	object getJointAngVelocityLocal( int index );
	object getJointAngAccelerationLocal( int index );

	void setJointAngAccelerationLocal(int index, const object& angacc);
	void setJointAccelerationGlobal( int index, const object& acc );

	bp::list getDOFPositions();
	bp::list getDOFVelocities();
	bp::list getDOFAccelerations();

	void setDOFAccelerations(const bp::list& dofaccs);
	void setDOFTorques(const bp::list& dofTorque);

	void set_q(const object& q);
	object get_q();
	object get_dq();
	object get_ddq();
	void set_ddq(const object& ddq);

	boost::python::numpy::ndarray get_force();
	void set_force(const object& force);

	bp::tuple computeCom_J_dJdq();

	object getMassMatrix();
	boost::python::numpy::ndarray getCoriAndGrav();
};

#endif  // _CS_VP_DART_MODEL_H_
