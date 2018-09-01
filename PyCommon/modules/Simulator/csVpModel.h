#ifndef _CS_VP_MODEL_H_
#define _CS_VP_MODEL_H_

#include <vector>
#include <VP/vphysics.h>
#include "../VirtualPhysics/pyVpBody.h"
#include "../VirtualPhysics/pyVpJoint.h"

class VpWorld;

// number of links: n <-> number of joints: n (including root None)
//                    <-> number of internal joints: n-1
// 
// parent        <--->        child
// joint[0](=None) - link[0] - joint[1] - link[1] - ... - joint[n-1] - link[n-1]
//
// link[0]: (root body) 
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

class VpModel
{
public:
	~VpModel();
	void createBodies(const object& posture);
	void _createBody(const object& joint, const SE3& parentT, const object& posture);

	void getBodyInertiaLocal(int index, SE3& inT);

	Vec3 getBodyPositionGlobal(int index, const Vec3* pPositionLocal=NULL);
	Vec3 getBodyVelocityGlobal( int index, const Vec3& positionLocal=Vec3(0.,0.,0.));
	Vec3 getBodyAccelerationGlobal( int index, const Vec3* pPositionLocal=NULL);


	void setBodyPositionGlobal(int index, const Vec3& position);
	void setBodyAccelerationGlobal( int index, const Vec3& acc, const Vec3* pPositionLocal=NULL);

	void build_name2index() { for(std::vector<int>::size_type i=0; i<_nodes.size(); ++i) _name2index[_nodes[i]->name] = i; }


	vpWorld* _pWorld;
	object _config;
	object _skeleton;

	vector<Node*> _nodes;
	typedef vector<Node*>::iterator NODES_ITOR;

	map<string, int> _name2index;
	map<int, int> _id2index;

	vector<SE3> _boneTs;	// joint position, orientation -> body position, orientation (body�� �߽����� body�� position)

public:	// expose to python
	VpModel(VpWorld* pWorld, const object& createPosture, const object& config);
	string __str__();

	/////////////////////////////////////////////////////////////////
	// body info
	int getBodyNum() { return _nodes.size(); }
	bp::list getBodyMasses();
	scalar getTotalMass();
	bp::list getBodyVerticesPositionGlobal(int index);

	int getBodyGeomNum(int index);
	bp::list getBodyGeomsType(int index);
	bp::list getBodyGeomsSize(int index);
	bp::list getBodyGeomsLocalFrame(int index);
	bp::list getBodyGeomsGlobalFrame(int index);
	object getBodyShape(int index);

	pyVpBody &getBodyByIndex(int index){ return *reinterpret_cast<pyVpBody*> (&_nodes[index]->body);}
	pyVpBody &getBodyByName(std::string name){ return *reinterpret_cast<pyVpBody*> (&_nodes[name2index(name)]->body);}
	pyVpJoint &getJointByIndex(int index){ return *reinterpret_cast<pyVpJoint*> (&_nodes[index]->joint);}
	pyVpJoint &getJointByName(std::string name){ return *reinterpret_cast<pyVpJoint*> (&_nodes[name2index(name)]->joint);}

	/////////////////////////////////////////////////////////////////
	// index converter
	string index2name(int index) { return _nodes[index]->name; }
	int index2vpid(int index) { return _nodes[index]->body.GetID(); }
	int name2index(const string& name) { if(_name2index.find(name)!=_name2index.end()) return _name2index.find(name)->second; else return -1; }
	int name2vpid(const string& name) { return index2vpid(name2index(name)); }
	int vpid2index(int id);
	// int id2index(int id) { return _id2index[id]; }

	/////////////////////////////////////////////////////////////////
	// body inertia
	object getBodyInertiaLocal_py(int index);
	object getBodyInertiaGlobal_py(int index);

	bp::list getBodyInertiasLocal();
	bp::list getBodyInertiasGlobal();

	/////////////////////////////////////////////////////////////////
	// body
	object getCOM();
	bp::list getBoneT(int index);
	bp::list getInvBoneT(int index);

    object getBodyFrame(int index);

	object getBodyGenVelLocal(int index);
	object getBodyGenVelGlobal(int index);
	object getBodyGenAccLocal(int index);
	object getBodyGenAccGlobal(int index);
	object getBodyPositionGlobal_py( int index, const object& positionLocal=object() );
	object getBodyVelocityGlobal_py( int index, const object& positionLocal=object() );
	object getBodyAccelerationGlobal_py(int index, const object& positionLocal=object() );
	object getBodyAngVelocityGlobal( int index );
	object getBodyAngAccelerationGlobal( int index );

	object getBodyOrientationGlobal(int index);

	bp::list getBodyPositionsGlobal();
	bp::list getBodyVelocitiesGlobal();
	bp::list getBodyAccelerationsGlobal();
	bp::list getBodyAngVelocitiesGlobal();
	bp::list getBodyAngAccelerationsGlobal();

	object getBodyTransformGlobal(int index);

	void setBodyPositionGlobal_py( int index, const object& pos );
	void setBodyVelocityGlobal_py( int index, const object& pos );
	void setBodyAccelerationGlobal_py( int index, const object& acc );
	void setBodyAngVelocityGlobal( int index, const object& angvel );
	void setBodyAngAccelerationGlobal( int index, const object& angacc );
	void SetGround(int index, bool flag);

	/////////////////////////////////////////////////////////////////
	// model
	void translateByOffset(const object& offset);
	void rotate(const object& rotation);

	// collision
	void ignoreCollisionWith(vpBody* pBody);
	void ignoreCollisionWith_py(vpBody* pBody);


	/////////////////////////////////////////
	// Additional
	void addBody(bool flagControl);
	void SetBodyColor(int id, unsigned char r, unsigned char g, unsigned char b, unsigned char a);
};

class VpMotionModel : public VpModel
{
private:
	bool _recordVelByFiniteDiff;
	scalar _inverseMotionTimeStep;
	void _updateBody(const object& joint, const SE3& parentT, const object& posture);

public:	// expose to python
	VpMotionModel(VpWorld* pWorld, const object& createPosture, const object& config);
	void update(const object& posture);
	void recordVelByFiniteDiff(bool flag=true, scalar motionTimeStep=1/30.) { _recordVelByFiniteDiff = flag; _inverseMotionTimeStep = 1./motionTimeStep; }
};

class VpControlModel : public VpModel
{
public:
	int m_total_dof;
//	vector<int> _jointElementIndexes;

	void ignoreCollisionBtwnBodies();
	void addBodiesToWorld(const object& createPosture);
	void createJoints(const object& posture);
	void _createJoint(const object& joint, const object& posture, const std::vector<int> &parent_ancestors);
	void _updateJoint(const object& joint, const object& posture);
//	void buildJointIndexes();
	
	vector<vpSpring*> _springs;

public:	// expose to python
	VpControlModel(VpWorld* pWorld, const object& createPosture, const object& config);
	string __str__();

	int getJointNum() { return _nodes.size(); }
	int getInternalJointNum() { return _nodes.size()-1; }

	bp::list getDOFs();
	int getTotalDOF();
	bp::list getInternalJointDOFs();
	int getTotalInternalJointDOF();

	bp::list getJointDOFIndexes(int index);

	void update(const object& posture);
	void fixBody(int index);

	/////////////////////////////////////////////////////////////////
	// hybrid dynamics
	void initializeHybridDynamics(bool floatingBase=true);
	void initializeForwardDynamics();
	void setHybridDynamics(int jointIndex, std::string dynamicsType);
	void solveHybridDynamics();
	void solveForwardDynamics();
	void solveInverseDynamics();

	/////////////////////////////////////////////////////////////////
	// DOF value

    void set_q(const object& q);
	bp::list get_q();
	bp::list get_dq();
	void set_ddq(const object& ddq);
	void set_ddq_vp(const std::vector<double>& ddq);

	bp::list get_dq_nested();

	// [T_g[0], R_l[1], R_l[2], ... ,R_l[n-1]]
	bp::list getDOFPositions();

	// [lv_g[0]<hmerge>av_l[0], av_l[1], av_l[2], ... av_l[n-1]]
	bp::list getDOFVelocities();

	// [la_g[0]<hmerge>aa_l[0], aa_l[1], aa_l[2], ... aa_l[n-1]]
	bp::list getDOFAccelerations();

	// [I<vmerge>R_g[0], R_l[1]^t, R_l[2]^t, ... R_l[n-1]^t]
	bp::list getDOFAxeses();

	// [T_l[0], R_l[1], R_l[2], ... ,R_l[n-1]]
	bp::list getDOFPositionsLocal();

	// [lv_l[0]<hmerge>av_l[0], av_l[1], av_l[2], ... av_l[n-1]]
	bp::list getDOFVelocitiesLocal();

	// [la_l[0]<hmerge>aa_l[0], aa_l[1], aa_l[2], ... aa_l[n-1]]
	bp::list getDOFAccelerationsLocal();

	// [I<vmerge>R_l[0], R_l[1]^t, R_l[2]^t, ... R_l[n-1]^t]
	bp::list getDOFAxesesLocal();



	// [lv_l[0]<hmerge>av_l[0], av_l[1], av_l[2], ... av_l[n-1]]
	bp::list getBodyRootDOFVelocitiesLocal();

	// [la_l[0]<hmerge>aa_l[0], aa_l[1], aa_l[2], ... aa_l[n-1]]
	bp::list getBodyRootDOFAccelerationsLocal();

	// [I<vmerge>R_l[0], R_l[1]^t, R_l[2]^t, ... R_l[n-1]^t]
	bp::list getBodyRootDOFAxeses();


	void setDOFAccelerations(const bp::list& dofaccs);
	void setDOFTorques(const bp::list& dofTorque);

	/////////////////////////////////////////////////////////////////
	// joint
	object getJointTransform(int index);

	object getJointOrientationLocal( int index );
	object getJointAngVelocityLocal( int index );
	object getJointAngAccelerationLocal( int index );

	object getJointAfterTransformGlobal(int index);
	object getJointPositionGlobal( int index, const object& positionLocal=object() );
//	object getJointPositionGlobal(int index);
	object getJointVelocityGlobal(int index);
	object getJointAccelerationGlobal(int index);
	object getJointOrientationGlobal( int index );
	object getJointAngVelocityGlobal( int index );
	object getJointAngAccelerationGlobal( int index );

	object getJointFrame(int index);

	object getJointVelocityLocal(int index);
	object getJointAccelerationLocal(int index);

	bp::list getJointOrientationsLocal();
	bp::list getJointAngVelocitiesLocal();
	bp::list getJointAngAccelerationsLocal();
	
	bp::list getJointPositionsGlobal();
	bp::list getJointVelocitiesGlobal();
	bp::list getJointAccelerationsGlobal();
	bp::list getJointOrientationsGlobal();
	bp::list getJointAngVelocitiesGlobal();
	bp::list getJointAngAccelerationsGlobal();


	bp::list getInternalJointOrientationsLocal();
	bp::list getInternalJointAngVelocitiesLocal();
	bp::list getInternalJointAngAccelerationsLocal();

	bp::list getInternalJointPositionsGlobal();
	bp::list getInternalJointOrientationsGlobal();
	// bp::list getInternalJointAngVelocitiesGlobal();
	// bp::list getInternalJointAngAccelerationsGlobal();


	void setJointAngVelocityLocal( int index, const object& angvel );
	void setJointAngAccelerationLocal(int index, const object& angacc);

	void setJointAccelerationGlobal( int index, const object& acc );
	void setJointAngAccelerationGlobal(int index, const object& angacc);

	// void setJointOrientationsLocal()
	// void setJointAngVelocitiesLocal()
	void setJointAngAccelerationsLocal( const bp::list& angaccs );

	void setInternalJointAngAccelerationsLocal( const bp::list& angaccs );

    /////////////////////////////////////////////////////////////////
	// joint dynamics
	void SetJointElasticity(int index, scalar Kx, scalar Ky=-1, scalar Kz=-1);
	void SetJointsElasticity(scalar Kx, scalar Ky=-1, scalar Kz=-1);
	void SetJointDamping(int index, scalar Kx, scalar Ky=-1, scalar Kz=-1);
	void SetJointsDamping(scalar Kx, scalar Ky=-1, scalar Kz=-1);

	/////////////////////////////////////////////////////////////////
	// joint force
	object getJointTorqueLocal( int index);
	bp::list getInternalJointTorquesLocal();

	void setJointTorqueLocal( int index, const object& torque);
	void setInternalJointTorquesLocal(const bp::list& torques);

	/////////////////////////////////////////////////////////////////
	// body force
	void applyBodyGenForceGlobal(int index, const object& torque, const object& force, const object& positionLocal=object());
	void applyBodyForceGlobal(int index, const object& force, const object& positionLocal=object());
	void applyBodyTorqueGlobal(int index, const object& torque );

	object getBodyForceLocal( int index );
	object getBodyNetForceLocal( int index );
	object getBodyGravityForceLocal( int index );

	/////////////////////////////////////////////////////////////////
	// jacobian
	object getLocalJacobian(int index);
	object getLocalJointVelocity(int index);
	object getLocalJointDisplacementDerivatives(int index);
	object computeJacobian(int index, const object& positionGlobal);
	bp::tuple computeCom_J_dJdq();

	/////////////////////////////////////////
	// Additional
	//void addBody();
	void setSpring(int body1Index, int body2Index, scalar elasticity, scalar damping, const object& p1, const object& p2, scalar initialDistance = 0.0);

	bp::list getInverseEquationOfMotion(object& invM, object& invMb); // use this
	bp::list getEquationOfMotion(object& M, object& b);// buggy
	//void stepKinematics( double dt, const object& acc);
	void stepKinematics( double dt, const bp::list& accs);
};


class VpDartModel : public VpControlModel
{
private:
    string name;
    void skel_init(const char *skel_path);
    scalar _planeHeight;
    scalar _lockingVel;

public:
    VpDartModel(const char *skel_path) : VpControlModel(nullptr, object(), object()){_lockingVel=0.; _planeHeight=0.; skel_init(skel_path);}
    ~VpDartModel(){delete _pWorld;}
    void step();
	bp::tuple calcPenaltyForce(const bp::list& bodyIDsToCheck, const bp::list& mus, scalar Ks, scalar Ds);
	bool _calcPenaltyForce(const vpBody* pBody, const Vec3& position, const Vec3& velocity, Vec3& force, scalar Ks, scalar Ds, scalar mu);
	void applyPenaltyForce(const bp::list& bodyIDs, const bp::list& positions, const bp::list& forces);
};

#endif  // _CS_VP_MODEL_H_
