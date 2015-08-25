#pragma once
#include <vector>
#include <map>
#include <gear/gear.h>

class GearWorld;

class HpGBody : public GBody
{
public:
	// each component of size represents length of the parallelpiped
	Vec3 size;
	SE3 T;
	char type; //to be implemented


	void setSize(Vec3 _size, SE3 _T = SE3()){size = _size; T = _T;}
	Vec3 getSize() const {return size;}
	SE3 getGeomTransform() const {return T;}
	SE3 getGeomGlobalFrame() {return getPoseGlobal()*T;}

	void getShape(char *_type, double *data) const {*_type = 'B'; data[0] = size[0]; data[1] = size[1]; data[2] = size[2];}
};

// number of links: n <-> number of joints: n (including root None)
//                    <-> number of internal joints: n-1
// 
// parent        <--->        child
// joint[0](=None) - link[0] - joint[1] - link[1] - ... - joint[n-1] - link[n-1]
//
// link[0]: (root body) 
class GearModel
{
public:
	enum JointDataType{
		JOINT_VALUE,
		JOINT_VEL,
		JOINT_ACC,
		JOINT_TAU,
	};
	struct Node
	{
		std::string name;
		HpGBody body;
		//vpMaterial material;
		std::vector<GJointRevolute*> joint;
		//GJointSpherical joint;
		int dof;
		bool use_joint;

		Node(std::string name_):name(name_), use_joint(false)
		{
			//body.SetMaterial(&material);
			dof = 3;
		}
	};

	~GearModel();
	void createBodiesAndJoints(const object& posture);
	void _createBodyAndJoint(const object& joint, const object& posture, GBody *parentBody, const SE3 &parentBoneTs);

	void update(const object& posture);
	void _updateJoint(const object& joint, const object& posture, std::vector<gReal> &_values);

	void getBodyInertiaLocal(int index, SE3& inT);

	Vec3 getBodyPositionGlobal(int index, const Vec3* pPositionLocal=NULL);
	Vec3 getBodyVelocityGlobal( int index, const Vec3& positionLocal=Vec3(0.,0.,0.));
	Vec3 getBodyAccelerationGlobal( int index, const Vec3* pPositionLocal=NULL);

	void getJointData(JointDataType t, std::vector<gReal> &values);
	void setJointData(JointDataType t, std::vector<gReal> &values);

	//void setBodyPositionGlobal(int index, const Vec3& position);
	//void setBodyAccelerationGlobal( int index, const Vec3& acc, const Vec3* pPositionLocal=NULL);

	void build_name2index() { for(size_t i=0; i<_nodes.size(); ++i) _name2index[_nodes[i]->name] = i; }


	GearWorld* _pWorld;
	//how about GJointFreeTs?
	// taesoo used GJointFreeTs(in his library, GJointFreeC2)
	GJointFreeTS modelJointFree;
	object _config;
	object _skeleton;

	std::vector<Node*> _nodes;
	typedef std::vector<Node*>::iterator NODES_ITOR;

	std::map<std::string, int> _name2index;

	std::vector<SE3> _boneTs;	// body position w.r.t. {joint} (SE3 but only have position info for compatibility

public:	// expose to python
	GearModel(GearWorld* pWorld, const object& createPosture, const object& config);
	std::string __str__();

	/////////////////////////////////////////////////////////////////
	// body info
	int getBodyNum() { return _nodes.size(); }
	bp::list getBodyMasses();
	gReal getTotalMass();
	object getBodyShape(int index);
	bp::list getBodyVerticesPositionGlobal(int index);

	/////////////////////////////////////////////////////////////////
	// index converter
	std::string index2name(int index) { return _nodes[index]->name; }
	//int index2id(int index) { return _nodes[index]->body.GetID(); }
	int name2index(const std::string& name) { if(_name2index.find(name)!=_name2index.end()) return _name2index.find(name)->second; else return -1; }
	//int name2id(const std::string& name) { return index2id(name2index(name)); }

	/////////////////////////////////////////////////////////////////
	// body inertia
	object getBodyInertiaLocal_py(int index);
	object getBodyInertiaGlobal_py(int index);

	bp::list getBodyInertiasLocal();
	bp::list getBodyInertiasGlobal();

	/////////////////////////////////////////////////////////////////
	// body
	object getBodyPositionGlobal_py( int index, const object& positionLocal=object() );
	object getBodyVelocityGlobal_py( int index, const object& positionLocal=object() );
	object getBodyAccelerationGlobal_py(int index, const object& positionLocal=object() );
	object getBodyOrientationGlobal( int index );
	object getBodyAngVelocityGlobal( int index );
	object getBodyAngAccelerationGlobal( int index );

	bp::list getBodyPositionsGlobal();
	bp::list getBodyVelocitiesGlobal();
	bp::list getBodyAccelerationsGlobal();
	bp::list getBodyAngVelocitiesGlobal();
	bp::list getBodyAngAccelerationsGlobal();

	//void setBodyPositionGlobal_py( int index, const object& pos );
	//void setBodyVelocityGlobal_py( int index, const object& pos );
	//void setBodyAccelerationGlobal_py( int index, const object& acc );
	//void setBodyAngVelocityGlobal( int index, const object& angvel );
	//void setBodyAngAccelerationGlobal( int index, const object& angacc );

	/////////////////////////////////////////////////////////////////
	// model
	void translateByOffset(const object& offset);
	void rotate(const object& rotation);
};


class GearControlModel : public GearModel
{
private:
//	vector<int> _jointElementIndexes;

	//void ignoreCollisionBtwnBodies();
	//void addBodiesToWorld(const object& createPosture);
////	void buildJointIndexes();

public:	// expose to python
	GearControlModel(GearWorld* pWorld, const object& createPosture, const object& config);
	std::string __str__();

	int getJointNum() { return _nodes.size(); }
	int getInternalJointNum() { return _nodes.size()-1; }

	bp::list getDOFs();
	int getTotalDOF();
	bp::list getInternalJointDOFs();
	int getTotalInternalJointDOF();

	///////////////////////////////////////////////////////////////////
	//// hybrid dynamics
	void initializeHybridDynamics(bool floatingBase=true);
	void solveHybridDynamics();

	///////////////////////////////////////////////////////////////////
	//// DOF value

	//// [T_g[0], R_l[1], R_l[2], ... ,R_l[n-1]]
	bp::list getDOFPositions();

	//// [lv_g[0]<hmerge>av_l[0], av_l[1], av_l[2], ... av_l[n-1]]
	//bp::list getDOFVelocities();

	//// [la_g[0]<hmerge>aa_l[0], aa_l[1], aa_l[2], ... aa_l[n-1]]
	//bp::list getDOFAccelerations();

	//// [I<vmerge>R_g[0], R_l[1]^t, R_l[2]^t, ... R_l[n-1]^t]
	//bp::list getDOFAxeses();

	//// [T_l[0], R_l[1], R_l[2], ... ,R_l[n-1]]
	//bp::list getDOFPositionsLocal();

	//// [lv_l[0]<hmerge>av_l[0], av_l[1], av_l[2], ... av_l[n-1]]
	//bp::list getDOFVelocitiesLocal();

	//// [la_l[0]<hmerge>aa_l[0], aa_l[1], aa_l[2], ... aa_l[n-1]]
	//bp::list getDOFAccelerationsLocal();

	//// [I<vmerge>R_l[0], R_l[1]^t, R_l[2]^t, ... R_l[n-1]^t]
	//bp::list getDOFAxesesLocal();

	bp::list getDOFPositionsEuler();
	bp::list getDOFVelocitiesEuler();
	bp::list getDOFAccelerationsEuler();
	bp::list getDOFAxesesEuler();
	
	//void setDOFAccelerations(const bp::list& dofaccs);

	///////////////////////////////////////////////////////////////////
	//// joint
	object getJointOrientationLocal( int index );
	//object getJointAngVelocityLocal( int index );
	//object getJointAngAccelerationLocal( int index );

	//object getJointPositionGlobal(int index);
	//object getJointVelocityGlobal(int index);
	//object getJointVelocityLocal(int index);
	//object getJointAccelerationGlobal(int index);
	//object getJointAccelerationLocal(int index);
	object getJointOrientationGlobal( int index );
	//object getJointAngVelocityGlobal( int index );
	//object getJointAngAccelerationGlobal( int index );


	//bp::list getJointOrientationsLocal();
	//bp::list getJointAngVelocitiesLocal();
	//bp::list getJointAngAccelerationsLocal();
	
	//bp::list getJointPositionsGlobal();
	//bp::list getJointVelocitiesGlobal();
	//bp::list getJointAccelerationsGlobal();
	//bp::list getJointOrientationsGlobal();
	//bp::list getJointAngVelocitiesGlobal();
	//bp::list getJointAngAccelerationsGlobal();


	bp::list getInternalJointOrientationsLocal();
	//bp::list getInternalJointAngVelocitiesLocal();
	//bp::list getInternalJointAngAccelerationsLocal();

	//bp::list getInternalJointPositionsGlobal();
	//bp::list getInternalJointOrientationsGlobal();
	//// bp::list getInternalJointAngVelocitiesGlobal();
	//// bp::list getInternalJointAngAccelerationsGlobal();


	//void setJointAngVelocityLocal( int index, const object& angvel );
	//void setJointAngAccelerationLocal(int index, const object& angacc);

	//void setJointAccelerationGlobal( int index, const object& acc );
	//void setJointAngAccelerationGlobal(int index, const object& angacc);

	//// void setJointOrientationsLocal()
	//// void setJointAngVelocitiesLocal()
	//void setJointAngAccelerationsLocal( const bp::list& angaccs );

	//void setInternalJointAngAccelerationsLocal( const bp::list& angaccs );

	///////////////////////////////////////////////////////////////////
	//// joint force
	//object getJointTorqueLocal( int index);
	//bp::list getInternalJointTorquesLocal();

	//void setJointTorqueLocal( int index, const object& torque);
	//void setInternalJointTorquesLocal(const bp::list& torques);

	///////////////////////////////////////////////////////////////////
	//// body force
	//void applyBodyGenForceGlobal(int index, const object& torque, const object& force, const object& positionLocal=object());
	//void applyBodyForceGlobal(int index, const object& force, const object& positionLocal=object());
	//void applyBodyTorqueGlobal(int index, const object& torque );

	//object getBodyForceLocal( int index );
	//object getBodyNetForceLocal( int index );
	//object getBodyGravityForceLocal( int index );

	//bp::list getEquationOfMotion(object& M, object& b);
	////void stepKinematics( double dt, const object& acc);
	void stepKinematics( double dt, const bp::list& accs);
	void calcMassMatrix3(object &M, object &b);
};
