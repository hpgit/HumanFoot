
#ifndef _CLASS_NODE
#define _CLASS_NODE

#include "LinearR3.h"

enum Purpose {JOINT, EFFECTOR};

class VectorR3;

namespace IK_sdls
{

class Node {

	friend class Tree;

public:
	Node(const VectorR3&, const VectorR3&, double, Purpose, double minTheta=-PI, double maxTheta=PI, double restAngle=0.);

	void PrintNode(int depth);
	void InitNode();

	const VectorR3& GetAttach() const { return attach; }

	double GetTheta() const { return theta; }
	double AddToTheta( double delta ) { theta += delta; return theta; }

	const VectorR3& GetS() const { return s; }
	const VectorR3& GetW() const { return w; }

	double GetMinTheta() const { return minTheta; }
	double GetMaxTheta() const { return maxTheta; } 
	double GetRestAngle() const { return restAngle; } ;
	void SetTheta(double newTheta) { theta = newTheta; }
	void ComputeS(void);
	void ComputeW(void);

	bool IsEffector() const { return purpose==EFFECTOR; } 
	bool IsJoint() const { return purpose==JOINT; }
	int GetEffectorNum() const { return seqNumEffector; }
	int GetJointNum() const { return seqNumJoint; }
	int GetParentJointNum() const { if(realparent) return realparent->GetJointNum(); return -1;}

	bool IsFrozen() const { return freezed; }
	void Freeze() { freezed = true; }
	void UnFreeze() { freezed = false; }

	Node* left;				// left child
	Node* right;			// right sibling
	Node* realparent;		// pointer to real parent

private:
	bool freezed;			// Is this node frozen?
	int seqNumJoint;		// sequence number if this node is a joint
	int seqNumEffector;		// sequence number if this node is an effector
	double size;			// size
	Purpose purpose;		// joint / effector / both
	VectorR3 attach;		// attachment point
	VectorR3 r;				// relative position vector
	VectorR3 v;				// rotation axis
	double theta;			// joint angle (radian)
	double minTheta;		// lower limit of joint angle
	double maxTheta;		// upper limit of joint angle
	double restAngle;		// rest position angle
	VectorR3 s;				// GLobal Position
	VectorR3 w;				// Global rotation axis
};
}
#endif