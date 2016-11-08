
#ifndef HP_BJOINT
#define HP_BJOINT

#include <VP/vpBJoint.h>

class hpBJoint : public vpBJoint
{
public:
	scalar                   GetDisplacement(int i){return m_rQ[i];}
	scalar                   GetFirstDeriv(int i){return m_rDq[i];}
	scalar                   GetSecondDeriv(int i){return m_rDdq[i];}
	scalar                   GetGenTorque(int i){return m_rActuationTau[i];}
	void                     SetSecondDeriv(int i, const scalar &a){m_rDdq[i] = a;}
	void                     SetGenTorque(int i, const scalar &a){m_rActuationTau[i] = a;}
    SE3                      GetTransform(){return Transform();}

};


#endif