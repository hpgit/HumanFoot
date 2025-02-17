
#ifndef HP_BJOINT
#define HP_BJOINT

#include <VP/vpBJoint.h>

class hpBJoint : public vpBJoint
{
public:
    hpBJoint():vpBJoint(){backup = false;}
	scalar                   GetDisplacement(int i){return m_rQ[i];}
	scalar                   GetFirstDeriv(int i){return m_rDq[i];}
	scalar                   GetSecondDeriv(int i){return m_rDdq[i];}
	scalar                   GetGenTorque(int i){return m_rActuationTau[i];}
	void                     SetSecondDeriv(int i, const scalar &a){m_rDdq[i] = a;}
	void                     SetGenTorque(int i, const scalar &a){m_rActuationTau[i] = a;}
    SE3                      GetTransform(){return Transform();}

    void                     BackupAccTau();
    void                     RestoreAccTau();

    boolean                  backup;
    Vec3                      backupAcc;
    Vec3                     backupTau;
};


void hpBJoint::BackupAccTau()
{
    backup = true;
    backupAcc = GetGenAcceleration();
    backupTau = GetTorque();
}

void hpBJoint::RestoreAccTau()
{
    backup = false;
    SetGenAcceleration(backupAcc);
    SetTorque(backupTau);
}


#endif