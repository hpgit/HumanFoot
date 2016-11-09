#ifndef HP_RJOINT
#define HP_RJOINT

#include <VP/vpDataType.h>
#include <VP/vpRJoint.h>

class hpRJoint : public vpRJoint
{
public:
    hpRJoint():vpRJoint(){backup = false;}
	scalar                   GetDisplacement(){return m_rQ;}
	scalar                   GetFirstDeriv(){return m_rDq;}
	scalar                   GetSecondDeriv(){return m_rDdq;}
	scalar                   GetGenTorque(){return m_rActuationTau;}
	void                     SetSecondDeriv( const scalar &a){m_rDdq = a;}
	void                     SetGenTorque(const scalar &a){m_rActuationTau = a;}
    SE3                      GetTransform(){return Transform();}

    void                     BackupAccTau();
    void                     RestoreAccTau();

    boolean                  backup;
    scalar                   backupAcc;
    scalar                   backupTau;
};

void hpRJoint::BackupAccTau()
{
    backup = true;
    backupAcc = GetAngle();
    backupTau = GetTorque();
}

void hpRJoint::RestoreAccTau()
{
    if (backup)
    {
        backup = false;
        SetAngle(backupAcc);
        SetTorque(backupTau);
    }
}


#endif