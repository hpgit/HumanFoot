#ifndef HP_UJOINT
#define HP_UJOINT

#include <VP/vpDataType.h>
#include <VP/vpUJoint.h>

class hpUJoint : public vpUJoint
{
public:
    hpUJoint():vpUJoint(){backup = false;}
	scalar                   GetDisplacement(int i){return m_rQ[i];}
	scalar                   GetFirstDeriv(int i){return m_rDq[i];}
	scalar                   GetSecondDeriv(int i){return m_rDdq[i];}
	scalar                   GetGenTorque(int i){return m_rActuationTau[i];}
	void                     SetSecondDeriv(int i, const scalar &a){m_rDdq[i] = a;}
	void                     SetGenTorque(int i, const scalar &a){m_rActuationTau[i] = a;}
    SE3                      GetTransform(){return Transform();}

    void                     BackupAccTau();
    void                     RestoreAccTau();

    bool                     backup;
    scalar                   backupAcc[2];
    scalar                   backupTau[2];
};

void hpUJoint::BackupAccTau()
{
    backup = true;
    for(int i=0; i<2; i++)
    {
        backupAcc[i] = GetAngle(i);
        backupTau[i] = GetTorque(i);
    }
}

void hpUJoint::RestoreAccTau()
{
    if (backup)
    {
        backup = false;
        for(int i=0; i<2; i++)
        {
            SetAngle(i, backupAcc[i]);
            SetTorque(i, backupTau[i]);
        }

    }
}

#endif
