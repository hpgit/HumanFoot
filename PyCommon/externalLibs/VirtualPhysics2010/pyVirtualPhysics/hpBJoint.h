
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
    
    /*!
        return angular velocity in local frame
     */
    Vec3                     GetVelocityLocal(){return GetVelocity();}

    /*!
        return angular acceleration in local frame
     */
    Vec3                     GetAccelerationLocal(){return GetAcceleration();}
	void					 SetVelocityLocal(const Vec3 &vel){SetVelocity(vel);}
	void					 SetAccelerationLocal(const Vec3 &acc){SetAcceleration(acc);}

    void                     SetTorqueLocal(const Vec3 &tor){SetTorque(tor);}
    Vec3                     GetTorqueLocal(){return GetTorque();}

    void                     BackupAccTau();
    void                     RestoreAccTau();

    bool                     backup;
    Vec3                     backupAcc;
    Vec3                     backupTau;
};


void hpBJoint::BackupAccTau()
{
    backup = true;
    backupAcc = GetAcceleration();
    backupTau = GetTorque();
}

void hpBJoint::RestoreAccTau()
{
    backup = false;
    SetAcceleration(backupAcc);
    SetTorque(backupTau);
}


#endif
