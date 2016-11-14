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
    SE3                      GetOrientation(){return Transform();}

    /*!
        return angular velocity in local frame
     */
    Vec3                     GetVelocityLocal(){return GetAxis()*GetVelocity();}

    /*!
        return angular acceleration in local frame
     */
    Vec3                     GetAccelerationLocal(){return GetAxis()*GetAcceleration();}

    void                    SetVelocityLocal(const Vec3& _vel){
                                    // Vec3 axis = GetAxis(), vel = _vel;
                                    // SetVelocity(Inner(vel, axis));}
                                    SetVelocity(Inner(_vel, GetAxis()));}
    void                    SetAccelerationLocal(const Vec3& _acc){
                                    // Vec3 axis = GetAxis(), acc = _acc;
                                    // SetAcceleration(Inner(acc, axis));}
                                    SetAcceleration(Inner(_acc, GetAxis()));}

    Vec3                     GetTorqueLocal(){return GetAxis()*GetTorque();}
    void                     SetTorqueLocal(const Vec3 &tor){SetTorque(Inner(tor, GetAxis()));}

    void                     BackupAccTau();
    void                     RestoreAccTau();

    bool                     backup;
    scalar                   backupAcc;
    scalar                   backupTau;
};

void hpRJoint::BackupAccTau()
{
    backup = true;
    backupAcc = GetAcceleration();
    backupTau = GetTorque();
}

void hpRJoint::RestoreAccTau()
{
    if (backup)
    {
        backup = false;
        SetAcceleration(backupAcc);
        SetTorque(backupTau);
    }
}


#endif
