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
    SE3                      GetOrientation(){return Transform();}
    
    /*!
        return angular velocity in body frame
     */
    Vec3                     GetVelocityLocal(){return GetVelocity(0)*GetAxis(0) + GetVelocity(1)*GetAxis(1);}
    /*!
        return angular acceleration in body frame
     */
    Vec3                     GetAccelerationLocal(){
                                    Axis axis[2]; Vec3 _axis[2];
                                    _axis[0] = GetAxis(0); _axis[1] = GetAxis(1);
                                    axis[0] = Axis(_axis[0][0], _axis[0][1], _axis[0][2]);
                                    axis[1] = Axis(_axis[1][0], _axis[1][1], _axis[1][2]);
                                    // m_sDSdq = (m_rDq[0] * m_rDq[1]) * ad(m_sS[0], m_sS[1]);
                                    Axis m_sDsdq = (GetVelocity(0)*GetVelocity(1))*ad(axis[0], axis[1]);
                                    Vec3 _m_sDsdq(m_sDsdq[0], m_sDsdq[1], m_sDsdq[2]);
                                    return GetAxis(0)*GetAcceleration(0) + GetAxis(1)*GetAcceleration(1) + _m_sDsdq;
                                    }

    void                    SetVelocityLocal(const Vec3& _vel){
                                    Vec3 axis[2], vel = _vel;
                                    axis[0] = GetAxis(0); axis[1] = GetAxis(1);
                                    scalar th0 = Inner(vel, axis[0]);
                                    scalar th1 = Inner(vel, axis[1]);
                                    SetVelocity(0, th0); SetVelocity(1, th1);
                                    }
    void                    SetAccelerationLocal(const Vec3& _acc){
                                    Axis axis[2]; Vec3 _axis[2];
                                    _axis[0] = GetAxis(0); _axis[1] = GetAxis(1);
                                    axis[0] = Axis(_axis[0][0], _axis[0][1], _axis[0][2]);
                                    axis[1] = Axis(_axis[1][0], _axis[1][1], _axis[1][2]);
                                    Axis m_sDsdq = (GetVelocity(0)*GetVelocity(1))*ad(axis[0], axis[1]);
                                    Vec3 _m_sDsdq(m_sDsdq[0], m_sDsdq[1], m_sDsdq[2]);
                                    Vec3 acc = _acc - _m_sDsdq;
                                    SetAcceleration(0, Inner(acc, _axis[0]));
                                    SetAcceleration(1, Inner(acc, _axis[1]));
                                    }
    
    void                     SetTorqueLocal(const Vec3 &tor){
                                                SetTorque(0, Inner(tor, GetAxis(0)));
                                                SetTorque(1, Inner(tor, GetAxis(1)));}
    Vec3                     GetTorqueLocal(){return GetTorque(0)*GetAxis(0) + GetTorque(1)*GetAxis(1);}

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
        backupAcc[i] = GetAcceleration(i);
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
            SetAcceleration(i, backupAcc[i]);
            SetTorque(i, backupTau[i]);
        }

    }
}

#endif
