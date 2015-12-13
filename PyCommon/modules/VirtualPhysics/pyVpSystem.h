#ifndef _PYVPSYSTEM_H_
#define _PYVPSYSTEM_H_

#include <VP/vpDataType.h>

class pyVpSystem : public vpSystem 
{
public:
    pyVpSystem&             self();
    /*!
        get a number of joints in the system.
    */
    int              GetNumJoint_py(void);

    /*!
        get a number of bodies in the system.
    */
    int              GetNumBody_py(void);

    /*!
        get a kinetic energy of the system.
    */
    scalar           GetKineticEnergy_py(void);

    /*!
        get a potential energy of the system.
    */
    scalar           GetPotentialEnergy_py(void);

    void             BackupState_py(void);
    void             RollbackState_py(void);

    void             ForwardDynamics_py(void);
    void             ForwardDynamics2_py(void);
    void             InverseDynamics_py(void);
    void             HybridDynamics_py(void);

};


#endif // _PYVPSYSTEM_H_