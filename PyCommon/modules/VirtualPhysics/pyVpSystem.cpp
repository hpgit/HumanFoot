#include "stdafx.h"

#include "pyVpSystem.h"
#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"
#include <VP/vpDataType.h>


#define make_tuple boost::python::make_tuple

BOOST_PYTHON_MODULE(vpSystem)
{
    class_<pyVpSystem, boost::shared_ptr<pyVpSystem>, boost::noncopyable>("vpSystem") 
    // class_<pyVpSystem>("vpSystem", init<>())
        .def("self", &pyVpSystem::self, return_value_policy<reference_existing_object>())
        .def("GetNumJoint", &pyVpSystem::GetNumJoint_py)
        .def("GetNumBody", &pyVpSystem::GetNumBody_py)
        .def("GetKineticEnergy", &pyVpSystem::GetKineticEnergy_py)
        .def("GetPotentialEnergy", &pyVpSystem::GetPotentialEnergy_py)
        .def("BackupState", &pyVpSystem::BackupState_py)
        .def("RollbackState", &pyVpSystem::RollbackState_py)
        .def("ForwardDynamics", &pyVpSystem::ForwardDynamics_py)
        .def("ForwardDynamics2", &pyVpSystem::ForwardDynamics2_py)
        .def("InverseDynamics", &pyVpSystem::InverseDynamics_py)
        .def("HybridDynamics", &pyVpSystem::HybridDynamics_py)
        ;

        bp::objects::class_value_wrapper< 
            boost::shared_ptr<vpSystem> , 
            bp::objects::make_ptr_instance<vpSystem, 
                bp::objects::pointer_holder<boost::shared_ptr<vpSystem>,vpSystem> > >();
}


/*********************
vpSystem wrapper
*********************/

pyVpSystem& pyVpSystem::self()
{
    return *this;
}

int pyVpSystem::GetNumJoint_py(void)
{
    return GetNumJoint();
}

int pyVpSystem::GetNumBody_py(void)
{
    return GetNumBody();
}

scalar pyVpSystem::GetKineticEnergy_py(void)
{
    return GetKineticEnergy();
}

scalar pyVpSystem::GetPotentialEnergy_py(void)
{
    return GetPotentialEnergy();
}

void pyVpSystem::BackupState_py(void)
{
    return BackupState();
}

void pyVpSystem::RollbackState_py(void)
{
    RollbackState();
}

void pyVpSystem::ForwardDynamics_py(void)
{
    ForwardDynamics();
}

void pyVpSystem::ForwardDynamics2_py(void)
{
    ForwardDynamics2();
}

void pyVpSystem::InverseDynamics_py(void)
{
    InverseDynamics();
}

void pyVpSystem::HybridDynamics_py(void)
{
    HybridDynamics();
}
