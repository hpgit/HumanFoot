#include "stdafx.h"

#include "pyVpGeom.h"
#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"
#include <VP/vpDataType.h>


#define make_tuple boost::python::make_tuple

BOOST_PYTHON_MODULE(vpGeom)
{
    numeric::array::set_module_and_type("numpy", "ndarray");

    class_<vpGeom>("vpGeom", init<>())
        .def("self", &pyVpBody::self, return_value_policy<reference_existing_object>())
        //.def("SetJoint(vpJoint *J, const SE3 &T = SE3(0));
        .def("ApplyGlobalForce", &pyVpBody::ApplyGlobalForce_py)
        .def("ApplyLocalForce_py", &pyVpBody::ApplyLocalForce_py, pyVpBody_ApplyLocalForce_py_overloads())
        .def("ResetForce", &pyVpBody::ResetForce_py)
        //.def("SetInertia(const Inertia &);
        //.def("GetInertia(void) const;
        //.def("SetJointFrame(vpJoint *J, const SE3 &T);
        //.def("GetJointFrame(const vpJoint *) const;
        .def("SetFrame", &pyVpBody::SetFrame_py)
        .def("GetFrame", &pyVpBody::GetFrame_py)
        .def("SetVelocity", &pyVpBody::SetVelocity_py)
        .def("SetAngularVelocity", &pyVpBody::SetAngularVelocity_py)
        .def("SetGenVelocity", &pyVpBody::SetGenVelocity_py)
        .def("SetGenVelocityLocal", &pyVpBody::SetGenVelocityLocal_py)
        .def("SetGenAcceleration", &pyVpBody::SetGenAcceleration_py)
        .def("SetGenAccelerationLocal", &pyVpBody::SetGenAccelerationLocal_py)
        .def("GetGenVelocity", &pyVpBody::GetGenVelocity_py)
        .def("GetGenVelocityLocal", &pyVpBody::GetGenVelocityLocal_py)
        .def("GetLinVelocity", &pyVpBody::GetLinVelocity_py)
        .def("GetAngVelocity", &pyVpBody::GetAngVelocity_py)
        .def("GetGenAcceleration", &pyVpBody::GetGenAcceleration_py)
        .def("GetGenAccelerationLocal", &pyVpBody::GetGenAccelerationLocal_py)
        .def("IsCollidable", &pyVpBody::IsCollidable_py)
        .def("IsGround", &pyVpBody::IsGround_py)
        .def("SetCollidable", &pyVpBody::SetCollidable_py)
        //.def("AddGeometry", &pyVpBody::AddGeometry_py)
        .def("GetBoundingSphereRadius", &pyVpBody::GetBoundingSphereRadius_py)
        //.def("SetMaterial", &pyVpBody::SetMaterial_py)
        //.def("GetMaterial(void) const;
        .def("GetCenterOfMass", &pyVpBody::GetCenterOfMass_py)
        //.def("GenerateDisplayList", &pyVpBody::GenerateDisplayList_py)
        .def("GetForce", &pyVpBody::GetForce_py)
        .def("GetNetForce", &pyVpBody::GetNetForce_py)
        .def("GetGravityForce", &pyVpBody::GetGravityForce_py)
        .def("IsSetInertia", &pyVpBody::IsSetInertia_py)
        .def("GetNumGeometry", &pyVpBody::GetNumGeometry_py)
        //.def("GetGeometry(int) const;
        .def("GetID", &pyVpBody::GetID_py)
        .def("SetGround", &pyVpBody::SetGround_py, pyVpBody_SetGround_py_overloads())
        //.def("ApplyGravity", &pyVpBody::ApplyGravity_py, pyVpBody_ApplyGravity_py_overloads())
        //.def("IsApplyingGravity", &pyVpBody::IsApplyingGravity_py)
        .def("GetWorld", &pyVpBody::GetWorld_py, return_value_policy<reference_existing_object>())
        .def("DetectCollisionApprox", &pyVpBody::DetectCollisionApprox_py)
        //.def("GetSystem", &pyVpBody::GetSystem_py)
        .def("SetHybridDynamicsType", &pyVpBody::SetHybridDynamicsType_py)
        .def("GetHybridDynamicsType", &pyVpBody::GetHybridDynamicsType_py)
        .def("BackupState", &pyVpBody::BackupState_py)
        .def("RollbackState", &pyVpBody::RollbackState_py)
        .def("UpdateGeomFrame", &pyVpBody::UpdateGeomFrame_py)
        ;
}
