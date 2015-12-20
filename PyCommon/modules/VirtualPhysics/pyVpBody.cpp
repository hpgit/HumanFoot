#include "stdafx.h"

#include "pyVpBody.h"
#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"
#include <VP/vpDataType.h>

#ifndef make_tuple
#define make_tuple boost::python::make_tuple
#endif

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(pyVpBody_AddGeometry_py_overloads, AddGeometry_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(pyVpBody_SetJoint_py_overloads, SetJoint_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(pyVpBody_ApplyLocalForce_py_overloads, ApplyLocalForce_py, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(pyVpBody_SetGround_py_overloads, SetGround_py, 0, 1);
//BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(pyVpBody_ApplyGravity_py_overloads, ApplyGravity_py, 0, 1);

BOOST_PYTHON_MODULE(vpBody)
{
    numeric::array::set_module_and_type("numpy", "ndarray");

    class_<pyVpBody, boost::shared_ptr<pyVpBody>, boost::noncopyable>("vpBody") 
    // class_<pyVpBody>("vpBody", init<>())
        .def("self", &pyVpBody::self, return_value_policy<reference_existing_object>())
        .def("SetJoint", &pyVpBody::SetJoint_py, pyVpBody_SetJoint_py_overloads() )
        .def("ApplyGlobalForce", &pyVpBody::ApplyGlobalForce_py)
        .def("ApplyLocalForce_py", &pyVpBody::ApplyLocalForce_py, pyVpBody_ApplyLocalForce_py_overloads())
        .def("ResetForce", &pyVpBody::ResetForce_py)
        //.def("SetInertia(const Inertia &);
        //.def("GetInertia(void) const;
        .def("SetJointFrame", &pyVpBody::SetJointFrame_py)
        .def("GetJointFrame", &pyVpBody::GetJointFrame_py)
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
        .def("AddGeometry", &pyVpBody::AddGeometry_py, pyVpBody_AddGeometry_py_overloads())
        .def("GetBoundingSphereRadius", &pyVpBody::GetBoundingSphereRadius_py)
        .def("SetMaterial", &pyVpBody::SetMaterial_py)
        .def("GetMaterial", &pyVpBody::GetMaterial_py, return_value_policy<reference_existing_object>())
        .def("GetCenterOfMass", &pyVpBody::GetCenterOfMass_py)
        //.def("GenerateDisplayList", &pyVpBody::GenerateDisplayList_py)
        .def("GetForce", &pyVpBody::GetForce_py)
        .def("GetNetForce", &pyVpBody::GetNetForce_py)
        .def("GetGravityForce", &pyVpBody::GetGravityForce_py)
        .def("IsSetInertia", &pyVpBody::IsSetInertia_py)
        .def("GetNumGeometry", &pyVpBody::GetNumGeometry_py)
        .def("GetGeometry", &pyVpBody::GetGeometry_py, return_value_policy<reference_existing_object>())
        .def("GetID", &pyVpBody::GetID_py)
        .def("SetGround", &pyVpBody::SetGround_py, pyVpBody_SetGround_py_overloads())
        //.def("ApplyGravity", &pyVpBody::ApplyGravity_py, pyVpBody_ApplyGravity_py_overloads())
        //.def("IsApplyingGravity", &pyVpBody::IsApplyingGravity_py)
        .def("GetWorld", &pyVpBody::GetWorld_py, return_value_policy<reference_existing_object>())
        .def("DetectCollisionApprox", &pyVpBody::DetectCollisionApprox_py)
        .def("GetSystem", &pyVpBody::GetSystem_py, return_value_policy<reference_existing_object>())
        .def("SetHybridDynamicsType", &pyVpBody::SetHybridDynamicsType_py)
        .def("GetHybridDynamicsType", &pyVpBody::GetHybridDynamicsType_py)
        .def("BackupState", &pyVpBody::BackupState_py)
        .def("RollbackState", &pyVpBody::RollbackState_py)
        .def("UpdateGeomFrame", &pyVpBody::UpdateGeomFrame_py)
        ;
        bp::objects::class_value_wrapper< 
            boost::shared_ptr<vpBody> , 
            bp::objects::make_ptr_instance<vpBody, 
                bp::objects::pointer_holder<boost::shared_ptr<vpBody>,vpBody> > >();
}


/*********************
vpBody wrapper
*********************/

pyVpBody& pyVpBody::self()
{
    return *this;
}

//TODO:
void pyVpBody::SetJoint_py(vpJoint *J, const object &pyT)
{
    if (pyT == object())
        SetJoint(J);
    else
    {
        SE3 T = pySE3_2_SE3(pyT);
        SetJoint(J, T);
    }
}

void pyVpBody::ApplyGlobalForce_py(object &pyF, object &pyP)
{
    Vec3 p = pyVec3_2_Vec3(pyP);
    if (checkPyVlen(pyF, 6))
    {
        dse3 F = pyVec6_2_dse3(pyF);
        ApplyGlobalForce(F, p);
    }
    else if(checkPyVlen(pyF, 3))
    {
        Vec3 F = pyVec3_2_Vec3(pyF);
        ApplyGlobalForce(F, p);
    }
    else
        std::cout << "Error:length is not proper" <<std::endl;
}

void pyVpBody::ApplyLocalForce_py(object &pyF, const object &pyP)
{
    if (pyP == object())
    {
        Vec3 p = pyVec3_2_Vec3(pyP);
        if (checkPyVlen(pyF, 6))
        {
            dse3 F = pyVec6_2_dse3(pyF);
            ApplyLocalForce(F, p);
        }
        else if(checkPyVlen(pyF, 3))
        {
            Vec3 F = pyVec3_2_Vec3(pyF);
            ApplyLocalForce(F, p);
        }
        else
            std::cout << "Error:length is not proper" <<std::endl;
    }
    else
    {
        Vec3 VecM = pyVec3_2_Vec3(pyF);
        Axis M(VecM[0], VecM[1], VecM[2]);
        ApplyLocalForce(M);
    }

}

void pyVpBody::ResetForce_py(void)
{
    ResetForce();
}

//TODO:
// void SetInertia_py(const Inertia &);
// const Inertia &GetInertia_py(void) const;

void pyVpBody::SetJointFrame_py(vpJoint *J, const object &pyT)
{
    SE3 T = pySE3_2_SE3(pyT);
    SetJointFrame(J, T);
}

object pyVpBody::GetJointFrame_py(vpJoint *J)
{
    SE3 T = GetJointFrame(J);
    object pyT;
    make_pySE3(pyT);
    SE3_2_pySE3(T, pyT);
    return pyT;
}

void pyVpBody::SetFrame_py(object &pySE3)
{
    SE3 T = pySE3_2_SE3(pySE3);
    SetFrame(T);
}

object pyVpBody::GetFrame_py(void)
{
    SE3 T = GetFrame();
	numeric::array O(make_tuple(make_tuple(0., 0., 0., 0.),
	            make_tuple(0., 0., 0., 0.),
	            make_tuple(0., 0., 0., 0.),
	            make_tuple(0.,0.,0.,0.)));
	object pySE3 = O.copy();
	SE3_2_pySE3(T, pySE3);

    return pySE3;
}

void pyVpBody::SetVelocity_py(object &pyV)
{
    if(checkPyVlen(pyV, 3))
    {
        Vec3 V = pyVec3_2_Vec3(pyV);
        SetVelocity(V);
    }
    else
        std::cout << "Error:length is not proper" <<std::endl;
}

void pyVpBody::SetAngularVelocity_py(object &pyV)
{
    if(checkPyVlen(pyV, 3))
    {
        Vec3 V = pyVec3_2_Vec3(pyV);
        SetAngularVelocity(V);
    }
    else
        std::cout << "Error:length is not proper" <<std::endl;
}

void pyVpBody::SetGenVelocity_py(object &pyV)
{ 
    if(checkPyVlen(pyV, 6))
    {
        se3 V = pyVec6_2_se3(pyV);
        SetGenVelocity(V);
    }
    else
        std::cout << "Error:length is not proper" <<std::endl;
}

void pyVpBody::SetGenVelocityLocal_py(object &pyV)
{ 
    if(checkPyVlen(pyV, 6))
    {
        se3 V = pyVec6_2_se3(pyV);
        SetGenVelocityLocal(V);
    }
    else
        std::cout << "Error:length is not proper" <<std::endl;
}

void pyVpBody::SetGenAcceleration_py(object &pyV)
{ 
    if(checkPyVlen(pyV, 6))
    {
        se3 V = pyVec6_2_se3(pyV);
        SetGenAcceleration(V);
    }
    else
        std::cout << "Error:length is not proper" <<std::endl;
}

void pyVpBody::SetGenAccelerationLocal_py(object &pyV)
{
    if(checkPyVlen(pyV, 6))
    {
        se3 V = pyVec6_2_se3(pyV);
        SetGenAccelerationLocal(V);
    }
    else
        std::cout << "Error:length is not proper" <<std::endl;
}

object pyVpBody::GetGenVelocity_py(void)
{
    se3 V = GetGenVelocity();
    numeric::array O(make_tuple(0., 0., 0., 0., 0., 0.));
    object pyV = O.copy();
    se3_2_pyVec6(V, pyV);
    return pyV;
}

object pyVpBody::GetGenVelocityLocal_py(void)
{
    se3 V = GetGenVelocityLocal();
    numeric::array O(make_tuple(0., 0., 0., 0., 0., 0.));
    object pyV = O.copy();
    se3_2_pyVec6(V, pyV);
    return pyV;
}

object pyVpBody::GetLinVelocity_py(object &pyP)
{
    Vec3 p = pyVec3_2_Vec3(pyP);
    Vec3 V = GetLinVelocity(p);
    numeric::array O(make_tuple(0., 0., 0.));
    object pyV = O.copy();
    Vec3_2_pyVec3(V, pyV);
    return pyV;
}

object pyVpBody::GetAngVelocity_py(void)
{
    Vec3 V = GetAngVelocity();
    numeric::array O(make_tuple(0., 0., 0.));
    object pyV = O.copy();
    Vec3_2_pyVec3(V, pyV);
    return pyV;
}

object pyVpBody::GetGenAcceleration_py(void)
{
    se3 V = GetGenAcceleration();
    numeric::array O(make_tuple(0., 0., 0., 0., 0., 0.));
    object pyV = O.copy();
    se3_2_pyVec6(V, pyV);
    return pyV;
}

object pyVpBody::GetGenAccelerationLocal_py(void)
{
    se3 V = GetGenAccelerationLocal();
    numeric::array O(make_tuple(0., 0., 0., 0., 0., 0.));
    object pyV = O.copy();
    se3_2_pyVec6(V, pyV);
    return pyV;
}

bool pyVpBody::IsCollidable_py(void)
{
    return IsCollidable();
}

bool pyVpBody::IsGround_py(void)
{
    return IsGround();
}

void pyVpBody::SetCollidable_py(bool pyB)
{
    SetCollidable(pyB);
}

void pyVpBody::AddGeometry_py(vpGeom *pGeom, const object &pyT)
{
    if (pyT == object())
    {
        AddGeometry(pGeom);
    }
    else
    {
        SE3 T = pySE3_2_SE3(pyT);
        AddGeometry(pGeom, T);
    }

}

scalar pyVpBody::GetBoundingSphereRadius_py(void)
{
    return GetBoundingSphereRadius();
}

void pyVpBody::SetMaterial_py(const vpMaterial *M)
{
    SetMaterial(M);
}

const vpMaterial &pyVpBody::GetMaterial_py(void)
{
    return *(GetMaterial());
}

object pyVpBody::GetCenterOfMass_py(void)
{
    Vec3 V = GetCenterOfMass();
    numeric::array O(make_tuple(0., 0., 0.));
    object pyV = O.copy();
    Vec3_2_pyVec3(V, pyV);
    return pyV;
}

// void pyVpBody::GenerateDisplayList_py(bool pyB)
// {
    // GenerateDisplayList(pyB);
// }

object pyVpBody::GetForce_py(void)
{
    dse3 F = GetForce();
    numeric::array O(make_tuple(0., 0., 0., 0., 0., 0.));
    object pyV = O.copy();
    dse3_2_pyVec6(F, pyV);
    return pyV;
}

object pyVpBody::GetNetForce_py(void)
{
    dse3 F = GetNetForce();
    numeric::array O(make_tuple(0., 0., 0., 0., 0., 0.));
    object pyV = O.copy();
    dse3_2_pyVec6(F, pyV);
    return pyV;
}

object pyVpBody::GetGravityForce_py(void)
{
    dse3 F = GetGravityForce();
    numeric::array O(make_tuple(0., 0., 0., 0., 0., 0.));
    object pyV = O.copy();
    dse3_2_pyVec6(F, pyV);
    return pyV;
}

bool pyVpBody::IsSetInertia_py(void)
{
    return IsSetInertia();
}

int	pyVpBody::GetNumGeometry_py(void)
{
    return GetNumGeometry();
}

//TODO:
//boost::shared_ptr<vpGeom> pyVpBody::GetGeometry_py(int geomIdx)
vpGeom& pyVpBody::GetGeometry_py(int geomIdx)
{
    // return boost::shared_ptr<vpGeom>(GetGeometry(geomIdx));
    return *(GetGeometry(geomIdx));
}

int	pyVpBody::GetID_py(void)
{
    return GetID();
}

void pyVpBody::SetGround_py(bool pyB)
{
    return SetGround(pyB);
}

// void pyVpBody::ApplyGravity_py(bool pyB)
// {
//     return ApplyGravity(pyB);
// }

// bool pyVpBody::IsApplyingGravity_py(void)
// {
//     return IsApplyingGravity();
// }

const vpWorld &pyVpBody::GetWorld_py(void)
{
    return *( GetWorld());
    // return *const_cast<pyVpWorld*>((reinterpret_cast<const pyVpWorld *>( GetWorld())));
}

bool pyVpBody::DetectCollisionApprox_py(object &pBody)
{
    return DetectCollisionApprox(reinterpret_cast<vpBody *> (&pBody));
}

vpSystem &pyVpBody::GetSystem_py(void)
{
    return *(GetSystem());
}

void pyVpBody::SetHybridDynamicsType_py(std::string typeStr)
{
    if(typeStr == "KINEMATIC")
        SetHybridDynamicsType(VP::KINEMATIC);
    else if(typeStr == "DYNAMIC")
        SetHybridDynamicsType(VP::DYNAMIC);
    std::cout << "Error: not available Hybrid Dynamics Type." << std::endl;
}

std::string pyVpBody::GetHybridDynamicsType_py(void)
{
    VP::HD_TYPE type = GetHybridDynamicsType();
    if (type == VP::DYNAMIC)
        return std::string("DYNAMIC");
    else
        return std::string("KINEMATIC");
}

void pyVpBody::BackupState_py(void)
{
    BackupState();
}

void pyVpBody::RollbackState_py(void)
{
    RollbackState();
}

void pyVpBody::UpdateGeomFrame_py(void)
{
    UpdateGeomFrame();
}
