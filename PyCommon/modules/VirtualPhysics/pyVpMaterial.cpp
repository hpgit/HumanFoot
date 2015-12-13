#include "stdafx.h"

#include "pyVpMaterial.h"
#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"
#include <VP/vpDataType.h>

#define make_tuple boost::python::make_tuple

BOOST_PYTHON_MODULE(vpMaterial)
{
    class_<pyVpMaterial>("vpMaterial", init<>())
        .def("self", &pyVpMaterial::self, return_value_policy<reference_existing_object>())
        .def("GetDensity", &pyVpMaterial::GetDensity_py)
        .def("SetDensity", &pyVpMaterial::SetDensity_py)
        .def("GetRestitution", &pyVpMaterial::GetRestitution_py)
        .def("SetRestitution", &pyVpMaterial::SetRestitution_py)
        .def("GetStaticFriction", &pyVpMaterial::GetStaticFriction_py)
        .def("SetStaticFriction", &pyVpMaterial::SetStaticFriction_py)
        .def("GetDynamicFriction", &pyVpMaterial::GetDynamicFriction_py)
        .def("SetDynamicFriction", &pyVpMaterial::SetDynamicFriction_py)
        .def("GetSpinningFriction", &pyVpMaterial::GetSpinningFriction_py)
        .def("SetSpinningFriction", &pyVpMaterial::SetSpinningFriction_py)
        .def("GetDefaultMaterial", &pyVpMaterial::GetDefaultMaterial_py, return_value_policy<reference_existing_object>())
    ;
}

pyVpMaterial& pyVpMaterial::self()
{
    return *this;
}

scalar pyVpMaterial::GetDensity_py(void)
{
    return GetDensity();
}

void pyVpMaterial::SetDensity_py(scalar pyD)
{
    SetDensity(pyD);
}

scalar pyVpMaterial::GetRestitution_py(void)
{
    return GetRestitution();
}

void pyVpMaterial::SetRestitution_py(scalar pyD)
{
    SetRestitution(pyD);
}

scalar pyVpMaterial::GetStaticFriction_py(void)
{
    return GetStaticFriction();
}

void pyVpMaterial::SetStaticFriction_py(scalar pyD)
{
    SetStaticFriction(pyD);
}

scalar pyVpMaterial::GetDynamicFriction_py(void)
{
    return GetDynamicFriction();
}

void pyVpMaterial::SetDynamicFriction_py(scalar pyD)
{
    SetDynamicFriction(pyD);
}

scalar pyVpMaterial::GetSpinningFriction_py(void)
{
    return GetSpinningFriction();
}

void pyVpMaterial::SetSpinningFriction_py(scalar pyD)
{
    SetSpinningFriction(pyD);
}

pyVpMaterial& pyVpMaterial::GetDefaultMaterial_py(void)
{
    return *(reinterpret_cast<pyVpMaterial*>(GetDefaultMaterial()));
}
