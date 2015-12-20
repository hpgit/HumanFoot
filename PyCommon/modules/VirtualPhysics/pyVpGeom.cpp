#include "stdafx.h"

#include "pyVpGeom.h"
#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
#include "../../../PyCommon/externalLibs/common/VPUtil.h"
#include <VP/vpDataType.h>


#define make_tuple boost::python::make_tuple

BOOST_PYTHON_MODULE(vpGeom)
{
    //numeric::array::set_module_and_type("numpy", "ndarray");
    class_<pyVpGeom, boost::shared_ptr<pyVpGeom>, boost::noncopyable>("vpGeom") 
    .def("GetInertia", bp::pure_virtual(&vpGeom::GetInertia))
    .def("GetBoundingSphereRadius", bp::pure_virtual(&vpGeom::GetBoundingSphereRadius))
    .def("GetShape", bp::pure_virtual(&vpGeom::GetShape))
        ;
        // taken from boost.python libs/python/test/shared_ptr.cpp: 
        // This is the ugliness required to register a to-python converter 
        // for shared_ptr<A>. 
        bp::objects::class_value_wrapper< 
            boost::shared_ptr<vpGeom> , 
            bp::objects::make_ptr_instance<vpGeom, 
                bp::objects::pointer_holder<boost::shared_ptr<vpGeom>,vpGeom> > >();

    bp::class_<pyVpBox, boost::shared_ptr<pyVpBox>, bp::bases<vpGeom>, boost::noncopyable >("vpBox") 
    .def("GetInertia", &vpBox::GetInertia, &pyVpBox::default_GetInertia)
    .def("GetBoundingSphereRadius", &vpBox::GetBoundingSphereRadius, &pyVpBox::default_GetBoundingSphereRadius)
    .def("GetShape", &vpBox::GetShape, &pyVpBox::default_GetShape)
    .def("SetSize", &pyVpBox::SetSize_py)
    .def("GetLocalFrame", &pyVpBox::GetLocalFrame_py)
    .def("GetGlobalFrame", &pyVpBox::GetGlobalFrame_py)
    .def("UpdateGlobalFrame", &pyVpBox::UpdateGlobalFrame_py)
    .def("GetHalfSize", &pyVpBox::GetHalfSize_py)
    .def("GetSize", &pyVpBox::GetSize_py)

    .def("GetShapePy", &pyVpBox::GetShape_py)
    .def("GetInertiaPy", &pyVpBox::GetInertia_py)
    ;

    bp::class_<pyVpCapsule, boost::shared_ptr<pyVpCapsule>, bp::bases<vpGeom>, boost::noncopyable >("vpCapsule") 
    .def("GetInertia", &vpCapsule::GetInertia, &pyVpCapsule::default_GetInertia)
    .def("GetBoundingSphereRadius", &vpCapsule::GetBoundingSphereRadius, &pyVpCapsule::default_GetBoundingSphereRadius)
    .def("GetShape", &vpCapsule::GetShape, &pyVpCapsule::default_GetShape)

    .def("GetLocalFrame", &pyVpCapsule::GetLocalFrame_py)
    .def("GetGlobalFrame", &pyVpCapsule::GetGlobalFrame_py)
    .def("UpdateGlobalFrame", &pyVpCapsule::UpdateGlobalFrame_py)

    .def("GetInertiaPy", &pyVpCapsule::GetInertia_py)
    .def("GetShapePy", &pyVpCapsule::GetShape_py)
    .def("SetSize", &vpCapsule::SetSize)
    .def("GetRadius", &vpCapsule::GetRadius)
    .def("GetHeight", &vpCapsule::GetHeight)
    ;

    bp::class_<pyVpSphere, boost::shared_ptr<pyVpSphere>, bp::bases<vpGeom>, boost::noncopyable >("vpSphere") 
    .def("GetInertia", &vpSphere::GetInertia, &pyVpSphere::default_GetInertia)
    .def("GetBoundingSphereRadius", &vpSphere::GetBoundingSphereRadius, &pyVpSphere::default_GetBoundingSphereRadius)
    .def("GetShape", &vpSphere::GetShape, &pyVpSphere::default_GetShape)

    .def("GetLocalFrame", &pyVpSphere::GetLocalFrame_py)
    .def("GetGlobalFrame", &pyVpSphere::GetGlobalFrame_py)
    .def("UpdateGlobalFrame", &pyVpSphere::UpdateGlobalFrame_py)

    .def("GetInertiaPy", &pyVpSphere::GetInertia_py)
    .def("GetShapePy", &pyVpSphere::GetShape_py)
    .def("SetRadius", &vpSphere::SetRadius)
    .def("GetRadius", &vpSphere::GetRadius)
    ;
}

void pyVpBox::SetSize_py(object &pyV)
{
    Vec3 V = pyVec3_2_Vec3(pyV);
    SetSize(V);
}

bp::list pyVpBox::GetShape_py()
{
    char type;
    scalar data[3];
    GetShape(&type, data);
    object pyV = make_tuple(data[0], data[1], data[2]);
    bp::list ls;
    ls.append(std::string(&type));
    ls.append(pyV);
    // return make_tuple(std::string(&type), data[0], data[1], data[2]);
    return ls;
}

object pyVpBox::GetLocalFrame_py(void)
{
    SE3 T = GetLocalFrame();
    object pyT;
    SE3_2_pySE3(T, pyT);
    return pyT;
}

object pyVpBox::GetGlobalFrame_py(void)
{
    SE3 T = GetGlobalFrame();
    object pyT;
    SE3_2_pySE3(T, pyT);
    return pyT;
}

void pyVpBox::UpdateGlobalFrame_py(void)
{
    UpdateGlobalFrame();
}

object pyVpBox::GetHalfSize_py(void)
{
    Vec3 V = GetHalfSize();
    object pyV;
    Vec3_2_pyVec3(V, pyV);
    return pyV;
}

object pyVpBox::GetSize_py(void)
{
    Vec3 V = GetSize();
    object pyV;
    Vec3_2_pyVec3(V, pyV);
    return pyV;
}

object pyVpBox::GetInertia_py(scalar d)
{
    Inertia I = GetInertia(d);
    object pyI;
    make_pyInertia(pyI);
    for(int i=0; i<10; i++)
        pyI[i] = I[i];
    return pyI;
}

object pyVpCapsule::GetLocalFrame_py(void)
{
    SE3 T = GetLocalFrame();
    object pyT;
    SE3_2_pySE3(T, pyT);
    return pyT;
}

object pyVpCapsule::GetGlobalFrame_py(void)
{
    SE3 T = GetGlobalFrame();
    object pyT;
    SE3_2_pySE3(T, pyT);
    return pyT;
}

void pyVpCapsule::UpdateGlobalFrame_py(void)
{
    UpdateGlobalFrame();
}
bp::list pyVpCapsule::GetShape_py(void)
{
    char type;
    scalar data[2];
    GetShape(&type, data);
    object pyV = make_tuple(data[0], data[1]);
    bp::list ls;
    ls.append(std::string(&type));
    ls.append(pyV);
    // return make_tuple(std::string(&type), data[0], data[1], data[2]);
    return ls;
}

object pyVpCapsule::GetInertia_py(scalar d)
{
    Inertia I = GetInertia(d);
    object pyI;
    make_pyInertia(pyI);
    for(int i=0; i<10; i++)
        pyI[i] = I[i];
    return pyI;
}

object pyVpSphere::GetLocalFrame_py(void)
{
    SE3 T = GetLocalFrame();
    object pyT;
    SE3_2_pySE3(T, pyT);
    return pyT;
}

object pyVpSphere::GetGlobalFrame_py(void)
{
    SE3 T = GetGlobalFrame();
    object pyT;
    SE3_2_pySE3(T, pyT);
    return pyT;
}

void pyVpSphere::UpdateGlobalFrame_py(void)
{
    UpdateGlobalFrame();
}
bp::list pyVpSphere::GetShape_py(void)
{
    char type;
    scalar data[1];
    GetShape(&type, data);
    object pyV = make_tuple(data[0]);
    bp::list ls;
    ls.append(std::string(&type));
    ls.append(pyV);
    // return make_tuple(std::string(&type), data[0], data[1], data[2]);
    return ls;
}

object pyVpSphere::GetInertia_py(scalar d)
{
    Inertia I = GetInertia(d);
    object pyI;
    make_pyInertia(pyI);
    for(int i=0; i<10; i++)
        pyI[i] = I[i];
    return pyI;
}
