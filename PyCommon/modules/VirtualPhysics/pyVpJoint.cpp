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
    .def("GetShapePy", &pyVpBox::GetShape_py)
    ;
}

void pyVpBox::SetSize_py(object pyV)
{
    Vec3 V = pyVec3_2_Vec3(pyV);
    SetSize(V);
}

object pyVpBox::GetShape_py()
{
    char type;
    scalar data[3];
    GetShape(&type, data);
    return make_tuple(std::string(&type), data[0], data[1], data[2]);
}