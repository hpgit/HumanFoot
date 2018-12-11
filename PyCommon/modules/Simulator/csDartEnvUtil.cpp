#include "stdafx.h"
#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"
namespace bp = boost::python;
namespace np = boost::python::numpy;
using boost::python::numpy::ndarray;

void worlds_step(const bp::list &envs, const bp::list &actions)
{
	int envs_num = len(envs);

    std::cout << "hhh"<< std::endl;
//    #pragma omp parallel for
    for(int i=0; i<envs_num; i++)
    {
    std::cout << "hhh"<< i << std::endl;
        envs[i].attr("step")(actions[i]);
    }
}

BOOST_PYTHON_MODULE(csDartEnvUtil)
{
	def("worlds_step", worlds_step);
}
