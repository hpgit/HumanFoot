#include "csLCPLemkeSolver.h"
#include "../../../PyCommon/externalLibs/common/boostPythonUtil.h"

BOOST_PYTHON_MODULE(csLCPLemkeSolver)
{
	numeric::array::set_module_and_type("numpy", "ndarray");

	class_<LemkeSolver>("LemkeSolver", init< >())
		.def(init< >())
		.def("solve", &LemkeSolver::solve)
		;
}

int LemkeSolver::solve(int dim, const object& A, const object& b, object& x, const object& lo,const object& hi)
{
	//solve x^T *(Ax+b) = 0 , x>= 0 , Ax+b >= 0
	if ( 0 >= dim )
	{
		return 0;
	}

	btMatrixXu btA;
	btVectorXu btx;
	btVectorXu btb;
	btVectorXu bthi;
	btVectorXu btlo;

	btA.resize(dim, dim);
	btx.resize(dim);
	btb.resize(dim);
	bthi.resize(dim);
	btlo.resize(dim);


	btAlignedObjectArray<int> limitDependency;
	limitDependency.resize(dim);
	
	for(int i=0; i<dim; i++)
	{
		for(int j=0; j<dim;j++)
		{
			btA.setElem(i, j, XD(A[i][j]));
		}
		btx[i]  = XD(x[i]);
		btb[i]  = -XD(b[i]);
		bthi[i] = XD(hi[i]);
		btlo[i] = XD(lo[i]);
		limitDependency[i] = 0;
	}


	//printf("hehe\n");
	//printf("heheheh\n");

	if( true == solveMLCP(btA, btb, btx, btlo, bthi, limitDependency, 0, false) )
	
	{
		//BT_PROFILE("lemke.solve");
		//btLemkeAlgorithm lemke(btA,btb);
		//btx = lemke.solve(m_maxLoops);
		for(int i=0; i<dim; i++)
		{
			x[i] = btx[i];
		}
		return 1;
	}
	//else
	//	printf("solveMLCP failed!\n");
	return 0;
}
