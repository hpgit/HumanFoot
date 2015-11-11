#include "stdafx.h"
//#include <bullet/BulletDynamics/MLCPSolvers/btPATHSolver.h>
#include "btDantzigSolver.h"

class DantzigSolver : public btDantzigSolver
{
public:
	DantzigSolver()
	{
		//Lemke
		//m_maxValue = 100000;
		//m_debugLevel = 0;
		//m_maxLoops = 1000;
		//m_useLoHighBounds = true;

		//Dantizg
		m_acceptableUpperLimitSolution = btScalar(1000);

		//PATH
		//License_SetString("2069810742&Courtesy_License&&&USR&2013&14_12_2011&1000&PATH&GEN&31_12_2013&0_0_0&0&0_0");
		//e.error_data = 0;
		//e.warning = MyWarning;
		//e.error = MyError;
		//Error_SetInterface(&e);
	}

	int solve(int dim, const object& A, const object& b, object& x, const object& lo,const object& hi);
};