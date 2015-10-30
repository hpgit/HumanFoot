#include "stdafx.h"
#include <bullet/BulletDynamics/MLCPSolvers/btLemkeSolver.h>

class LemkeSolver : public btLemkeSolver
{
public:
	LemkeSolver()
	{
		m_maxValue = 100000;
		m_debugLevel = 0;
		m_maxLoops = 1000;
		m_useLoHighBounds = true;
	}

	int solve(int dim, const object& A, const object& b, object& x, const object& lo,const object& hi);
};