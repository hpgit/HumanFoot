from fltk import *
import copy
import numpy as np
import time

import numpy.linalg as npl

import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')

import Optimization.csLCPLemkeSolver as LCP

def main():
	lcpSolver = LCP.LemkeSolver()
	A = np.array( ((1.,0.), (0., 1.)) )
	x = np.array( (0., 0.) )
	b = np.array( (30., -1.) )
	lo = np.array( (0., 0.) )
	hi = np.array( (100000000000., 100000000000.) )
	lcpSolver.solve(2, A, -b, x, lo, hi)

	print x, np.dot(A, x) + b

main()