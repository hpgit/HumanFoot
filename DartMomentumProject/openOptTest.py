from openopt import LCP as openLCP
from fltk import *
import copy
import numpy as np
import time
import numpy.linalg as npl

def main():
    A = np.array(((1., 0.), (0., 1.)))
    x = np.array((0., 0.))
    b = np.array((30., -1.))
    lo = np.array((0., 0.))
    hi = np.array((100000000000., 100000000000.))

    p = openLCP(A, b)
    r = p.solve('lcpsolve')
    f_opt, x_opt = r.ff, r.xf
    w, x = x_opt[x_opt.size/2:], x_opt[:x_opt.size/2]

    print x, np.dot(w, x)

main()
