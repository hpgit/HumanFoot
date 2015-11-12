import sys
if '../PyCommon/modules' not in sys.path:
    sys.path.append('../PyCommon/modules')

import Optimization.csLCPLemkeSolver as lcp

import Util.ysPythonEx as ype

import numpy as np
import math

def main():
    h = 0.001
    mu = 1.
    m = 5.
    r = 1.
    I = 2./5.*m*r*r
    w = np.matrix((0., 0., 0.)).T

    invM = np.matrix(( 
        (1./m, 0.,0.,0.,0.,0.), 
        (0., 1./m,0.,0.,0.,0.),
        (0.,0., 1./m,0.,0.,0.),
        (0.,0.,0., 1./I,0.,0.),
        (0.,0.,0.,0., 1./I,0.),
        (0.,0.,0.,0.,0., 1./I), 
        ))

    N = np.matrix([[0.], [1.], [0.], [0.], [0.], [0.]])

    D = np.matrix(( 
        (1.,0.,-1.,0. ),
        (0.,0., 0.,0. ),
        (0.,1., 0.,-1.),
        (0.,-r, 0.,r  ),
        (0.,0., 0.,0. ),
        (r ,0.,-r ,0. ),
        ))

    c = m*np.matrix((0.,-9.8, 0., 0., 0., 0.)).T #+ np.matrix(np.hstack((np.zeros(3), np.cross(w.T, I*w.T))).T

    invMc = np.dot(invM, c)
    E = np.matrix((1., 1., 1., 1.)).T
    mus = mu *np.eye(1)

    temp_NM = N.T.dot(invM)
    temp_DM = D.T.dot(invM)

    #A =[ A11,  A12, 0]
    #   [ A21,  A22, E]
    #   [ mus, -E.T, 0]

    A11 = h*temp_NM.dot(N)
    A12 = h*temp_NM.dot(D)
    A21 = h*temp_DM.dot(N)
    A22 = h*temp_DM.dot(D)
    
   
    #print "A11: ", A11
    #print "A12: ", A12
    #print "A21: ", A21
    #print "A22: ", A22
    #print "E: ", E

    A1 = np.hstack((np.hstack((A11, A12)),[[0.]] ))
    A2 = np.hstack((np.hstack((A21, A22)), E ))
    A3 = np.hstack((np.hstack((mus,-E.T)), np.zeros((mus.shape[0], E.shape[1])) ))
    A = np.vstack((np.vstack((A1, A2)), A3))

    #bx= h * (M*qdot_0 + tau - c)
    #b =[N.T * Jc * invM * kx]
    #   [D.T * Jc * invM * kx]
    #   [0]

    qdot_0 = np.matrix((0., -1., 0., 0., 0., 0.)).T

    tau = np.zeros(np.shape(qdot_0))

    b1 = N.T.dot(qdot_0 - h*invMc) + h*temp_NM.dot(tau)
    b2 = D.T.dot(qdot_0 - h*invMc) + h*temp_DM.dot(tau)
    b3 = np.matrix(np.zeros(mus.shape[0]))

    #print "b1: ", b1
    #print "b2: ", b2
    #print "b3: ", b3

    b  = np.vstack((np.vstack((b1, b2)), b3))

    #print "A: ", A
    #print "b: ", b
    
    #lo = np.zeros(A.shape[0])
    lo = 0. * np.ones(A.shape[0])
    hi = 100000. * np.ones(A.shape[0])

    x = 0.*np.ones(A.shape[0])
    lcpSolver = lcp.LemkeSolver()
    #lcpSolver = lcpD.DantzigSolver()
    lcpSolver.solve(A.shape[0], A.A, b.A1, x, lo, hi)
    x = np.matrix(x).T
    print "A: ", A
    print "b: ", b
    print "x: ", x
    print "w: ", np.dot(x.T, np.dot(A,x)+b)

    force = np.matrix(np.zeros(3)).T
    force[1] = x[0]
    force[0] = x[1] - x[3]
    force[2] = x[2] - x[4]
    print force



if __name__ == '__main__':
	main()
