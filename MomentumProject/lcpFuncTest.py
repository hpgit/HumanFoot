import numpy as np
from numpy import linalg as npl


def pivot(M, q, p1, p2):
    return M, q
    pass


def solveLcpLemke(M, q):
    # step 0.
    if np.min(q) >= 0:
        return np.zeros_like(q)

    dim = np.shape(q)[0]
    c = np.ones(np.shape(q))
    r = np.argmin(np.divide(q, c))
    M1 = np.hstack((M, c))
    M1, q = pivot(M1, q, r, dim)

    for ii in range(0, 300):
        # step 1.
        m1 = M1[:, dim]
        if np.min(M1[:, dim]) >= 0:
            return np.zeros_like(q)
        s = np.argmin(-q, m1)

        # step 2.
        M1, q = pivot(M1, q, s, r)
        if s == 1:
            return q
        else:
            r = s

print solveLcpLemke(None, np.ones(3))
