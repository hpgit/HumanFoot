import numpy as np
from numpy import linalg as npl


def swap_cols(arr, frm, to):
    arr[:, [frm, to]] = arr[:, [to, frm]]


def swap_rows(arr, frm, to):
    arr[[frm, to], :] = arr[[to, frm], :]


def findPosMinIdx(v):
    return 1
    pass

# w = Mz + q
def pivot(M, q, zp, wp):

    # arrange z
    M = M[:, [zp]+range(0, zp)+range(zp+1, M.shape[1])]

    # arrange w
    M = M[[wp]+range(0, wp)+range(wp+1, M.shape[1]), :]
    q = q[[wp]+range(0, wp)+range(wp+1, M.shape[1])]

    return M, q


def solveLcpLemke(M, q):
    # step 0.
    if np.min(q) >= 0:
        return np.zeros_like(q)

    dim = np.shape(q)[0]
    c = np.ones(np.shape(q))
    r = np.argmin(np.divide(q, c))
    M1 = np.vstack((M.T, c)).T
    M1, q = pivot(M1, q, r, dim)

    for ii in range(0, 300):
        # step 1.
        m1 = M1[:, dim]
        if np.min(M1[:, dim]) >= 0:
            return np.zeros_like(q)
        s = findPosMinIdx(np.divide(-q, m1))

        # step 2.
        M1, q = pivot(M1, q, s, r)
        if s == 1:
            return q
        else:
            r = s

A = np.array( ((1.,0.), (0., 1.)) )
b = np.array( (30., -1.) )
print solveLcpLemke(None, np.ones(3))
print solveLcpLemke(A, b)
