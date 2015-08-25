import numpy as np

import sys
if '..' not in sys.path:
    sys.path.append('..')
import Math.mmMath as mm

#===============================================================================
# Reference : Appendix A of Momentum Control for Balance, SIGGRAPH 2009
#===============================================================================
def make_TO(masses):
    O = np.zeros((3,3)) 
    TOs = [None]*len(masses)
    for i in range(len(masses)):
        TOs[i] = np.concatenate((mm.I_SO3()*masses[i], O), 1)
    return np.concatenate(TOs, 1)

def _make_Us(masses, positions, CM):
    Us = [None]*len(masses)
    for i in range(len(masses)):
        Us[i] = masses[i] * mm.getCrossMatrixForm(positions[i] - CM) 
#        Us[i] = masses[i] * mm.getCrossMatrixForm(CM - positions[i]) 
    return Us

# pure inertia matrix
# CM : CM or origin about angular momentum
def getPureInertiaMatrix(TO, masses, positions, CM, inertias):
    Us = _make_Us(masses, positions, CM)
    Vs = inertias
    UVs = [None]*len(masses)
    for i in range(len(masses)):
        UVs[i] = np.concatenate((Us[i], Vs[i]), 1)
    return np.concatenate((TO, np.concatenate(UVs, 1)), 0)

def make_dTO(linkNum):
    O = np.zeros((3, linkNum*3))
    return np.concatenate((O, O), 1)

def _make_dUs(masses, velocities, dCM):
    dUs = [None]*len(masses)
    for i in range(len(masses)):
        dUs[i] = masses[i] * mm.getCrossMatrixForm(velocities[i] - dCM) 
#        dUs[i] = masses[i] * mm.getCrossMatrixForm(dCM - velocities[i]) 
    return dUs

def _make_dVs(angVels, inertias):
    dVs = [None]*len(angVels)
    for i in range(len(angVels)):
        dVs[i] = np.dot(mm.getCrossMatrixForm(angVels[i]), inertias[i])
    return dVs

# time derivative of pure inertia matrix
def getPureInertiaMatrixDerivative(dTO, masses, velocities, dCM, angVels, inertias):
    dUs = _make_dUs(masses, velocities, dCM) 
    dVs = _make_dVs(angVels, inertias)
    dUVs = [None]*len(masses)
    for i in range(len(masses)):
        dUVs[i] = np.concatenate((dUs[i], dVs[i]), 1)
    return np.concatenate((dTO, np.concatenate(dUVs, 1)), 0) 

#===============================================================================
# momentum caculation by standard method
#===============================================================================
def getLinearMomentum(masses, velocities):
    L = mm.v3(0.,0.,0.)
    for i in range(len(masses)):
        L += masses[i] * velocities[i]
    return L

def getAngularMomentum(origin, inertias, angVelocities, positions, masses, velocities):
    H = mm.v3(0.,0.,0.)
    for i in range(len(masses)):
        H += np.dot(inertias[i], angVelocities[i]) + np.cross(positions[i]-origin, masses[i]*velocities[i])
    return H

