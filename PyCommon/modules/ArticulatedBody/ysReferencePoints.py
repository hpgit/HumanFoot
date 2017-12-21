from ..Math import mmMath as mm

def getCM(pos_or_vels, masses, totalMass=None, validIndexes=None):
    if validIndexes==None:
        validIndexes = range(len(pos_or_vels))
    if totalMass==None:
        totalMass = 0.
        for i in range(len(masses)):
            totalMass += masses[i]
        
    CM = mm.Vec3(0.,0.,0.)
    for i in validIndexes:
        CM += (pos_or_vels[i] * masses[i])
    CM /= totalMass
    return CM

def getCP(contactPositions, contactForces, normal=(0,1,0)):
    if(len(contactPositions) == 0): 
        return None
    
    CP = mm.Vec3(0.,0.,0.)
    totalNormalForce = 0.
    
    for i in range(len(contactPositions)):
        CP += (contactPositions[i] * contactForces[i][1])
        totalNormalForce += contactForces[i][1]

    if totalNormalForce <0.000001:
        return None

    CP /= totalNormalForce
    return CP

def getCMP(contactForces, CM):
    CMP = mm.Vec3(0.,0.,0.)
    f = mm.Vec3(0.,0.,0.)
    for i in range(len(contactForces)):
        f[0] = contactForces[i][0]/contactForces[i][1]
        f[2] = contactForces[i][2]/contactForces[i][1]
    CMP[0] = CM[0] - f[0]*CM[1]
    CMP[2] = CM[2] - f[2]*CM[1]
    return CMP

def getSimpleCP(contactPositions):
    if(len(contactPositions) == 0): 
        return None
    
    CP = mm.Vec3(0.,0.,0.)
    
    for i in range(len(contactPositions)):
        CP += (contactPositions[i])
    
    CP /= len(contactPositions)
    return CP
