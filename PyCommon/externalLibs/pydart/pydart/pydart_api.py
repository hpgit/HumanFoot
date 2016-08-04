# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.8
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.





from sys import version_info
if version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_pydart_api', [dirname(__file__)])
        except ImportError:
            import _pydart_api
            return _pydart_api
        if fp is not None:
            try:
                _mod = imp.load_module('_pydart_api', fp, pathname, description)
            finally:
                fp.close()
            return _mod
    _pydart_api = swig_import_helper()
    del swig_import_helper
else:
    import _pydart_api
del version_info
try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.


def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr_nondynamic(self, class_type, name, static=1):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    if (not static):
        return object.__getattr__(self, name)
    else:
        raise AttributeError(name)

def _swig_getattr(self, class_type, name):
    return _swig_getattr_nondynamic(self, class_type, name, 0)


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except AttributeError:
    class _object:
        pass
    _newclass = 0



def init():
    return _pydart_api.init()
init = _pydart_api.init

def destroy():
    return _pydart_api.destroy()
destroy = _pydart_api.destroy

def createWorld(timestep):
    return _pydart_api.createWorld(timestep)
createWorld = _pydart_api.createWorld

def createWorldFromSkel(path):
    return _pydart_api.createWorldFromSkel(path)
createWorldFromSkel = _pydart_api.createWorldFromSkel

def destroyWorld(wid):
    return _pydart_api.destroyWorld(wid)
destroyWorld = _pydart_api.destroyWorld

def saveWorldToFile(wid, path):
    return _pydart_api.saveWorldToFile(wid, path)
saveWorldToFile = _pydart_api.saveWorldToFile

def addSkeleton(wid, path, frictionCoeff=1.0, traditionalMode=0):
    return _pydart_api.addSkeleton(wid, path, frictionCoeff, traditionalMode)
addSkeleton = _pydart_api.addSkeleton

def numSkeletons(wid):
    return _pydart_api.numSkeletons(wid)
numSkeletons = _pydart_api.numSkeletons

def setSkeletonJointDamping(wid, skid, damping):
    return _pydart_api.setSkeletonJointDamping(wid, skid, damping)
setSkeletonJointDamping = _pydart_api.setSkeletonJointDamping

def setCollisionDetector(wid, detector_type):
    return _pydart_api.setCollisionDetector(wid, detector_type)
setCollisionDetector = _pydart_api.setCollisionDetector

def printCollisionDetector(wid):
    return _pydart_api.printCollisionDetector(wid)
printCollisionDetector = _pydart_api.printCollisionDetector

def resetWorld(wid):
    return _pydart_api.resetWorld(wid)
resetWorld = _pydart_api.resetWorld

def stepWorld(wid):
    return _pydart_api.stepWorld(wid)
stepWorld = _pydart_api.stepWorld

def checkCollisionWorld(wid, _checkAllCollisions):
    return _pydart_api.checkCollisionWorld(wid, _checkAllCollisions)
checkCollisionWorld = _pydart_api.checkCollisionWorld

def render(wid):
    return _pydart_api.render(wid)
render = _pydart_api.render

def renderSkeleton(wid, skid):
    return _pydart_api.renderSkeleton(wid, skid)
renderSkeleton = _pydart_api.renderSkeleton

def renderSkeletonWithColor(wid, skid, r, g, b, a):
    return _pydart_api.renderSkeletonWithColor(wid, skid, r, g, b, a)
renderSkeletonWithColor = _pydart_api.renderSkeletonWithColor

def renderSkeletonMarkers(wid, skid):
    return _pydart_api.renderSkeletonMarkers(wid, skid)
renderSkeletonMarkers = _pydart_api.renderSkeletonMarkers

def getWorldTime(wid):
    return _pydart_api.getWorldTime(wid)
getWorldTime = _pydart_api.getWorldTime

def getWorldTimeStep(wid):
    return _pydart_api.getWorldTimeStep(wid)
getWorldTimeStep = _pydart_api.getWorldTimeStep

def setWorldTimeStep(wid, _timeStep):
    return _pydart_api.setWorldTimeStep(wid, _timeStep)
setWorldTimeStep = _pydart_api.setWorldTimeStep

def getWorldSimFrames(wid):
    return _pydart_api.getWorldSimFrames(wid)
getWorldSimFrames = _pydart_api.getWorldSimFrames

def setWorldSimFrame(wid, playFrame):
    return _pydart_api.setWorldSimFrame(wid, playFrame)
setWorldSimFrame = _pydart_api.setWorldSimFrame

def getWorldNumContacts(wid):
    return _pydart_api.getWorldNumContacts(wid)
getWorldNumContacts = _pydart_api.getWorldNumContacts

def getWorldContacts(wid, outv):
    return _pydart_api.getWorldContacts(wid, outv)
getWorldContacts = _pydart_api.getWorldContacts

def setWorldCollisionPair(wid, skid1, bid1, skid2, bid2, bEnable):
    return _pydart_api.setWorldCollisionPair(wid, skid1, bid1, skid2, bid2, bEnable)
setWorldCollisionPair = _pydart_api.setWorldCollisionPair

def getSkeletonName(wid, skid):
    return _pydart_api.getSkeletonName(wid, skid)
getSkeletonName = _pydart_api.getSkeletonName

def getSkeletonMass(wid, skid):
    return _pydart_api.getSkeletonMass(wid, skid)
getSkeletonMass = _pydart_api.getSkeletonMass

def getSkeletonNumBodies(wid, skid):
    return _pydart_api.getSkeletonNumBodies(wid, skid)
getSkeletonNumBodies = _pydart_api.getSkeletonNumBodies

def getSkeletonNumDofs(wid, skid):
    return _pydart_api.getSkeletonNumDofs(wid, skid)
getSkeletonNumDofs = _pydart_api.getSkeletonNumDofs

def getSkeletonBodyName(wid, skid, bodyid):
    return _pydart_api.getSkeletonBodyName(wid, skid, bodyid)
getSkeletonBodyName = _pydart_api.getSkeletonBodyName

def getSkeletonDofName(wid, skid, dofid):
    return _pydart_api.getSkeletonDofName(wid, skid, dofid)
getSkeletonDofName = _pydart_api.getSkeletonDofName

def getSkeletonMobile(wid, skid):
    return _pydart_api.getSkeletonMobile(wid, skid)
getSkeletonMobile = _pydart_api.getSkeletonMobile

def setSkeletonMobile(wid, skid, mobile):
    return _pydart_api.setSkeletonMobile(wid, skid, mobile)
setSkeletonMobile = _pydart_api.setSkeletonMobile

def setSkeletonSelfCollision(wid, skid, bSelfCollision, bAdjacentBodies):
    return _pydart_api.setSkeletonSelfCollision(wid, skid, bSelfCollision, bAdjacentBodies)
setSkeletonSelfCollision = _pydart_api.setSkeletonSelfCollision

def changeRootJointToTransAndEuler(wid, skid):
    return _pydart_api.changeRootJointToTransAndEuler(wid, skid)
changeRootJointToTransAndEuler = _pydart_api.changeRootJointToTransAndEuler

def getSkeletonPositions(wid, skid, outpose):
    return _pydart_api.getSkeletonPositions(wid, skid, outpose)
getSkeletonPositions = _pydart_api.getSkeletonPositions

def getSkeletonVelocities(wid, skid, outpose):
    return _pydart_api.getSkeletonVelocities(wid, skid, outpose)
getSkeletonVelocities = _pydart_api.getSkeletonVelocities

def getSkeletonPositionDifferences(wid, skid, inpose, inpose2, outpose3):
    return _pydart_api.getSkeletonPositionDifferences(wid, skid, inpose, inpose2, outpose3)
getSkeletonPositionDifferences = _pydart_api.getSkeletonPositionDifferences

def getSkeletonVelocityDifferences(wid, skid, inpose, inpose2, outpose3):
    return _pydart_api.getSkeletonVelocityDifferences(wid, skid, inpose, inpose2, outpose3)
getSkeletonVelocityDifferences = _pydart_api.getSkeletonVelocityDifferences

def getSkeletonMassMatrix(wid, skid, array2):
    return _pydart_api.getSkeletonMassMatrix(wid, skid, array2)
getSkeletonMassMatrix = _pydart_api.getSkeletonMassMatrix

def getSkeletonCoriolisAndGravityForces(wid, skid, outpose):
    return _pydart_api.getSkeletonCoriolisAndGravityForces(wid, skid, outpose)
getSkeletonCoriolisAndGravityForces = _pydart_api.getSkeletonCoriolisAndGravityForces

def getSkeletonConstraintForces(wid, skid, outpose):
    return _pydart_api.getSkeletonConstraintForces(wid, skid, outpose)
getSkeletonConstraintForces = _pydart_api.getSkeletonConstraintForces

def setSkeletonPositions(wid, skid, inpose):
    return _pydart_api.setSkeletonPositions(wid, skid, inpose)
setSkeletonPositions = _pydart_api.setSkeletonPositions

def setSkeletonVelocities(wid, skid, inpose):
    return _pydart_api.setSkeletonVelocities(wid, skid, inpose)
setSkeletonVelocities = _pydart_api.setSkeletonVelocities

def setSkeletonForces(wid, skid, intorque):
    return _pydart_api.setSkeletonForces(wid, skid, intorque)
setSkeletonForces = _pydart_api.setSkeletonForces

def getSkeletonPositionLowerLimit(wid, skid, outpose):
    return _pydart_api.getSkeletonPositionLowerLimit(wid, skid, outpose)
getSkeletonPositionLowerLimit = _pydart_api.getSkeletonPositionLowerLimit

def getSkeletonPositionUpperLimit(wid, skid, outpose):
    return _pydart_api.getSkeletonPositionUpperLimit(wid, skid, outpose)
getSkeletonPositionUpperLimit = _pydart_api.getSkeletonPositionUpperLimit

def getSkeletonForceLowerLimit(wid, skid, outpose):
    return _pydart_api.getSkeletonForceLowerLimit(wid, skid, outpose)
getSkeletonForceLowerLimit = _pydart_api.getSkeletonForceLowerLimit

def getSkeletonForceUpperLimit(wid, skid, outpose):
    return _pydart_api.getSkeletonForceUpperLimit(wid, skid, outpose)
getSkeletonForceUpperLimit = _pydart_api.getSkeletonForceUpperLimit

def getSkeletonWorldCOM(wid, skid):
    return _pydart_api.getSkeletonWorldCOM(wid, skid)
getSkeletonWorldCOM = _pydart_api.getSkeletonWorldCOM

def getSkeletonWorldCOMVelocity(wid, skid):
    return _pydart_api.getSkeletonWorldCOMVelocity(wid, skid)
getSkeletonWorldCOMVelocity = _pydart_api.getSkeletonWorldCOMVelocity

def getBodyNodeMass(wid, skid, bid):
    return _pydart_api.getBodyNodeMass(wid, skid, bid)
getBodyNodeMass = _pydart_api.getBodyNodeMass

def getBodyNodeInertia(wid, skid, bid):
    return _pydart_api.getBodyNodeInertia(wid, skid, bid)
getBodyNodeInertia = _pydart_api.getBodyNodeInertia

def getBodyNodeShapeBoundingBoxDim(wid, skid, bid):
    return _pydart_api.getBodyNodeShapeBoundingBoxDim(wid, skid, bid)
getBodyNodeShapeBoundingBoxDim = _pydart_api.getBodyNodeShapeBoundingBoxDim

def getBodyNodeLocalCOM(wid, skid, bid):
    return _pydart_api.getBodyNodeLocalCOM(wid, skid, bid)
getBodyNodeLocalCOM = _pydart_api.getBodyNodeLocalCOM

def getBodyNodeWorldCOM(wid, skid, bid):
    return _pydart_api.getBodyNodeWorldCOM(wid, skid, bid)
getBodyNodeWorldCOM = _pydart_api.getBodyNodeWorldCOM

def getBodyNodeWorldCOMVelocity(wid, skid, bid):
    return _pydart_api.getBodyNodeWorldCOMVelocity(wid, skid, bid)
getBodyNodeWorldCOMVelocity = _pydart_api.getBodyNodeWorldCOMVelocity

def getBodyNodeWorldCOMAngularVelocity(wid, skid, bid):
    return _pydart_api.getBodyNodeWorldCOMAngularVelocity(wid, skid, bid)
getBodyNodeWorldCOMAngularVelocity = _pydart_api.getBodyNodeWorldCOMAngularVelocity

def getBodyNodeWorldCOMSpatialVelocity(wid, skid, bid):
    return _pydart_api.getBodyNodeWorldCOMSpatialVelocity(wid, skid, bid)
getBodyNodeWorldCOMSpatialVelocity = _pydart_api.getBodyNodeWorldCOMSpatialVelocity

def getBodyNodeWorldCOMSpatialAcceleration(wid, skid, bid):
    return _pydart_api.getBodyNodeWorldCOMSpatialAcceleration(wid, skid, bid)
getBodyNodeWorldCOMSpatialAcceleration = _pydart_api.getBodyNodeWorldCOMSpatialAcceleration

def getBodyNodeLocalCOMSpatialVelocity(wid, skid, bid):
    return _pydart_api.getBodyNodeLocalCOMSpatialVelocity(wid, skid, bid)
getBodyNodeLocalCOMSpatialVelocity = _pydart_api.getBodyNodeLocalCOMSpatialVelocity

def getBodyNodeLocalCOMSpatialAcceleration(wid, skid, bid):
    return _pydart_api.getBodyNodeLocalCOMSpatialAcceleration(wid, skid, bid)
getBodyNodeLocalCOMSpatialAcceleration = _pydart_api.getBodyNodeLocalCOMSpatialAcceleration

def getBodyNodeNumContacts(wid, skid, bid):
    return _pydart_api.getBodyNodeNumContacts(wid, skid, bid)
getBodyNodeNumContacts = _pydart_api.getBodyNodeNumContacts

def getBodyNodeContacts(wid, skid, bid, outv):
    return _pydart_api.getBodyNodeContacts(wid, skid, bid, outv)
getBodyNodeContacts = _pydart_api.getBodyNodeContacts

def getBodyNodeTransformation(wid, skid, bid):
    return _pydart_api.getBodyNodeTransformation(wid, skid, bid)
getBodyNodeTransformation = _pydart_api.getBodyNodeTransformation

def getBodyNodeWorldLinearJacobian(wid, skid, bid, inv3, array2):
    return _pydart_api.getBodyNodeWorldLinearJacobian(wid, skid, bid, inv3, array2)
getBodyNodeWorldLinearJacobian = _pydart_api.getBodyNodeWorldLinearJacobian

def getBodyNodeWorldAngularJacobian(wid, skid, bid, array2):
    return _pydart_api.getBodyNodeWorldAngularJacobian(wid, skid, bid, array2)
getBodyNodeWorldAngularJacobian = _pydart_api.getBodyNodeWorldAngularJacobian

def getBodyNodeWorldJacobian(wid, skid, bid, inv3, array2):
    return _pydart_api.getBodyNodeWorldJacobian(wid, skid, bid, inv3, array2)
getBodyNodeWorldJacobian = _pydart_api.getBodyNodeWorldJacobian

def getBodyNodeWorldJacobianClassicDeriv(wid, skid, bid, inv3, array2):
    return _pydart_api.getBodyNodeWorldJacobianClassicDeriv(wid, skid, bid, inv3, array2)
getBodyNodeWorldJacobianClassicDeriv = _pydart_api.getBodyNodeWorldJacobianClassicDeriv

def getBodyNodeWorldJacobianSpatialDeriv(wid, skid, bid, inv3, array2):
    return _pydart_api.getBodyNodeWorldJacobianSpatialDeriv(wid, skid, bid, inv3, array2)
getBodyNodeWorldJacobianSpatialDeriv = _pydart_api.getBodyNodeWorldJacobianSpatialDeriv

def addBodyNodeExtForce(wid, skid, bid, inv3):
    return _pydart_api.addBodyNodeExtForce(wid, skid, bid, inv3)
addBodyNodeExtForce = _pydart_api.addBodyNodeExtForce

def addBodyNodeExtForceAt(wid, skid, bid, inv3, inv3_2):
    return _pydart_api.addBodyNodeExtForceAt(wid, skid, bid, inv3, inv3_2)
addBodyNodeExtForceAt = _pydart_api.addBodyNodeExtForceAt

def getBodyNodeNumMarkers(wid, skid, bid):
    return _pydart_api.getBodyNodeNumMarkers(wid, skid, bid)
getBodyNodeNumMarkers = _pydart_api.getBodyNodeNumMarkers

def getMarkerLocalPosition(wid, skid, bid, mid):
    return _pydart_api.getMarkerLocalPosition(wid, skid, bid, mid)
getMarkerLocalPosition = _pydart_api.getMarkerLocalPosition

def getMarkerPosition(wid, skid, bid, mid):
    return _pydart_api.getMarkerPosition(wid, skid, bid, mid)
getMarkerPosition = _pydart_api.getMarkerPosition

def readC3D(path, outv):
    return _pydart_api.readC3D(path, outv)
readC3D = _pydart_api.readC3D
# This file is compatible with both classic and new-style classes.


