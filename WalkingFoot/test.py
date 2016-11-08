from PyCommon.modules.pyVirtualPhysics import *
# import PyCommon.modules._pyVirtualPhysics as pyv

import math

from PyCommon.modules.Math import mmMath as mm
import numpy as np

world = vpWorld()
body1 = vpBody()
body2 = vpBody()

joint = vpUJoint()

joint.SetAxis(0, Vec3(1., 0., 0.))
joint.SetAxis(1, Vec3(0., 1., 0.))

body1.SetFrame(SE3(Vec3(0., 0., -1.)))
body2.SetFrame(SE3(Vec3(0., 0., +1.)))

body1.SetJoint(joint, SE3(Vec3(0., 0., 1.)))
body2.SetJoint(joint, SE3(Vec3(0., 0., -1.)))

joint.SetAngle(0, M_RADIAN * 90.)
joint.SetAngle(1, M_RADIAN * 90.)

world.AddBody(body1)


world.Initialize()

# print body1.GetFrame().GetPosition()
print body2.GetFrame().GetPosition()


theta0 = joint.GetAngle(0)
theta1 = joint.GetAngle(1)

axis0 = joint.GetAxis(0)
axis1 = joint.GetAxis(1)

w0 = InvRotate(Exp(Axis(axis1[0], axis1[1], axis1[2]), theta1), Vec3(1., 0., 0.))
print w0

print axis0
