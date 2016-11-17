from PyCommon.modules.pyVirtualPhysics import *
# import PyCommon.modules._pyVirtualPhysics as pyv

import math

from PyCommon.modules.Math import mmMath as mm
import numpy as np

def test1():
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


    geom1 = vpBox()
    geom2 = vpBox()
    body1.AddGeometry(geom1)
    body2.AddGeometry(geom2)

    print ord(geom1.GetShape()[0])
    print vpBox().GetType()
    print ord(vpPlane().GetShape()[0])


    world.SetGravity(Vec3(0., -10., 0.))
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

    print SE3(Vec3(0., 0., 0.))
    for i in range(90):
        world.StepAhead()

    print body1.GetFrame()
    print body2.GetFrame()


def test2():
    world = vpWorld()
    body1 = vpBody()
    body2 = vpBody()

    joint1 = vpRJoint()
    joint2 = vpRJoint()
    joint1.SetAxis(Vec3(1., 0., 0.))
    joint2.SetAxis(Vec3(0., 1., 0.))

    body1.SetFrame(SE3(Vec3(0., 0., -1.)))
    body2.SetFrame(SE3(Vec3(0., 0., +1.)))

    body1.SetJoint(joint1, SE3(Vec3(0., 0., 1.)))
    body2.SetJoint(joint1, SE3(Vec3(0., 0., -1.)))

    body1.SetJoint(joint2, SE3(Vec3(0., 0., 1.)))
    body2.SetJoint(joint2, SE3(Vec3(0., 0., -1.)))

    joint1.SetAngle(M_RADIAN * 90.)
    joint2.SetAngle(M_RADIAN * 90.)
    geom1 = vpBox()
    geom2 = vpBox()

    body1.AddGeometry(geom1)
    body2.AddGeometry(geom2)
    world.AddBody(body1)

    world.SetGravity(Vec3(0., -10., 0.))
    world.Initialize()


    for i in range(30):
        world.StepAhead()

    theta0 = joint1.GetAngle()
    theta1 = joint2.GetAngle()
    print theta0, theta1
    print body1.GetFrame()
    print body2.GetFrame()


test1()


