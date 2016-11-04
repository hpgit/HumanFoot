from PyCommon.modules.pyVirtualPhysics import *
# import PyCommon.modules._pyVirtualPhysics as pyv

world = vpWorld()
body1 = vpBody()
body2 = vpBody()

joint = vpUJoint()


body1.SetFrame(SE3(Vec3(-1., 0., 0.)))
body2.SetFrame(SE3(Vec3(1., 0., 0.)))

body1.SetJoint(joint, SE3(Vec3(1., 0., 0.)))
body2.SetJoint(joint, SE3(Vec3(-1., 0., 0.)))

joint.SetAngle(1, M_RADIAN * 90.)
joint.SetAngle(0, M_RADIAN * -90.)

world.AddBody(body1)


world.Initialize()

print body2.GetFrame().GetPosition()





