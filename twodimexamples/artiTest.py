import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE, K_SPACE)

import Box2D
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody)

import math
import os

from PyCommon.modules.Simulator import ysPhysConfig as ypc
from PyCommon.modules import pydart2 as pydart
from PyCommon.modules.Simulator import csDartModel as cpm
from PyCommon.modules.Resource import ysMotionLoader as yf

import numpy as np
from PyCommon.modules.Math import mmMath as mm


SEGMENT_FOOT = False

def buildMassMap():
    massMap = {}
    massMap = massMap.fromkeys(['Head', 'Head_Effector', 'Hips',
                                'LeftArm', 'LeftFoot', 'LeftForeArm', 'LeftHand', 'LeftHand_Effector',
                                'LeftLeg', 'LeftShoulder1', 'LeftUpLeg',
                                'RightArm', 'RightFoot', 'RightForeArm', 'RightHand', 'RightHand_Effector',
                                'RightLeg', 'RightShoulder', 'RightUpLeg',
                                'Spine', 'Spine1',
                                'RightFoot_foot_0_0', 'RightFoot_foot_0_1', 'RightFoot_foot_0_1_Effector',
                                'RightFoot_foot_1_0', 'RightFoot_foot_1_1', 'RightFoot_foot_1_1_Effector',
                                'RightFoot_foot_2_0', 'RightFoot_foot_2_1', 'RightFoot_foot_2_1_Effector',
                                'LeftFoot_foot_0_0', 'LeftFoot_foot_0_1', 'LeftFoot_foot_0_1_Effector',
                                'LeftFoot_foot_1_0', 'LeftFoot_foot_1_1', 'LeftFoot_foot_1_1_Effector',
                                'LeftFoot_foot_2_0', 'LeftFoot_foot_2_1', 'LeftFoot_foot_2_1_Effector',
                                ], 0.)

    # torso : 10
    massMap['Hips'] += 2.
    massMap['Spine'] += 8.

    # head : 3
    massMap['Spine1'] += 3.

    # right upper arm : 2
    massMap['RightArm'] += 2.

    # left upper arm : 2
    massMap['LeftArm'] += 2.

    # right lower arm : 1
    massMap['RightForeArm'] = 1.
    #    massMap['RightForeArm'] = 2.

    # left lower arm : 1
    massMap['LeftForeArm'] = 1.
    #    massMap['LeftForeArm'] = 2.

    # right thigh : 7
    massMap['Hips'] += 2.
    massMap['RightUpLeg'] += 5.

    # left thigh : 7
    massMap['Hips'] += 2.
    massMap['LeftUpLeg'] += 5.

    # right shin : 5
    massMap['RightLeg'] += 5.

    # left shin : 5
    massMap['LeftLeg'] += 5.

    # right foot : 4
    massMap['RightFoot'] += 2.
    # massMap['RightFoot'] += .4

    # left foot : 4
    massMap['LeftFoot'] += 2.
    # massMap['LeftFoot'] += .4
    '''
    massMap['RightFoot_foot_0_0'] = .3
    massMap['RightFoot_foot_0_1'] = .3
    massMap['RightFoot_foot_1_0'] = .3
    massMap['RightFoot_foot_1_1'] = .3
    massMap['RightFoot_foot_2_0'] = .3
    massMap['RightFoot_foot_2_1'] = .3
    massMap['LeftFoot_foot_0_0'] = .3
    massMap['LeftFoot_foot_0_1'] = .3
    massMap['LeftFoot_foot_1_0'] = .3
    massMap['LeftFoot_foot_1_1'] = .3
    massMap['LeftFoot_foot_2_0'] = .3
    massMap['LeftFoot_foot_2_1'] = .3
    #'''

    massMap['RightFoot_foot_0_0'] = .1
    massMap['RightFoot_foot_0_1'] = .1
    massMap['RightFoot_foot_0_0_0'] = .1
    massMap['RightFoot_foot_0_1_0'] = .1
    massMap['RightFoot_foot_1_0'] = .1
    massMap['RightFoot_foot_1_1'] = .1
    massMap['RightFoot_foot_1_2'] = .1
    massMap['LeftFoot_foot_0_0'] = .1
    massMap['LeftFoot_foot_0_1'] = .1
    massMap['LeftFoot_foot_0_0_0'] = .1
    massMap['LeftFoot_foot_0_1_0'] = .1
    massMap['LeftFoot_foot_1_0'] = .1
    massMap['LeftFoot_foot_1_1'] = .1
    massMap['LeftFoot_foot_1_2'] = .1

    return massMap


def buildMcfg():
    massMap = buildMassMap()
    mcfg = ypc.ModelConfig()
    mcfg.defaultDensity = 1000.
    mcfg.defaultBoneRatio = .9

    totalMass = 0.
    for name in massMap:
        node = mcfg.addNode(name)
        node.mass = massMap[name]
        # totalMass += node.mass

    node = mcfg.getNode('Hips')
    node.length = .2
    node.width = .25

    node = mcfg.getNode('Spine1')
    node.length = .2
    node.offset = (0,0,0.1)

    node = mcfg.getNode('Spine')
    node.width = .22

    node = mcfg.getNode('RightFoot')
    node.length = .25
    #    node.length = .27
    #    node.offset = (0,0,0.01)
    node.width = .1
    node.geom = 'MyFoot1'

    node = mcfg.getNode('LeftFoot')
    node.length = .25
    #    node.length = .27
    #    node.offset = (0,0,0.01)
    node.width = .1
    node.geom = 'MyFoot1'

    def capsulize(node_name):
        node = mcfg.getNode(node_name)
        node.geom = 'MyFoot4'
        node.width = 0.01
        node.density = 200.
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., math.pi/4., 0.])], ypc.CapsuleMaterial(1000., .02, .2))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., math.pi/4., 0.])], ypc.CapsuleMaterial(1000., .02, .1))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0., 0., 0.])], ypc.CapsuleMaterial(1000., .01, -1))
        # node.addGeom('MyFoot4', None, ypc.CapsuleMaterial(1000., .02, .1))

    # capsulize('RightFoot')
    # capsulize('LeftFoot')

    if SEGMENT_FOOT:
        node = mcfg.getNode('RightFoot')
        node.density = 200.
        node.geom = 'MyFoot5'
        node.width = 0.01
        node.jointType = 'B'

        node = mcfg.getNode('LeftFoot')
        node.density = 200.
        node.geom = 'MyFoot5'
        node.width = 0.01
        node.jointType = 'B'

    # bird foot
    # capsulize('RightFoot_foot_0_0')
    # capsulize('RightFoot_foot_0_1')
    # capsulize('RightFoot_foot_1_0')
    # capsulize('RightFoot_foot_1_1')
    # capsulize('RightFoot_foot_2_0')
    # capsulize('RightFoot_foot_2_1')
    # capsulize('LeftFoot_foot_0_0')
    # capsulize('LeftFoot_foot_0_1')
    # capsulize('LeftFoot_foot_1_0')
    # capsulize('LeftFoot_foot_1_1')
    # capsulize('LeftFoot_foot_2_0')
    # capsulize('LeftFoot_foot_2_1')


    # human foot
    if SEGMENT_FOOT:
        footJointType = 'B'
        capsulDensity = 400.

        capsulize('RightFoot_foot_0_0')
        node = mcfg.getNode('RightFoot_foot_0_0')
        node.addGeom('MyFoot3', [0.02*np.array([-0.3, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])], ypc.CapsuleMaterial(capsulDensity, .01, 0.02*2.5+0.02))
        node.addGeom('MyFoot3', [0.02*np.array([-0.3-1.2, 0., 2.5*0.25]), mm.exp([0., -math.atan2(1.2, 2.5), 0.])], ypc.CapsuleMaterial(capsulDensity, .01, 0.02*2.5+0.02))
        # node.addGeom('MyFoot4', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0., 0., 0.])], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        capsulize('RightFoot_foot_0_0_0')
        node = mcfg.getNode('RightFoot_foot_0_0_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.addGeom('MyFoot4', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.jointType = footJointType

        capsulize('RightFoot_foot_0_1')
        node = mcfg.getNode('RightFoot_foot_0_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.addGeom('MyFoot3', [0.02*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity,.01, -1))
        node.jointType = footJointType

        capsulize('RightFoot_foot_0_1_0')
        node = mcfg.getNode('RightFoot_foot_0_1_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.addGeom('MyFoot4', [0.02*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.jointType = footJointType

        capsulize('RightFoot_foot_1_0')
        node = mcfg.getNode('RightFoot_foot_1_0')
        node.addGeom('MyFoot3', [0.02*np.array([0., 0., .7]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, 0.02*2.0+0.02))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        capsulize('RightFoot_foot_1_1')
        node = mcfg.getNode('RightFoot_foot_1_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.jointType = footJointType

        capsulize('RightFoot_foot_1_2')
        node = mcfg.getNode('RightFoot_foot_1_2')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.jointType = footJointType


        capsulize('LeftFoot_foot_0_0')
        node = mcfg.getNode('LeftFoot_foot_0_0')
        node.addGeom('MyFoot3', [0.02*np.array([0.3, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])], ypc.CapsuleMaterial(capsulDensity, .01, 0.02*2.5+0.02))
        node.addGeom('MyFoot3', [0.02*np.array([0.3+1.2, 0., 2.5*0.25]), mm.exp([0., math.atan2(1.2, 2.5), 0.])], ypc.CapsuleMaterial(capsulDensity, .01, 0.02*2.5+0.02))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_0_0_0')
        node = mcfg.getNode('LeftFoot_foot_0_0_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.addGeom('MyFoot4', [0.02*np.array([1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_0_1')
        node = mcfg.getNode('LeftFoot_foot_0_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.addGeom('MyFoot3', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_0_1_0')
        node = mcfg.getNode('LeftFoot_foot_0_1_0')
        node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.addGeom('MyFoot4', [0.02*np.array([-1.2, 0., 0.]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_1_0')
        node = mcfg.getNode('LeftFoot_foot_1_0')
        node.addGeom('MyFoot3', [0.02*np.array([0., 0., .7]), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, 0.02*2.0+0.02))
        # node.addGeom('MyFoot4', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(1000., .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_1_1')
        node = mcfg.getNode('LeftFoot_foot_1_1')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.jointType = footJointType

        capsulize('LeftFoot_foot_1_2')
        node = mcfg.getNode('LeftFoot_foot_1_2')
        node.addGeom('MyFoot3', [np.array([0.]*3), mm.exp([0.]*3)], ypc.CapsuleMaterial(capsulDensity, .01, -1))
        node.jointType = footJointType


    return mcfg

PPM = 50.
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 240
TARGET_FPS = 40
TIME_STEP = 1.0 / TARGET_FPS

FRAME_PER_TIME_STEP = 25
WORLD_TIME_STEP = 1./(TARGET_FPS * FRAME_PER_TIME_STEP)


pydart.init()

current_path = os.path.dirname(os.path.abspath(__file__))
motionDir = current_path+'/../DartWalkingFoot/ppmotion/'
filename = 'segfoot_wd2_WalkForwardNormal00.bvh'
bvh = yf.readBvhFileAsBvh(motionDir+filename)
motion_ori = bvh.toJointMotion(1., False)

# motion_ori = yf.readBvhFile(motionDir+filename)
frameTime = 1/motion_ori.fps

mcfg = buildMcfg()
c_locking_vel = .05

wcfg = ypc.WorldConfig()
wcfg.planeHeight = 0.
wcfg.useDefaultContactModel = False
wcfg.lockingVel = c_locking_vel
stepsPerFrame = 50
wcfg.timeStep = frameTime/stepsPerFrame

pydart.init()
dartMotionModel = cpm.DartModel(wcfg, motion_ori[0], mcfg) # type: cpm.DartModel

screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)

clock = pygame.time.Clock()

_world = world(gravity=(0, -9.8), doSleep=True) # type: Box2D.b2World
# --- pybox2d world setup ---
# Create the world


# And a static body to hold the ground shape
# ground_body = _world.CreateStaticBody(position=(0., -1.), shapes=polygonShape(box=(100., 1.)), )
ground_body = _world.CreateStaticBody(position=(0., -1.))  # type:Box2D.b2Body
ground_body.CreatePolygonFixture(box=(100., 1.), categoryBits=4, maskBits=2)

'''
# Create a dynamic body
dynamic_body = _world.CreateDynamicBody(position=(10, 15), angle=0.) # type: Box2D.b2Body

# And add a box fixture onto it (with a nonzero density, so it will move)
box = dynamic_body.CreatePolygonFixture(box=(4, 1), density=1, friction=0.3)

dynamic_body2 = _world.CreateDynamicBody(position=(14, 15), angle=math.radians(90.))
box2 = dynamic_body2.CreatePolygonFixture(box=(5, 1), density=1, friction=0.3)


joint = _world.CreateRevoluteJoint(bodyA=dynamic_body, bodyB=dynamic_body2, anchor=(14, 15), collideConnected=False) # type: Box2D.b2RevoluteJoint
# joint.motorEnabled = True
# joint.motorSpeed = math.radians(30.)
'''
bodies = []  # type: list[Box2D.b2Body]
nameToBody = dict()  #type:  dict[str, Box2D.b2Body]
joints = []  # type: list[Box2D.b2RevoluteJoint]
dartMotionModel.translateByOffset(np.array((0., 0.2, 0.)))


for i in range(dartMotionModel.getBodyNum()):

    name = dartMotionModel.getBody(i).name
    position = dartMotionModel.getBodyPositionGlobal(i)
    pos2d = np.array((position[0], position[1]))
    boxsize = dartMotionModel.getBody(i).shapenodes[0].shape.size()
    boxsize2d = np.array((.5*boxsize[1], .5*boxsize[2]))

    transform = dartMotionModel.getBodyOrientationGlobal(i)
    xaxis = dartMotionModel.getBodyOrientationGlobal(i).dot(np.array((1., 0., 0.)))
    yaxis = dartMotionModel.getBodyOrientationGlobal(i).dot(np.array((0., 1., 0.)))
    zaxis = dartMotionModel.getBodyOrientationGlobal(i).dot(np.array((0., 0., 1.)))

    zaxis[2] = 0.

    _angle = math.atan2(zaxis[1], zaxis[0]) - math.radians(90.)

    # bodies.append(_world.CreateDynamicBody(position=pos2d, angle=math.radians(1.)))
    bodies.append(_world.CreateDynamicBody(position=pos2d, angle=_angle))
    nameToBody[name] = bodies[-1]
    geom = bodies[-1].CreatePolygonFixture(box=boxsize2d, density=1., friction=.3, categoryBits=2, maskBits=4)  # type: Box2D.b2Fixture
    geomFilter = geom.filterData  # type: Box2D.b2Filter

for j in range(dartMotionModel.skeleton.num_joints()):
    parentId = dartMotionModel.getJoint(j).parent_body_node_id()
    childId = dartMotionModel.getJoint(j).child_body_node_id()
    position = dartMotionModel.getJointPositionGlobal(j)
    # position = dartMotionModel.getJoint(j).get_world_frame_after_transform()[:3, 3]
    # print dartMotionModel.getJoint(j).get_world_frame_after_transform()[3][:3]
    # print bodies[parentId], bodies[childId]
    joints.append(_world.CreateRevoluteJoint(bodyA=bodies[parentId], bodyB=bodies[childId],
                               anchor=(position[0], position[1]), collideConnected=False))


def getAngleDiff(angle1, angle2):
    '''
    return angle1 - angle2 in -pi ~ pi
    :type angle1: float
    :type angle2: float
    :return: float
    '''
    _angle1 = angle1 - 2.*math.pi * math.floor(.5*angle1/math.pi)
    _angle2 = angle2 - 2.*math.pi * math.floor(.5*angle2/math.pi)


    if _angle1 - _angle2 > math.pi:
        return _angle1 - _angle2 - 2.*math.pi
    elif _angle1 - _angle2 < -math.pi:
        return _angle1 - _angle2 + 2.*math.pi
    return _angle1 - _angle2

def getJointAngles(_joints):
    '''
    :type _joints: list[Box2D.b2RevoluteJoint]
    :return: list[Box2D.b2RevoluteJoint]
    '''
    jointAngles = []
    for j in _joints:
        bodyA = j.bodyA  # type: Box2D.b2Body
        bodyB = j.bodyB  # type: Box2D.b2Body
        jointAngles.append(getAngleDiff(bodyA.angle, bodyB.angle))
    return jointAngles


print getJointAngles(joints)

colors = {    staticBody: (255, 255, 255, 255),    dynamicBody: (127, 127, 127, 255)}


running = True
playing = False
while running:
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            running = False
        elif event.type == KEYDOWN and event.key == K_SPACE:
            playing = not playing

    screen.fill((0, 0, 0, 0))

    for body in _world.bodies:  # or: world.bodies
        # The body gives us the position and angle of its shapes
        for fixture in body.fixtures:
            # The fixture holds information like density and friction,
            # and also the shape.
            shape = fixture.shape

            # Naively assume that this is a polygon shape. (not good normally!)
            # We take the body's transform and multiply it with each
            # vertex, and then convert from meters to pixels with the scale
            # factor.
            vertices = [(body.transform * v) * PPM for v in shape.vertices]
            vertices = [(SCREEN_WIDTH/2 +v[0], -1.*PPM+SCREEN_HEIGHT - v[1]) for v in vertices]

            pygame.draw.polygon(screen, colors[body.type], vertices)

    # dynamic_body.ApplyTorque(-100., True)

    if playing:
        for i in range(FRAME_PER_TIME_STEP):
            # dynamic_body.ApplyTorque(-5000.*(math.radians(45.)-joint.angle) + 500.*joint.speed, True)
            # dynamic_body2.ApplyTorque(-400., True)
            joint = nameToBody['LeftFoot'].joints[0].joint  # type: Box2D.b2RevoluteJoint
            # nameToBody['LeftFoot'].ApplyTorque(10.*(math.radians(45.)-joint.angle) - 2.*joint.speed, True)
            _world.Step(WORLD_TIME_STEP, 1, 1)
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
