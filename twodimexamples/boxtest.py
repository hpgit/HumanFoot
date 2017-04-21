import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE, K_SPACE)

import Box2D
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody)

import math

PPM = 8.
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480
TARGET_FPS = 40
TIME_STEP = 1.0 / TARGET_FPS

FRAME_PER_TIME_STEP = 25
WORLD_TIME_STEP = 1./(TARGET_FPS * FRAME_PER_TIME_STEP)



screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)

clock = pygame.time.Clock()

_world = world(gravity=(0, -9.8), doSleep=True) # type: Box2D.b2World
# --- pybox2d world setup ---
# Create the world

# And a static body to hold the ground shape
ground_body = _world.CreateStaticBody(position=(0., -1.), shapes=polygonShape(box=(100., 1.)), )

# Create a dynamic body
dynamic_body = _world.CreateDynamicBody(position=(10, 15), angle=0.) # type: Box2D.b2Body

# And add a box fixture onto it (with a nonzero density, so it will move)
box = dynamic_body.CreatePolygonFixture(box=(4, 1), density=1, friction=0.3)

dynamic_body2 = _world.CreateDynamicBody(position=(14, 15), angle=math.radians(90.))
box2 = dynamic_body2.CreatePolygonFixture(box=(5, 1), density=1, friction=0.3)


joint = _world.CreateRevoluteJoint(bodyA=dynamic_body, bodyB=dynamic_body2, anchor=(14, 15), collideConnected=False) # type: Box2D.b2RevoluteJoint
# joint.motorEnabled = True
# joint.motorSpeed = math.radians(30.)

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
            dynamic_body.ApplyTorque(-5000.*(math.radians(45.)-joint.angle) + 500.*joint.speed, True)
            # dynamic_body2.ApplyTorque(-400., True)
            _world.Step(WORLD_TIME_STEP, 1, 1)
        pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()

