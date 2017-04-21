import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)

import Box2D
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody)

pygame.display.set_mode((1280, 960), 0, 32)

playing = True
while playing:
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            playing = False
    pygame.display.flip()

pygame.quit()


