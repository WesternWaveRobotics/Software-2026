import sys
import time

import pygame

pygame.init()
pygame.joystick.init()

controller = pygame.joystick.Joystick(0)
# axis 0: left-stick-x
# axis 1: left-stick-y
# axis 2: right-stick-x
# axis 3: right-stick-y
# axis 4: left-trigger
# axis 5: right-trigger
