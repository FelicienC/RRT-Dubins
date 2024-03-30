"""
Demo of the RRT with dubin planner using Pygame.
"""

import pygame
import numpy as np
from rrt import Dubins, EmptyEnvironment, RRT


# Pygame parameters
WIDHT = 800
HEIGHT = 600

# Initialize the planner
env = EmptyEnvironment((WIDHT, HEIGHT, 6.28))
local_planner = Dubins(radius=20, point_separation=10)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 2))

pygame.init()
screen = pygame.display.set_mode((WIDHT, HEIGHT))
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONUP:
            x, y = pygame.mouse.get_pos()
            my_rrt.set_start((WIDHT / 2, HEIGHT / 2, 0))
            end = (x, y, np.random.rand() * np.pi * 2)
            path = my_rrt.find_path(end, 100, interupt_when_reached=False)

    # Plot the rrt
    screen.fill((255, 255, 255))
    for edge in my_rrt.edges.values():
        points = [(x, y) for x, y, _ in edge.path]
        pygame.draw.lines(screen, (200, 200, 200), False, points)
    pygame.display.flip()
