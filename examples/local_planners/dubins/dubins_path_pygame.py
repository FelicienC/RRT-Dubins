"""
Demo of the Dubins path planner using Pygame.
Creating the Dubins path going from the center to your mouse.
"""

import pygame
import numpy as np
from rrt import Dubins

# We initialize the planner with the turn radius and the desired distance between consecutive points
local_planner = Dubins(radius=20, point_separation=10)

# Pygame parameters
WIDHT = 800
HEIGHT = 600

# We generate two points, x, y, psi
start = (WIDHT / 2, HEIGHT / 2, 0)
end_angle = np.random.rand() * np.pi * 2

pygame.init()
screen = pygame.display.set_mode((WIDHT, HEIGHT))
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEWHEEL:
            end_angle += np.pi / 50 * np.sign(event.y)

    screen.fill((255, 255, 255))
    x, y = pygame.mouse.get_pos()
    path = local_planner.get_path(start, (x, y, end_angle))
    for point in path:
        # drawing arrows with the heading of each point (point[2]):
        pygame.draw.line(
            screen,
            (0, 0, 0),
            (point[0], point[1]),
            (
                point[0] - 10 * np.cos(point[2]),
                point[1] - 10 * np.sin(point[2]),
            ),
        )
        pygame.draw.line(
            screen,
            (0, 0, 0),
            (point[0], point[1]),
            (
                point[0] - 5 * np.cos(point[2] + np.pi / 4),
                point[1] - 5 * np.sin(point[2] + np.pi / 4),
            ),
        )
        pygame.draw.line(
            screen,
            (0, 0, 0),
            (point[0], point[1]),
            (
                point[0] - 5 * np.cos(point[2] - np.pi / 4),
                point[1] - 5 * np.sin(point[2] - np.pi / 4),
            ),
        )
    pygame.display.flip()
