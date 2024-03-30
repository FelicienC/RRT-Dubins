"""
Demo of the RRT with dubin planner using Pygame.
"""

import pygame
import numpy as np
from rrt import EmptyEnvironment, RRT

# Pygame parameters
WIDHT = 800
HEIGHT = 600
N_STEPS = 2000

# Initialize the planner
env = EmptyEnvironment((WIDHT, HEIGHT))
local_planner = DefaultPlanner(point_separation=10)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

my_rrt.set_start((WIDHT / 2, HEIGHT / 2))
path = my_rrt.find_path(
    (WIDHT, HEIGHT), N_STEPS, interupt_when_reached=False, metric="euclidean"
)
nb_edges_to_plot = 0


pygame.init()
screen = pygame.display.set_mode((WIDHT, HEIGHT))
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONUP:
            x, y = pygame.mouse.get_pos()
            my_rrt.set_start((WIDHT / 2, HEIGHT / 2))
            end = (x, y)
            path = my_rrt.find_path(
                end, N_STEPS, interupt_when_reached=False, metric="euclidean"
            )
            nb_edges_to_plot = 0

    # Plot the rrt, one edge at a time
    nb_edges_to_plot += 1
    screen.fill((255, 255, 255))
    for edge in list(my_rrt.edges.values())[:nb_edges_to_plot]:
        points = [edge.path[0], edge.path[-1]]
        pygame.draw.lines(screen, (200, 200, 200), False, points)
    pygame.display.flip()
