"""
Demo of the RRT with dubin planner using Pygame.
"""

import pygame
import numpy as np
from rrt import StaticEnvironment, RRT, DefaultPlanner

# Pygame parameters
WIDHT = 800
HEIGHT = 600
N_STEPS = 10000

# Initialize the planner
env = StaticEnvironment((WIDHT, HEIGHT), 100)
local_planner = DefaultPlanner(point_separation=5)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

my_rrt.set_start(env.random_free_space())
path = my_rrt.grow((WIDHT, HEIGHT), N_STEPS, metric="euclidean")
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
            my_rrt.grow(end, N_STEPS, metric="euclidean")
            nb_edges_to_plot = 0

    # Plot the obstacles
    screen.fill((255, 255, 255))
    for obstacle in env.obstacles:
        pygame.draw.polygon(screen, (0, 0, 0), obstacle.points)

    # Plot the rrt, one edge at a time
    nb_edges_to_plot += 1
    for edge in list(my_rrt.edges.values())[:nb_edges_to_plot]:
        points = [edge.path[0], edge.path[-1]]
        pygame.draw.lines(screen, (200, 200, 200), False, points)
    pygame.display.flip()
