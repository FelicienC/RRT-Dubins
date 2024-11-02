"""
Demo of the RRT with dubin planner using Pygame.
"""

import pygame
import numpy as np
from rrt import EmptyEnvironment, RRT, DefaultPlanner

# Pygame parameters
WIDHT = 800
HEIGHT = 600
N_STEPS = 10000

# Initialize the planner
env = EmptyEnvironment((WIDHT, HEIGHT))
local_planner = DefaultPlanner(point_separation=5)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

# We generate two random points
start = env.random_free_space()
end = env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run N_STEPS iterations of growth
path = my_rrt.grow(end, N_STEPS, metric="euclidean")

# We plot the rrt using pygame, adding one edge at a time
pygame.init()
screen = pygame.display.set_mode((WIDHT, HEIGHT))
nb_edges_to_plot = 0
running = True
while running:
    # Using the event system to reset the tree
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONUP:
            # Reset the tree
            my_rrt.set_start(env.random_free_space())
            # Using the mouse to set the objective point
            end = pygame.mouse.get_pos()
            # All the computation is done here
            my_rrt.grow(end, N_STEPS, metric="euclidean")
            nb_edges_to_plot = 0

    # Plot the rrt, one edge at a time
    nb_edges_to_plot += 1
    screen.fill((255, 255, 255))
    for edge in list(my_rrt.edges.values())[:nb_edges_to_plot]:
        points = [edge.path[0], edge.path[-1]]
        pygame.draw.lines(screen, (200, 200, 200), False, points)

    # plot the start and end points
    pygame.draw.circle(screen, (0, 255, 0), my_rrt.nodes[0].state[:2], 5)
    pygame.draw.circle(screen, (255, 0, 0), end[:2], 5)

    pygame.display.flip()
