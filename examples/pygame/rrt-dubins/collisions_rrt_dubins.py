"""
Demo of the RRT with dubin planner using Pygame.

"""

import pygame
import numpy as np
from rrt import StaticEnvironment, RRT, Dubins

# Pygame parameters
WIDTH, HEIGHT, N_STEPS = 800, 600, 1000

# Initialize the planner
env = StaticEnvironment([(0, WIDTH), (0, HEIGHT), (0, 2 * np.pi)], 100, 30)
local_planner = Dubins(radius=10, point_separation=0.5)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 2))

my_rrt.set_start(env.random_free_space())
end = env.random_free_space()
# All the computation is done here
path = my_rrt.grow(goal=end, nb_iteration=N_STEPS, metric="euclidean")
nb_edges_to_plot = 0

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
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
            x, y = pygame.mouse.get_pos()
            end = (x, y, np.random.rand() * np.pi * 2)
            # All the computation is done here
            my_rrt.grow(end, N_STEPS, metric="euclidean")
            nb_edges_to_plot = 0

    # Plot the obstacles
    screen.fill((255, 255, 255))
    for obstacle in env.obstacles:
        pygame.draw.polygon(screen, (0, 0, 0), obstacle.points)

    # Plot the rrt, one edge at a time
    nb_edges_to_plot += 1
    for node in list(my_rrt.nodes.values())[:nb_edges_to_plot]:
        for path in node.paths:
            path_2d = [(x, y) for x, y, _ in path]
            if not path_2d:
                continue
            pygame.draw.lines(screen, (200, 200, 200), False, path_2d, 1)

    # plot the start and end points
    pygame.draw.circle(screen, (0, 255, 0), my_rrt.nodes[0].state[:2], 5)
    pygame.draw.circle(screen, (255, 0, 0), end[:2], 5)
    pygame.display.flip()
