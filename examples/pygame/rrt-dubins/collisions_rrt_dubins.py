"""
Demo of the RRT with Dubins planner using Pygame.

This example demonstrates the use of Rapidly-exploring Random Tree (RRT) with a Dubins
local planner in an environment with static obstacles using Pygame for visualization.
The planner grows the RRT to find a path from a start state to a goal state while
considering the Dubins path constraints.
"""

import pygame
import numpy as np
from rrt import StaticEnvironment, RRT, Dubins
from typing import Tuple

# Pygame parameters
WIDTH, HEIGHT, N_STEPS = 800, 600, 1000

# Initialize a static environment with boundaries, obstacles, and a random seed for reproducibility
env = StaticEnvironment(
    [(0, WIDTH), (0, HEIGHT), (0, 2 * np.pi)], 100, 30, random_seed=7
)

# Create a Dubins local planner with specified radius and point separation
local_planner = Dubins(radius=10, point_separation=0.5)

# Create an RRT planner with the environment and local planner
rrt_planner = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 2))

# Generate a random start point in the free space of the environment
start: Tuple[float, float, float] = env.random_free_space()
goal: Tuple[float, float, float] = env.random_free_space()

# Set the start and goal points for the RRT planner
rrt_planner.set_start(start)
rrt_planner.set_goal(goal)

# Grow the RRT for 1000 iterations using the Euclidean distance metric
rrt_planner.grow(nb_iteration=N_STEPS, metric="euclidean")
nb_edges_to_plot = 0

# Initialize Pygame
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
            rrt_planner.set_start(env.random_free_space())
            # Using the mouse to set the objective point
            x, y = pygame.mouse.get_pos()
            goal = (x, y, np.random.rand() * np.pi * 2)
            rrt_planner.set_goal(goal)
            # Grow the RRT for 1000 iterations using the Euclidean distance metric
            rrt_planner.grow(nb_iteration=N_STEPS, metric="euclidean")
            nb_edges_to_plot = 0

    # Plot the obstacles
    screen.fill((255, 255, 255))
    for obstacle in env.obstacles:
        pygame.draw.polygon(screen, (0, 0, 0), obstacle.points)

    # Plot the RRT, one edge at a time
    nb_edges_to_plot += 1
    for node in list(rrt_planner.nodes.values())[:nb_edges_to_plot]:
        for path in node.paths:
            path_2d = [(x, y) for x, y, _ in path]
            if not path_2d:
                continue
            pygame.draw.lines(screen, (200, 200, 200), False, path_2d, 1)

    # Plot the start and end points
    pygame.draw.circle(screen, (0, 255, 0), rrt_planner.nodes[0].state[:2], 5)
    pygame.draw.circle(screen, (255, 0, 0), goal[:2], 5)
    pygame.display.flip()
