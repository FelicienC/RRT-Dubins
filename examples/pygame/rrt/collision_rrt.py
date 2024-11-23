"""
Demo of the RRT using Pygame.

This example demonstrates the use of Rapidly-exploring Random Tree (RRT) in an
environment with static obstacles using Pygame for visualization. The planner grows the
RRT to find a path from a start state to a goal state while avoiding obstacles.
"""

import pygame
from rrt import StaticEnvironment, RRT
from typing import Tuple

# Pygame parameters
WIDTH, HEIGHT, N_STEPS = 800, 600, 10

# Initialize a static environment with boundaries, obstacles, and a random seed for reproducibility
env = StaticEnvironment([(0, WIDTH), (0, HEIGHT)], 100, 30, random_seed=7)

# Create an RRT planner with the environment
rrt_planner = RRT(environment=env)

# Generate two random points in the free space of the environment
start: Tuple[float, float] = env.random_free_space()
goal: Tuple[float, float] = env.random_free_space()

# Set the start and goal points for the RRT planner
rrt_planner.set_start(start)
rrt_planner.set_goal(goal)

# Grow the RRT for 10 iterations using the Euclidean distance metric
rrt_planner.grow(nb_iteration=N_STEPS, metric="euclidean")

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
running = True

while running:
    # Using the event system to stop the simulation if the window is closed
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Plotting the environment
    screen.fill((0, 0, 0))
    for obstacle in env.obstacles:
        pygame.draw.polygon(screen, (0, 0, 200), obstacle.points)
    pygame.draw.circle(screen, (0, 255, 0), goal[:2], 2)  # goal

    # Plotting the RRT
    for node in rrt_planner.nodes.values():
        for path in node.paths:
            pygame.draw.line(screen, (0, 255, 0), path[0], path[1], 1)

    # Plotting the start point
    start = rrt_planner.nodes[rrt_planner.root_index].state
    pygame.draw.circle(screen, (255, 255, 255), start, 2)  # start

    # By selecting the largest subtree, we advance on the tree, and remove all the
    # states that cannot be reached anymore, on the other subtree.
    rrt_planner.select_largest_subtree()

    # Grow the RRT further
    rrt_planner.grow(N_STEPS)

    pygame.display.flip()
