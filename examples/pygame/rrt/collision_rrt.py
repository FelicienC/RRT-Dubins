"""
Demo of the RRT using Pygame.
"""

import pygame
from rrt import StaticEnvironment, RRT

# Pygame parameters
WIDTH, HEIGHT, N_STEPS = 800, 600, 10

# Initialize the planner
env = StaticEnvironment([(0, WIDTH), (0, HEIGHT)], 100, 30, random_seed=7)
my_rrt = RRT(environment=env)

# We generate two random points and initialize the tree
start, end = env.random_free_space(), env.random_free_space()
my_rrt.set_start(start)

# We run N_STEPS iterations of growth
my_rrt.grow(end, N_STEPS, metric="euclidean")

# We plot the rrt using pygame,
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
    pygame.draw.circle(screen, (0, 255, 0), end[:2], 2)  # goal

    # Plotting the rrt
    for node in my_rrt.nodes.values():
        for path in node.paths:
            pygame.draw.line(screen, (0, 255, 0), path[0], path[1], 1)

    start = my_rrt.nodes[my_rrt.root_index].state
    pygame.draw.circle(screen, (255, 255, 255), start, 2)  # goal

    my_rrt.select_largest_subtree()
    my_rrt.grow(end, N_STEPS)

    pygame.display.flip()
