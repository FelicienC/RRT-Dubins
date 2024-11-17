"""
Demo of the RRT with dubin planner using Pygame.
"""

import pygame
from rrt import SimpleDynamicEnvironment, RRT
from time import sleep

# Pygame parameters
WIDTH, HEIGHT, N_STEPS = 800, 600, 10000

# Initialize the planner
env = SimpleDynamicEnvironment((WIDTH, HEIGHT, WIDTH))  # x, y, Time
my_rrt = RRT(environment=env)

start = env.random_free_space()
start[2] = 0  # we start at time 0
my_rrt.set_start(start)

end = env.random_free_space()
path = my_rrt.grow(end, N_STEPS, metric="euclidean")

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
running = True
t = 0
while t < WIDTH:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONUP:
            t = 0
            my_rrt.set_start((WIDTH / 2, HEIGHT / 2, 0))
            end = env.random_free_space()
            my_rrt.grow(end, N_STEPS, metric="euclidean")

    # Plot the obstacles
    screen.fill((255, 255, 255))

    for x in range(0, WIDTH, 10):
        for y in range(0, HEIGHT, 10):
            if not env.is_free((x, y, t)):
                pygame.draw.circle(screen, (0, 0, 0), (x, y), 3)

    for edge in list(my_rrt.edges.values())[:t]:
        points = [edge.path[0][:2], edge.path[-1][:2]]
        pygame.draw.lines(screen, (200, 200, 200), False, points)

    pygame.draw.circle(screen, (0, 255, 0), my_rrt.nodes[0].state[:2], 5)
    pygame.draw.circle(screen, (255, 0, 0), end[:2], 5)

    t += 1
    pygame.display.flip()
    sleep(1 / 60)
    if not running:
        break
