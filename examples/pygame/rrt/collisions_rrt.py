"""
Demo of the RRT with dubin planner using Pygame.

"""

import pygame
from rrt import StaticEnvironment, RRT

WIDTH, HEIGHT, N_STEPS = 800, 600, 10000

# Initialize the planner
env = StaticEnvironment((WIDTH, HEIGHT), 100)
my_rrt = RRT(environment=env)

my_rrt.set_start(env.random_free_space())
# All the computation is done here
end = env.random_free_space()
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
            end = pygame.mouse.get_pos()
            # All the computation is done here
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

    # plot the start and end points
    pygame.draw.circle(screen, (0, 255, 0), my_rrt.nodes[0].state[:2], 5)
    pygame.draw.circle(screen, (255, 0, 0), end[:2], 5)

    pygame.display.flip()
