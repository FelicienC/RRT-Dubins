"""
Demo of the RRT using Pygame.
"""

import pygame
from rrt import EmptyEnvironment, RRT
import numpy as np


class MyPlanner:
    def get_next_state(self, state1, state2) -> list[np.ndarray]:
        """
        Given two states, it returns the path [state1, stateX].

        stateX is the point obtained by starting at state1 and going towards state2
        following the dynamic constraints of the object we simulate.

        Args:
            state1 (np.ndarray): The initial state in the form [x, y, psi, speed]
            state2 (np.ndarray): The goal state in the form [x, y, psi, speed]

        Returns:
            list[np.ndarray]: The path [state1, stateX]
        """

        x, y, psi, speed = state1
        # only 4 actions, turn left, turn right, speed up, slow down
        new_x, new_y = x + speed * np.cos(psi), y + speed * np.sin(psi)
        possible_states = (
            (new_x, new_y, (psi + (np.pi / 10)) % (2 * np.pi), speed),
            (new_x, new_y, (psi - (np.pi / 10)) % (2 * np.pi), speed),
            (new_x, new_y, psi, speed - 1),
            (new_x, new_y, psi, speed + 1),
        )

        return [state1, min(possible_states, key=lambda x: np.linalg.norm(x - state2))]


# Pygame parameters
WIDTH, HEIGHT, N_STEPS = 800, 600, 200

# Initialize the planner
env = EmptyEnvironment(
    [(0, WIDTH), (0, HEIGHT), (0, 2 * np.pi), (0, 10)], random_seed=42
)
my_rrt = RRT(environment=env, local_planner=MyPlanner(), precision=(10, 10, 1, 1))

# We generate two random points and initialize the tree
start, end = env.random_free_space(), env.random_free_space()
my_rrt.set_start(start)

# We run N_STEPS iterations of growth
my_rrt.grow(end, N_STEPS, metric="euclidean")

# We plot the rrt using pygame,
pygame.init()
font = pygame.font.Font(None, 16)
screen = pygame.display.set_mode((WIDTH, HEIGHT))
running = True
while running:
    # Using the event system to stop the simulation if the window is closed
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Plotting the environment
    screen.fill((0, 0, 0))
    pygame.draw.circle(screen, (0, 255, 0), end[:2], 2)  # goal

    # Plotting the rrt
    for node in my_rrt.nodes.values():
        for path in node.paths:
            pygame.draw.line(screen, (0, 255, 0), path[0][:2], path[1][:2], 1)

    x, y, psi, speed = my_rrt.nodes[my_rrt.root_index].state
    pygame.draw.circle(screen, (255, 255, 255), (x, y), 2)

    my_rrt.select_deepest_subtree()
    while my_rrt.max_depth - my_rrt.nodes[my_rrt.root_index].depth < 100:
        my_rrt.grow(end, 100)

    pygame.display.flip()
