"""
Demo of the RRT using Pygame.

This example demonstrates the use of Rapidly-exploring Random Tree (RRT) in an empty
environment using Pygame for visualization. The planner simulates an object with dynamic
constraints and grows the RRT to find a path from a start state to a goal state.
"""

import pygame
from rrt import EmptyEnvironment, RRT
import numpy as np
from typing import List, Tuple


class MyPlanner:
    def get_next_state(
        self, state1: np.ndarray, state2: np.ndarray
    ) -> List[np.ndarray]:
        """
        Given two states, it returns the path [state1, stateX].

        stateX is the point obtained by starting at state1 and going towards state2
        following the dynamic constraints of the object we simulate.

        Args:
            state1 (np.ndarray): The initial state in the form [x, y, psi]
            state2 (np.ndarray): The goal state in the form [x, y, psi]

        Returns:
            list[np.ndarray]: The path [state1, stateX]
        """
        x, y, psi = state1
        speed = 15  # fixed speed
        new_x, new_y = x + speed * np.cos(psi), y + speed * np.sin(psi)
        possible_states = (
            (new_x, new_y, (psi + (np.pi / 10)) % (2 * np.pi)),  # turn right
            (new_x, new_y, (psi - (np.pi / 10)) % (2 * np.pi)),  # turn left
            (new_x, new_y, psi),  # go straight
        )

        return [state1, min(possible_states, key=lambda s: np.linalg.norm(s - state2))]


# Pygame parameters
WIDTH, HEIGHT, N_STEPS = 800, 600, 200

# Initialize an empty environment with boundaries and a random seed for reproducibility
env = EmptyEnvironment([(0, WIDTH), (0, HEIGHT), (0, 2 * np.pi)], random_seed=42)

# Create an RRT planner with the environment and custom local planner
rrt_planner = RRT(environment=env, local_planner=MyPlanner(), precision=(10, 10, 1))

# Generate two random points in the free space of the environment
start: Tuple[float, float, float] = env.random_free_space()
goal: Tuple[float, float, float] = env.random_free_space()

# Set the start and goal points for the RRT planner
rrt_planner.set_start(start)
rrt_planner.set_goal(goal)

# Grow the RRT for 200 iterations using the Euclidean distance metric
rrt_planner.grow(nb_iteration=N_STEPS, metric="euclidean")

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()  # Add a clock to control the frame rate
running = True

while running:
    # Using the event system to stop the simulation if the window is closed
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Plotting the environment
    screen.fill((0, 0, 0))
    pygame.draw.circle(screen, (0, 255, 0), goal[:2], 2)  # goal

    # Plotting the RRT
    total_nodes = len(rrt_planner.nodes)
    for i, node in enumerate(rrt_planner.nodes.values()):
        # The more depth below a node, the greener the edge
        color = (0, 255 * ((total_nodes - i) / total_nodes) ** 0.5, 0)
        for path in node.paths:
            pygame.draw.line(screen, color, path[0][:2], path[1][:2], 1)

    # Plotting the start point
    x, y, psi = rrt_planner.nodes[rrt_planner.root_index].state
    pygame.draw.circle(screen, (255, 255, 255), (x, y), 2)

    # By selecting the deepest subtree, we advance on the tree, and remove all the
    # states that cannot be reached anymore, on the other subtree.
    rrt_planner.select_deepest_subtree()

    # Grow the RRT further if the largest subtree is less than 10*N_STEPS nodes
    if len(rrt_planner.nodes) < 10 * N_STEPS:
        rrt_planner.grow(N_STEPS)

    pygame.display.flip()
    clock.tick(24)  # Maintain a constant frame rate of 30 FPS
