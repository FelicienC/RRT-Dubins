"""
RRT in an empty 2D environment with dynamic constraints.

This example demonstrates the use of Rapidly-exploring Random Tree (RRT) in an empty 2D
environment. The planner simulates an object with limited acceleration and velocity,
and grows the RRT to find a path from a start state to a goal state.
"""

import matplotlib.pyplot as plt
from rrt import RRT, EmptyEnvironment
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
            state1 (np.ndarray): The initial state in the form [x, y, vx, vy]
            state2 (np.ndarray): The goal state in the form [x, y, vx, vy]

        Returns:
            list[np.ndarray]: The path [state1, stateX]
        """
        x, y, vx, vy = state1
        possible_states = (
            (x + vx, y + vy, vx - 1, vy),
            (x + vx, y + vy, vx + 1, vy),
            (x + vx, y + vy, vx, vy - 1),
            (x + vx, y + vy, vx, vy + 1),
        )

        return [state1, min(possible_states, key=lambda s: np.linalg.norm(s - state2))]


# Initialize an empty environment with boundaries and a random seed for reproducibility
env = EmptyEnvironment([(-50, 50), (-50, 50), (-5, 5), (-5, 5)], random_seed=7)

# Create an RRT planner with the environment and custom local planner
rrt_planner = RRT(environment=env, local_planner=MyPlanner(), precision=(3, 3, 3, 3))

# Generate two random points in the free space of the environment
start: Tuple[float, float, float, float] = env.random_free_space()
goal: Tuple[float, float, float, float] = env.random_free_space()

# Set the start and goal points for the RRT planner
rrt_planner.set_start(start)
rrt_planner.set_goal(goal)

# Grow the RRT for 3000 iterations using the Euclidean distance metric
rrt_planner.grow(nb_iteration=3000, metric="euclidean")

# Create a 2D plot to visualize the RRT
fig, ax = plt.subplots()

# Plot all the edges in the RRT
for node in rrt_planner.nodes.values():
    for path in node.paths:
        ax.plot([x[0] for x in path], [x[1] for x in path], c="grey")

# If a path to the goal was found, plot it in red
if rrt_planner.reached_goal:
    path = rrt_planner.get_path_to_node(rrt_planner.reached_goal[-1])
    ax.plot([x[0] for x in path], [x[1] for x in path], c="red")

# Plot the start point in green and the goal point in blue
ax.scatter(*start[:2], c="green", label="Start")
ax.scatter(*goal[:2], c="blue", label="Goal")

# Add a legend and show the plot
ax.legend()
plt.show()
