"""
RRT using Dubins in an empty 2D environment.

This example demonstrates the use of Rapidly-exploring Random Tree (RRT) with a Dubins
local planner in an empty 2D environment. The planner grows the RRT to find a path from
a start state to a goal state.
"""

import matplotlib.pyplot as plt
import numpy as np
from rrt import Dubins, RRT, EmptyEnvironment
from typing import Tuple

# Initialize an empty environment with boundaries and a random seed for reproducibility
env = EmptyEnvironment([(0, 100), (0, 100), (0, 2 * np.pi)], random_seed=7)

# Create a Dubins local planner with specified radius and point separation
local_planner = Dubins(radius=2, point_separation=0.5)

# Create an RRT planner with the environment and local planner
rrt_planner = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 1))

# Generate two random points in the free space of the environment
start: Tuple[float, float, float] = env.random_free_space()
goal: Tuple[float, float, float] = env.random_free_space()

# Set the start and goal points for the RRT planner
rrt_planner.set_start(start)
rrt_planner.set_goal(goal)

# Grow the RRT for 100 iterations using the Euclidean distance metric
rrt_planner.grow(nb_iteration=100)

# Create a 2D plot to visualize the RRT
fig, ax = plt.subplots()

# Plot all the edges in the RRT
for node in rrt_planner.nodes.values():
    for path in node.paths:
        ax.plot([x[0] for x in path], [x[1] for x in path], c="grey")

# If a path to the goal was found, plot it in red
if rrt_planner.reached_goal:
    for goal_index in rrt_planner.reached_goal:
        path = rrt_planner.get_path_to_node(goal_index)
        ax.plot([x[0] for x in path], [x[1] for x in path], c="red")

# Plot the start point in green and the goal point in blue
ax.scatter(*start[:2], c="green", label="Start")
ax.scatter(*goal[:2], c="blue", label="Goal")

# Add a legend and show the plot
ax.legend()
plt.show()
