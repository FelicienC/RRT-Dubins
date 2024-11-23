"""
RRT in an empty 3D environment.

This example demonstrates the use of Rapidly-exploring Random Tree (RRT) in an empty 3D
environment. The planner grows the RRT to find a path from a start state to a goal
state.
"""

import matplotlib.pyplot as plt
from rrt import RRT, EmptyEnvironment
from typing import Tuple

# Initialize an empty environment with boundaries and a random seed for reproducibility
env = EmptyEnvironment([(-50, 50), (-50, 50), (-50, 50)], random_seed=7)

# Create an RRT planner with the environment
rrt_planner = RRT(environment=env)

# Generate two random points in the free space of the environment
start: Tuple[float, float, float] = env.random_free_space()
goal: Tuple[float, float, float] = env.random_free_space()

# Set the start and goal points for the RRT planner
rrt_planner.set_start(start)
rrt_planner.set_goal(goal)

# Grow the RRT for 1000 iterations using the Euclidean distance metric
rrt_planner.grow(nb_iteration=1000, metric="euclidean")

# Create a 3D plot to visualize the RRT
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot all the edges in the RRT
for node in rrt_planner.nodes.values():
    for path in node.paths:
        ax.plot(
            [x[0] for x in path], [x[1] for x in path], [x[2] for x in path], c="grey"
        )

# If a path to the goal was found, plot it in red
if rrt_planner.reached_goal:
    path = rrt_planner.get_path_to_node(rrt_planner.reached_goal[-1])
    ax.plot([x[0] for x in path], [x[1] for x in path], [x[2] for x in path], c="red")

# Plot the start point in green and the goal point in blue
ax.scatter(*start, c="green", label="Start")
ax.scatter(*goal, c="blue", label="Goal")

# Add a legend and show the plot
ax.legend()
plt.show()
