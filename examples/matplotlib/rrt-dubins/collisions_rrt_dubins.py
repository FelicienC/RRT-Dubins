"""
RRT using Dubins in a 2D environment with static obstacles.

This example demonstrates the use of Rapidly-exploring Random Tree (RRT) with a Dubins
local planner in a 2D environment with static obstacles. The planner grows the RRT to
find a path from a start state to a goal state while avoiding obstacles.
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from rrt import Dubins, RRT, StaticEnvironment
import numpy as np
from typing import Tuple

# Initialize a static environment with boundaries, obstacles, and a random seed for
# reproducibility
env = StaticEnvironment(
    dimensions=[(-50, 50), (-50, 50), (0, 2 * np.pi)],
    nb_obstacles=100,
    obstacle_size=5,
    random_seed=7,
)

# Create a Dubins local planner with specified radius and point separation
local_planner = Dubins(radius=2, point_separation=0.5)

# Create an RRT planner with the environment and local planner
rrt_planner = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 2))

# Generate two random points in the free space of the environment
start: Tuple[float, float, float] = env.random_free_space()
goal: Tuple[float, float, float] = env.random_free_space()

# Set the start and goal points for the RRT planner
rrt_planner.set_start(start)
rrt_planner.set_goal(goal)

# Grow the RRT for 300 iterations using the Euclidean distance metric
rrt_planner.grow(nb_iteration=300)

# Create a 2D plot to visualize the RRT and obstacles
fig, ax = plt.subplots()

# Plot all the obstacles
for obstacle in env.obstacles:
    ax.add_patch(Polygon(obstacle.points, closed=True, fill=True, color="black"))

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
