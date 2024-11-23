"""
RRT in a 2D environment with static obstacles.

This example demonstrates the use of Rapidly-exploring Random Tree (RRT) in a 2D
environment with static obstacles. The planner grows the RRT to find a path from a
start state to a goal state while avoiding obstacles.
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from rrt import RRT, StaticEnvironment
from typing import Tuple

N_STEPS = 1000

# Initialize a static environment with boundaries, obstacles, and a random seed for reproducibility
env = StaticEnvironment([(-50, 50), (-50, 50)], 100, 5, random_seed=7)

# Create an RRT planner with the environment
rrt_planner = RRT(environment=env)

# Generate two random points in the free space of the environment
start: Tuple[float, float] = env.random_free_space()
goal: Tuple[float, float] = env.random_free_space()

# Set the start and goal points for the RRT planner
rrt_planner.set_start(start)
rrt_planner.set_goal(goal)

# Grow the RRT for 1000 iterations using the Euclidean distance metric
rrt_planner.grow(nb_iteration=N_STEPS, metric="euclidean")

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
ax.scatter(*start, c="green", label="Start")
ax.scatter(*goal, c="blue", label="Goal")

# Add a legend and show the plot
ax.legend()
plt.show()
