"""
RRT using dubins in a empty 2D environment
"""

import matplotlib.pyplot as plt
import numpy as np
from rrt import Dubins, RRT, EmptyEnvironment

N_STEPS = 100

# Initialize an empty environment and a planner
env = EmptyEnvironment([(0, 100), (0, 100), (0, 2 * np.pi)])
local_planner = Dubins(radius=2, point_separation=0.5)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 1))

# We generate two random points
start, end = env.random_free_space(), env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run N_STEPS iterations of growth
my_rrt.grow(end, N_STEPS)

# We plot the rrt
for node in my_rrt.nodes.values():
    for path in node.paths:
        plt.plot([x[0] for x in path], [x[1] for x in path], c="grey")

if my_rrt.reached_goal:
    for goal_index in my_rrt.reached_goal:
        path = my_rrt.get_path_to_node(goal_index)
        plt.plot([x[0] for x in path], [x[1] for x in path], c="red")

plt.plot(*start[:2], "o", c="green")
plt.plot(*end[:2], "o", c="blue")
plt.show()
