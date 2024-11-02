"""
RRT using dubins in a empty 2D environment. 
"""

import matplotlib.pyplot as plt
import numpy as np
from rrt import RRT, EmptyEnvironment, DefaultPlanner

N_STEPS = 1000

# Initialize an empty environment and a planner
env = EmptyEnvironment((100, 100))
local_planner = DefaultPlanner(1)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

# We generate two random points
start = env.random_free_space()
end = env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run N_STEPS iterations of growth
my_rrt.grow(end, N_STEPS, metric="euclidean")

# We plot the rrt using matplotlib, all at once
for edge in my_rrt.edges.values():
    plt.plot([x[0] for x in edge.path], [x[1] for x in edge.path], c="grey")
# We plot the path to the goal if it exists
if my_rrt.reached_goal:
    path = my_rrt.get_path_to_node(my_rrt.reached_goal[-1])
    plt.plot([x[0] for x in path], [x[1] for x in path], c="red")
plt.plot(start[0], start[1], "o", c="green")
plt.plot(end[0], end[1], "o", c="blue")
plt.show()
