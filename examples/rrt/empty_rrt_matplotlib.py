"""
RRT using dubins in a empty 2D environment
"""

import matplotlib.pyplot as plt
import numpy as np
from rrt import RRT, EmptyEnvironment, DefaultPlanner


env = EmptyEnvironment((100, 100))
local_planner = MyPlanner(1)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

start = env.random_free_space()
end = env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run 100 iterations of growth
path = my_rrt.find_path(end, 1000, interupt_when_reached=False, metric="euclidean")

# We plot the rrt
for edge in my_rrt.edges.values():
    plt.plot([x[0] for x in edge.path], [x[1] for x in edge.path], c="grey")
plt.plot(start[0], start[1], "o", c="green")
plt.plot(end[0], end[1], "o", c="blue")
plt.show()
