"""
RRT using dubins in a empty 2D environment
"""

import matplotlib.pyplot as plt
import numpy as np
from rrt import Dubins, RRT, EmptyEnvironment

env = EmptyEnvironment((100, 100, 6.28))

local_planner = Dubins(radius=2, point_separation=0.5)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 1))
start = env.random_free_space()
end = env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run 100 iterations of growth
path = my_rrt.find_path(end, 100, interupt_when_reached=False)

# We plot the rrt
for edge in my_rrt.edges.values():
    plt.plot([x[0] for x in edge.path], [x[1] for x in edge.path], c="grey")

plt.plot(start[0], start[1], "o", c="green")
plt.plot(end[0], end[1], "o", c="blue")
plt.show()
