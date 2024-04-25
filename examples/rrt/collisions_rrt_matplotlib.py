"""
RRT using dubins in a empty 2D environment
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from rrt import RRT, StaticEnvironment, DefaultPlanner


env = StaticEnvironment((100, 100), 100)
local_planner = DefaultPlanner(0.1)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

start = env.random_free_space()
end = env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run 100 iterations of growth
my_rrt.grow(end, 2000, metric="euclidean")

# We plot the obstacles
fig, ax = plt.subplots()
for obstacle in env.obstacles:
    ax.add_patch(Polygon(obstacle.points, closed=True, fill=True, color="black"))
# We plot the rrt
for edge in my_rrt.edges.values():
    plt.plot([x[0] for x in edge.path], [x[1] for x in edge.path], c="grey")
plt.plot(start[0], start[1], "o", c="green")
plt.plot(end[0], end[1], "o", c="blue")
plt.show()
