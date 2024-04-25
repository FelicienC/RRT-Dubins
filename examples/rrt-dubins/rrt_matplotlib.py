"""
RRT using dubins in a 2D environment 
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from rrt import Dubins, RRT, StaticEnvironment

# We initialize the planner with the turn radius and the desired distance between consecutive points
env = StaticEnvironment((100, 100, 6.28), 50)
local_planner = Dubins(radius=2, point_separation=0.5)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 2))

start = env.random_free_space()
end = env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run 100 iterations of growth
my_rrt.grow(end, 200)


# We plot
fig, ax = plt.subplots()
for obstacle in env.obstacles:
    ax.add_patch(Polygon(obstacle.points, closed=True, fill=True, color="black"))
for edge in my_rrt.edges.values():
    plt.plot([x[0] for x in edge.path], [x[1] for x in edge.path], c="grey")
plt.plot([x[0] for x in path], [x[1] for x in path], c="red")
plt.plot(start[0], start[1], "o", c="green")
plt.plot(end[0], end[1], "o", c="blue")
plt.show()
