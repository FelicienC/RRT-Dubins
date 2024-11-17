"""
RRT using dubins in a empty 2D environment.
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from rrt import RRT, StaticEnvironment, DefaultPlanner

N_STEPS = 1000

env = StaticEnvironment((100, 100), 100)
local_planner = DefaultPlanner(0.1)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

start = env.random_free_space()
end = env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run 100 iterations of growth
my_rrt.grow(end, N_STEPS, metric="euclidean")

# We plot the obstacles
fig, ax = plt.subplots()
for obstacle in env.obstacles:
    ax.add_patch(Polygon(obstacle.points, closed=True, fill=True, color="black"))
# We plot the rrt
for edge in my_rrt.edges.values():
    plt.plot([x[0] for x in edge.path], [x[1] for x in edge.path], c="grey")
# We plot the path to the goal if it exists
if my_rrt.reached_goal:
    path = my_rrt.get_path_to_node(my_rrt.reached_goal[-1])
    plt.plot([x[0] for x in path], [x[1] for x in path], c="red")

plt.plot(*start, "o", c="green")
plt.plot(*end, "o", c="blue")
plt.show()
