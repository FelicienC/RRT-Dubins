"""
RRT using dubins in a 2D environment
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from rrt import Dubins, RRT, StaticEnvironment

# We initialize the planner with the turn radius and the desired distance between
# consecutive points
env = StaticEnvironment([(-50, 50), (-50, 50), (0, 6.29)], 100, 5, random_seed=7)
local_planner = Dubins(radius=2, point_separation=0.5)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 2))

start, end = env.random_free_space(), env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run 100 iterations of growth
my_rrt.grow(end, 300)

# We plot
fig, ax = plt.subplots()
for obstacle in env.obstacles:
    ax.add_patch(Polygon(obstacle.points, closed=True, fill=True, color="black"))

for node in my_rrt.nodes.values():
    for path in node.paths:
        plt.plot([x[0] for x in path], [x[1] for x in path], c="grey")

if my_rrt.reached_goal:
    path = my_rrt.get_path_to_node(my_rrt.reached_goal[-1])
    plt.plot([x[0] for x in path], [x[1] for x in path], c="red")

plt.plot(*start[:2], "o", c="green")
plt.plot(*end[:2], "o", c="blue")
plt.show()
