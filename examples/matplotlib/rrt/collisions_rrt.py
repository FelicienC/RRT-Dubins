"""
RRT using dubins in a empty 2D environment.
"""

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from rrt import RRT, StaticEnvironment

N_STEPS = 1000

env = StaticEnvironment([(-50, 50), (-50, 50)], 100, 5, random_seed=7)
my_rrt = RRT(environment=env)

start, end = env.random_free_space(), env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)
my_rrt.set_goal(end)

# We run 100 iterations of growth
my_rrt.grow(nb_iteration=N_STEPS, metric="euclidean")

# We plot the obstacles
fig, ax = plt.subplots()
for obstacle in env.obstacles:
    ax.add_patch(Polygon(obstacle.points, closed=True, fill=True, color="black"))
# We plot the rrt
for node in my_rrt.nodes.values():
    for path in node.paths:
        ax.plot([x[0] for x in path], [x[1] for x in path], c="grey")
# We plot the path to the goal if it exists
if my_rrt.reached_goal:
    path = my_rrt.get_path_to_node(my_rrt.reached_goal[-1])
    plt.plot([x[0] for x in path], [x[1] for x in path], c="red")

plt.plot(*start, "o", c="green")
plt.plot(*end, "o", c="blue")
plt.show()
