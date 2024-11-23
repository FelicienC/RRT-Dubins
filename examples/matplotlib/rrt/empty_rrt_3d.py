"""
RRT using dubins in an empty 3D environment.
"""

import matplotlib.pyplot as plt
from rrt import RRT, EmptyEnvironment

N_STEPS = 1000

# Initialize an empty environment and a planner
env = EmptyEnvironment([(-50, 50), (-50, 50), (-50, 50)], random_seed=7)
my_rrt = RRT(environment=env)

# We generate two random points
start, end = env.random_free_space(), env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run N_STEPS iterations of growth
my_rrt.grow(end, N_STEPS, metric="euclidean")

# We plot the rrt using matplotlib, all at once
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
for node in my_rrt.nodes.values():
    for path in node.paths:
        ax.plot(
            [x[0] for x in path], [x[1] for x in path], [x[2] for x in path], c="grey"
        )
# We plot the path to the goal if it exists
if my_rrt.reached_goal:
    path = my_rrt.get_path_to_node(my_rrt.reached_goal[-1])
    plt.plot([x[0] for x in path], [x[1] for x in path], [x[2] for x in path], c="red")

plt.plot(*start, "o", c="green")
plt.plot(*end, "o", c="blue")
plt.show()
