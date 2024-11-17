"""
RRT using dubins in an empty 3D environment.
"""

import matplotlib.pyplot as plt
from rrt import RRT, EmptyEnvironment, DefaultPlanner

N_STEPS = 1000

# Initialize an empty environment and a planner
env = EmptyEnvironment((100, 100, 100))
local_planner = DefaultPlanner(1)
my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1, 1))

# We generate two random points
start = env.random_free_space()
end = env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run N_STEPS iterations of growth
my_rrt.grow(end, N_STEPS, metric="euclidean")

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# We plot the rrt using matplotlib, all at once
for edge in my_rrt.edges.values():
    plt.plot(
        [x[0] for x in edge.path],
        [x[1] for x in edge.path],
        [x[2] for x in edge.path],
        c="grey",
    )
# We plot the path to the goal if it exists
if my_rrt.reached_goal:
    path = my_rrt.get_path_to_node(my_rrt.reached_goal[-1])
    plt.plot([x[0] for x in path], [x[1] for x in path], [x[2] for x in path], c="red")

plt.plot(*start, "o", c="green")
plt.plot(*end, "o", c="blue")
plt.show()
