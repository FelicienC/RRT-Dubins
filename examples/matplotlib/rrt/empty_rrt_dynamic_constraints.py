"""
RRT using dubins in a empty 2D environment, simulating an object having a limited
acceleration and velocity.

The state of the object is [x, y, vx, vy], where x and y are the position of the object
and vx and vy are the velocity of the object in the x and y axis respectively.

The object can only accelerate by 1 in the x and y axis, and can only have a velocity
between -5 and 5 in both axis.

"""

import matplotlib.pyplot as plt
from rrt import RRT, EmptyEnvironment
import numpy as np


class MyPlanner:
    def get_next_state(self, state1, state2) -> list[np.ndarray]:
        """
        Given two states, it returns the path [state1, stateX].

        stateX is the point obtained by starting at state1 and going towards state2
        following the dynamic constraints of the object we simulate.

        Args:
            state1 (np.ndarray): The initial state in the form [x, y, vx, vy]
            state2 (np.ndarray): The goal state in the form [x, y, vx, vy]

        Returns:
            list[np.ndarray]: The path [state1, stateX]
        """
        x, y, vx, vy = state1
        possible_states = (
            (x + vx, y + vy, vx - 1, vy),
            (x + vx, y + vy, vx + 1, vy),
            (x + vx, y + vy, vx, vy - 1),
            (x + vx, y + vy, vx, vy + 1),
        )

        return (state1, min(possible_states, key=lambda x: np.linalg.norm(x - state2)))


N_STEPS = 10000

env = EmptyEnvironment([(-50, 50), (-50, 50), (-5, 5), (-5, 5)])
my_rrt = RRT(environment=env, local_planner=MyPlanner(), precision=(3, 3, 3, 3))

# We generate two random points
start, end = env.random_free_space(), env.random_free_space()

# We initialize an empty tree
my_rrt.set_start(start)

# We run N_STEPS iterations of growth
my_rrt.grow(end, N_STEPS, metric="euclidean")

# We plot the rrt using matplotlib, all at once
for node in my_rrt.nodes.values():
    for path in node.paths:
        plt.plot([x[0] for x in path], [x[1] for x in path], c="grey")

# We plot the path to the goal if it exists
if my_rrt.reached_goal:
    path = my_rrt.get_path_to_node(my_rrt.reached_goal[-1])
    plt.plot([x[0] for x in path], [x[1] for x in path], c="red")

plt.plot(*start[:2], "o", c="green", label="Start")
plt.plot(*end[:2], "o", c="blue", label="End")
plt.show()
