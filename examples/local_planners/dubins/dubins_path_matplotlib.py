"""
Demo of the Dubins path planner using matplotlib..
"""

import matplotlib.pyplot as plt
import numpy as np
from rrt import Dubins

# We initialize the planner with the turn radius and the desired distance between consecutive points
local_planner = Dubins(radius=2, point_separation=0.5)

# We generate two points, x, y, psi
start = (0, 0, 0)  # heading east
end = (0, 1, 3.141)  # heading west

# We compute the path between them
path = local_planner.get_path(start, end)

# plot arrows with the heading of each point (point[2]):
for point in path:
    plt.arrow(
        point[0],
        point[1],
        0.1 * np.cos(point[2]),
        0.1 * np.sin(point[2]),
        head_width=0.1,
        head_length=0.1,
    )
plt.show()
