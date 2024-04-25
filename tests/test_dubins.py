"""
Test module, making sure the main functionnalities are always functionning
"""

import numpy as np
from rrt.dubins import Dubins
from rrt.environment import StaticEnvironment
from rrt.dynamic_environment import DynamicEnvironment
from rrt.rrt import RRT


def test_dubins():
    """
    Tests that the class Dubins initializes
    """

    Dubins(10, 1)


def test_dynamic_env():
    """
    Tests that the RRT works in a dynamic environement
    """

    env = DynamicEnvironment((100, 100, 6.29), 5)
    local_planner = Dubins(4, 1)
    rrt = RRT(env, local_planner=local_planner, precision=(1, 1, 1))
    start = (50, 1, 1.57, 0)
    end = (50, 99, 1.57, 10)

    # Initialisation of the tree, to have a first edge
    rrt.set_start(start)
    path = rrt.grow(end, 200, metric="local")

    # Initialisation of the position of the vehicle
    position = start[:2]
    current_edge = rrt.selet_largest_subtree()

    for i in range(60):
        # We check if we are on an edge or if we have to choose a new edge
        if not current_edge.path:
            current_edge = rrt.selet_largest_subtree()
        # Update the position of the vehicle
        position = current_edge.path.popleft()
        # Update the environment
        #   The frontiers of the sampling and the obstacles
        env.update(position)
        #   The position of the goal
        end = (50, position[1] + 90, 1.57)
        # Continue the growth of the tree
        rrt.grow(end, 2, metric="local")


def test_dynamic_env_moving():
    """
    Tests that the RRT works with moving obstacles
    """

    env = DynamicEnvironment((100, 100), 5, moving=True)
    local_planner = Dubins(4, 1)
    rrt = RRT(env, local_planner=local_planner, precision=(1, 1, 1))
    start = (50, 1, 1.57)
    end = (50, 99, 1.57)

    # Initialisation of the tree, to have a first edge
    rrt.set_start(start)
    rrt.grow(end, 200, metric="local")

    # Initialisation of the position of the vehicle
    position = start[:2]
    current_edge = rrt.selet_largest_subtree()

    for time in range(60):
        # We check if we are on an edge or if we have to choose a new edge
        if not current_edge.path:
            current_edge = rrt.selet_largest_subtree()
        # Update the position of the vehicle
        position = current_edge.path.popleft()
        # Update the environment
        #   The frontiers of the sampling and the obstacles
        env.update(position)
        #   The position of the goal
        end = (50, position[1] + 90, 1.57)
        # Continue the growth of the tree
        rrt.grow(end, 2, metric="local")
