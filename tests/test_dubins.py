"""
Test module, making sure the main functionnalities are always functionning
"""

import pytest

from rrt.dubins import Dubins
from rrt.environment import StaticEnvironment
from rrt.dynamic_environment import DynamicEnvironment
from rrt.rrt import RRT


def test_dubins():
    """
    Tests that the class Dubins initializes
    """

    Dubins(10, 1)


def test_environment():
    """
    Tests that the class Environment initializes and plots
    """

    env = StaticEnvironment((100, 100), 200)
    env.plot()


def test_rrt():
    """
    Tests that the RRT class works
    """

    env = StaticEnvironment((100, 100), 100)
    rrt = RRT(env)

    # Selection of random starting and ending points
    start = env.random_free_space()
    end = env.random_free_space()

    # Trying first the euclidian distance
    rrt.set_start(start)
    rrt.run(end, 200, metric="euclidian")

    # Trying first the distance defined by the local planner
    rrt.set_start(start)
    rrt.run(end, 200, metric="local")


def test_dynamic_env():
    """
    Tests that the RRT works in a dynamic environement
    """

    env = DynamicEnvironment((100, 100), 5)
    rrt = RRT(env)
    start = (50, 1, 1.57)
    end = (50, 99, 1.57)

    # Initialisation of the tree, to have a first edge
    rrt.set_start(start)
    rrt.run(end, 200, metric="local")

    # Initialisation of the position of the vehicle
    position = start[:2]
    current_edge = rrt.select_best_edge()

    for i in range(60):
        # We check if we are on an edge or if we have to choose a new edge
        if not current_edge.path:
            current_edge = rrt.select_best_edge()
        # Update the position of the vehicle
        position = current_edge.path.popleft()
        # Update the environment
        #   The frontiers of the sampling and the obstacles
        env.update(position)
        #   The position of the goal
        end = (50, position[1] + 90, 1.57)
        # Continue the growth of the tree
        rrt.run(end, 2, metric="local")


def test_dynamic_env_moving():
    """
    Tests that the RRT works with moving obstacles
    """

    env = DynamicEnvironment((100, 100), 5, moving=True)
    rrt = RRT(env)
    start = (50, 1, 1.57)
    end = (50, 99, 1.57)

    # Initialisation of the tree, to have a first edge
    rrt.set_start(start)
    rrt.run(end, 200, metric="local")

    # Initialisation of the position of the vehicle
    position = start[:2]
    current_edge = rrt.select_best_edge()

    for time in range(60):
        # We check if we are on an edge or if we have to choose a new edge
        if not current_edge.path:
            time = rrt.nodes[current_edge.node_to].time
            current_edge = rrt.select_best_edge()
        # Update the position of the vehicle
        position = current_edge.path.popleft()
        # Update the environment
        #   The frontiers of the sampling and the obstacles
        env.update(position)
        #   The position of the goal
        end = (50, position[1] + 90, 1.57)
        # Continue the growth of the tree
        rrt.run(end, 2, metric="local")
