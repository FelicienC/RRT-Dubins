"""
Test module, making sure the main functionnalities are always functionning
"""

import numpy as np
from rrt.dubins import Dubins
from rrt.default_planner import DefaultPlanner
from rrt.environment import StaticEnvironment, EmptyEnvironment
from rrt.dynamic_environment import SimpleDynamicEnvironment, DynamicEnvironment
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

    env = SimpleDynamicEnvironment((100, 100, 10))  # x, y, time
    rrt = RRT(env, local_planner=DefaultPlanner(1), precision=(1, 1, 1))
    start = (1, 0, 0)
    end = (50, 50, 10)

    # Initialisation of the tree, to have a first edge
    rrt.set_start(start)
    path = rrt.grow(end, 200, metric="local")
