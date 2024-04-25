from rrt import RRT, EmptyEnvironment, DefaultPlanner, Dubins
import numpy as np
import time
import pytest

np.random.seed(0)


@pytest.mark.parametrize("n_dim", range(2, 11))
@pytest.mark.parametrize("metric", ["local", "euclidean"])
def test_rrt_nd(n_dim, metric):
    """Test the growth of the tree in n dimensions, n=2 to 9."""
    env = EmptyEnvironment((100,) * n_dim)
    local_planner = DefaultPlanner(1)
    start, end = env.random_free_space(), env.random_free_space()
    my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1,) * n_dim)
    my_rrt.set_start(start)
    path = my_rrt.grow(end, 10, metric=metric, goal_rate=0)
    assert len(my_rrt.nodes[0].state) == n_dim
    assert list(my_rrt.nodes.keys()) == list(range(10))


def test_rrt_timeit():
    """Test with a large number of nodes to see if it is fast enough"""
    env = EmptyEnvironment((100, 100))
    local_planner = DefaultPlanner(1)
    my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

    start = env.random_free_space()
    end = env.random_free_space()
    my_rrt.set_start(start)
    path = my_rrt.grow(end, 10000, metric="euclidean", goal_rate=0)


def test_rrt_dubins():
    """
    Tests that the RRT class works
    """

    env = EmptyEnvironment((100, 100, 6.29))
    local_planner = Dubins(4, 1)
    rrt = RRT(env, local_planner=local_planner, precision=(1, 1, 2))

    # Selection of random starting and ending points
    start = env.random_free_space()
    end = env.random_free_space()

    # Trying first the euclidian distance
    rrt.set_start(start)
    path = rrt.grow(end, 100, metric="euclidian")

    # # Trying then the distance defined by the local planner
    rrt.set_start(start)
    path = rrt.grow(end, 1000, metric="local")
    print(path)
