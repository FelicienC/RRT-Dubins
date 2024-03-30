from rrt import RRT, EmptyEnvironment, DefaultPlanner
import numpy as np
import time

np.random.seed(0)


def test_rrt():
    env = EmptyEnvironment((100, 100))
    local_planner = DefaultPlanner(1)
    my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

    start = env.random_free_space()
    end = env.random_free_space()

    # We initialize an empty tree
    my_rrt.set_start(start)

    # We run 100 iterations of growth
    path = my_rrt.find_path(end, 3, metric="euclidean", goal_rate=0)

    assert len(my_rrt.nodes) == 4
    assert list(my_rrt.nodes.keys()) == [
        (54.88135039273247, 71.51893663724195),
        (55.21145802594044, 70.57499333889304),
        (55.998752004700016, 69.95841553276561),
        (55.768716576583394, 68.98523327757843),
    ]


def test_rrt_timeit():
    env = EmptyEnvironment((100, 100))
    local_planner = DefaultPlanner(1)
    my_rrt = RRT(environment=env, local_planner=local_planner, precision=(1, 1))

    start = env.random_free_space()
    end = env.random_free_space()

    # We initialize an empty tree
    my_rrt.set_start(start)

    # We time the growth of the tree
    start_time = time.time()
    path = my_rrt.find_path(
        end, 3000, metric="euclidean", goal_rate=0, interupt_when_reached=False
    )
    end_time = time.time()
    print("Execution time:", end_time - start_time)
