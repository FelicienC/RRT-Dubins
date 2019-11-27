"""
Test module, making sure the main functionnalities are always functionning
"""
import unittest

from dubins import Dubins
from environment import StaticEnvironment
from dynamic_environment import DynamicEnvironment
from rrt import RRT

class TestSum(unittest.TestCase):

    def test_dubins(self):
        """
        Tests that the class Dubins initializes
        """

        Dubins(10, 1)
        print('Dubins OK')

    def test_environment(self):
        """
        Tests that the class Environment initializes and plots
        """

        env = StaticEnvironment((100, 100), 200)
        env.plot()
        print('Static Environment OK')

    def test_rrt(self):
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
        rrt.run(end, 200, metric='euclidian')

        # Trying first the distance defined by the local planner
        rrt.set_start(start)
        rrt.run(end, 200, metric='local')

        print('Static RRT OK')

    def test_dynamic_env(self):
        """
        Tests that the RRT works in a dynamic environement
        """

        env = DynamicEnvironment((100, 100), 5)
        rrt = RRT(env)
        start = (50, 1, 1.57)
        end = (50, 99, 1.57)

        # Initialisation of the tree, to have a first edge
        rrt.set_start(start)
        rrt.run(end, 200, metric='local')

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
            end = (50, position[1]+90, 1.57)
            # Continue the growth of the tree
            rrt.run(end, 2, metric='local')
            env.plot(display=False)
            rrt.plot(file_name='static'+str(i)+'.png', close=True)
        print('Dynamic RRT OK')

    def test_dynamic_env_moving(self):
        """
        Tests that the RRT works with moving obstacles
        """

        env = DynamicEnvironment((100, 100), 5, moving=True)
        rrt = RRT(env)
        start = (50, 1, 1.57)
        end = (50, 99, 1.57)

        # Initialisation of the tree, to have a first edge
        rrt.set_start(start)
        rrt.run(end, 200, metric='local')

        # Initialisation of the position of the vehicle
        position = start[:2]
        current_edge = rrt.select_best_edge()

        time = 0
        for i in range(60):
            time += 1
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
            end = (50, position[1]+90, 1.57)
            # Continue the growth of the tree
            rrt.run(end, 2, metric='local')
            env.plot(time, display=False)
            rrt.plot(file_name='moving'+str(i)+'.png', close=True)
        print('Dynamic RRT with moving obstacles OK')

if __name__ == '__main__':
    unittest.main()
