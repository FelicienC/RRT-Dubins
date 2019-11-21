from dubins import Dubins
from environment import Environment
from rrt import RRT

import unittest

class TestSum(unittest.TestCase):

    def test_rrt(self):
        """
	    Test that the RRT class works
	    """
        myEnv = Environment((100, 100), 200)
        myRRT = RRT(myEnv)
        start = myEnv.random_free_space()
        end = myEnv.random_free_space()
        myRRT.run(start, end, (5, 5, 1), 100)
    
    def test_dubins(self):
        """
        Test that the class Dubins initializes
        """
        myDub = Dubins(10, 1)
        #self.assertEqual(result, 6)

    def test_environment(self):
        """
        Test that the class Environment initializes and plots
        """
        myEnv = Environment((100, 100), 200)
        myEnv.plot()
        

if __name__ == '__main__':
    unittest.main()
