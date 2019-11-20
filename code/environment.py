import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import KDTree
from obstacle import Obstacle

class Environment:
    """
    Class implementing a very simple bounded 2D world, containing polygonal
    obstacles stored in an appropriate data structure for rapid access to close
    obstacles, even with a large amount of them.

    Attributes
    ----------
    dimensions : tuple
        (dim_x, dim_y) The x and y dimension of the rectangular world.
    obstacles : list
        list of obstacles, instances of the obstacle class.

    Methods
    -------
    plot
        Draws the environnement using matplotlib
    is_free
        Returns False if a point is within an obstacle or outside of the
        boundaries of the environnement.
    """

    def __init__(self, dimensions, nb_obstacles):
        self.dimensions = dimensions
        self.obstacles = [Obstacle(dimensions, 0.05*dimensions[0], 5)\
                          for _ in range(nb_obstacles)]
        self.kdtree = KDTree([obs.center for obs in self.obstacles])

    def plot(self):
        """
        Creates a figure and plots the environement
        """
        for obstacle in self.obstacles:
            obstacle.plot()
        plt.gca().set_xlim(0, self.dimensions[0])
        plt.gca().set_ylim(0, self.dimensions[1])

    def is_free(self, x, y):
        """
        Returns False if a point is within an obstacle or outside of the
        boundaries of the environnement.
        """
        if x < 0 or x > self.dimensions[0] \
        or y < 0 or y > self.dimensions[1]:
            return False
        for obstacle in self.obstacles:
            if obstacle.colides(x, y):
                return False
        return True

    def close_obstacles(self, x, y):
        """
        Returns the list of all the obstacles close enough to be considered.

        Note
        ----
        To be sure that this step actually does not remove any obstacle which
        could yield to a collision, the relation between the size of the
        obstacles and the considered radius for search must be verified:
            R_search > R_obs_Max
        Whit R_obs_Max the maximum distance between the center of an obstacle
        and one of its vertices.
        """
        return self.kdtree.query((x, y), 1)

    def random_free_space(self):
        """
        Returns a point in the free space randomly selected
        """
        x = np.random.rand()*self.dimensions[0]
        y = np.random.rand()*self.dimensions[1]
        while not self.is_free(x, y):
            x = np.random.rand()*self.dimensions[0]
            y = np.random.rand()*self.dimensions[1]
        return x, y, np.random.rand()*np.pi*2
    
