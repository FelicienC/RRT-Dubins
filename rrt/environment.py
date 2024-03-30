"""
The environment with static polygonal obstacles
"""

import numpy as np

from scipy.spatial import KDTree
from rrt.obstacle import Obstacle


class EmptyEnvironment:

    def __init__(self, dimensions: list[int]) -> None:
        self.dimensions = dimensions

    def is_free(self, state: np.ndarray) -> bool:
        return True

    def random_free_space(self) -> np.ndarray:
        """Return a random np.array in the free space"""
        return np.random.rand(len(self.dimensions)) * self.dimensions


class StaticEnvironment:
    """
    Class implementing a very simple bounded 2D world, containing polygonal
    obstacles stored in an appropriate data structure for rapid access to close
    obstacles, even with a large amount of them.

    Attributes
    ----------
    dimensions : tuple
        (dim_x, dim_y) The x and y dimension of the rectangular world.
    obstacles : list
        List of obstacles, instances of the obstacle class.
    kdtree : KDTree
        The binary search tree used to have a rapid access to the obstacles,
        even with a large amount of them.

    Methods
    -------
    is_free
        Returns False if a point is within an obstacle or outside of the
        boundaries of the environnement.
    """

    def __init__(self, dimensions, nb_obstacles):
        self.dimensions = dimensions
        self.obstacles = [
            Obstacle(dimensions, 0.05 * dimensions[0], 5) for _ in range(nb_obstacles)
        ]
        self.kdtree = KDTree([obs.center for obs in self.obstacles])

    def is_free(self, state):
        """
        Returns False if a point is within an obstacle or outside of the
        boundaries of the environnement.
        """
        x, y, psi = state
        if x < 0 or x > self.dimensions[0] or y < 0 or y > self.dimensions[1]:
            return False
        for obstacle in self.close_obstacles(x, y, nb_obstacles=5):
            if obstacle.colides(x, y):
                return False
        return True

    def close_obstacles(self, x, y, nb_obstacles=1):
        """
        Returns the list of all the obstacles close enough to be considered.

        Parameters
        ----------
        x : float
            The x coordinate of the point requested
        y : float
            The y coordinate of the point requested
        nb_obstacles : int
            The number of obstacles to return, has to be less than the total
            number of obstacles of the environment.

        Note
        ----
        To be sure that this step actually does not remove any obstacle which
        could lead to a collision, the relation between the size of the
        obstacles and the considered radius for search must be verified:
            R_search > R_obs_Max
        With R_obs_Max the maximum distance between the center of an obstacle
        and one of its vertices.
        """

        return [
            self.obstacles[index]
            for index in self.kdtree.query((x, y), nb_obstacles)[1]
        ]

    def random_free_space(self):
        """
        Returns a randomly selected point in the free space.
        """
        x = np.random.rand() * self.dimensions[0]
        y = np.random.rand() * self.dimensions[1]
        while not self.is_free((x, y, 0)):
            x = np.random.rand() * self.dimensions[0]
            y = np.random.rand() * self.dimensions[1]
        return x, y, np.random.rand() * np.pi * 2
