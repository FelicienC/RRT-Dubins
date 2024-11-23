"""
Environment modules defining spaces with or without static polygonal obstacles.
"""

import numpy as np

from scipy.spatial import KDTree
from rrt.environments.obstacle import Obstacle
from abc import ABC, abstractmethod


class Environment(ABC):
    """
    Abstract base class for environments.

    Methods
    -------
    is_free(state)
        Determine if a given state is free (not occupied or obstructed).
    random_free_space()
        Generate a random free state within the environment.
    """

    @abstractmethod
    def is_free(self, state: np.ndarray) -> bool:
        """
        Determine if the state is free.

        Parameters
        ----------
        state : np.ndarray
            The state to check.

        Returns
        -------
        bool
            True if the state is free, False otherwise.
        """
        pass

    @abstractmethod
    def random_free_space(self) -> np.ndarray:
        """
        Generate a random free state within the environment.

        Returns
        -------
        np.ndarray
            A random free state.
        """
        pass


class EmptyEnvironment(Environment):
    """
    A simple environment without any obstacles.

    Parameters
    ----------
    dimensions : list of tuple
        Boundaries for each dimension in the environment.
    random_seed : int, optional
        Seed for the random number generator.

    Methods
    -------
    is_free(state)
        Check if a given state is within the environment boundaries.
    random_free_space()
        Generate a random state within the free space.
    """

    def __init__(self, dimensions: list[tuple], random_seed: int = None) -> None:
        if random_seed is not None:
            np.random.seed(random_seed)
        self.dimensions = dimensions

    def is_free(self, state: np.ndarray) -> bool:
        """
        Check if the state is within the boundaries of the environment.

        Parameters
        ----------
        state : np.ndarray
            The state to check.

        Returns
        -------
        bool
            True if the state is within the boundaries, False otherwise.
        """
        for (dim_min, dim_max), state_dim in zip(self.dimensions, state):
            if state_dim < dim_min or state_dim > dim_max:
                return False
        return True

    def random_free_space(self) -> np.ndarray:
        """
        Generate a random state within the free space.

        Returns
        -------
        np.ndarray
            A randomly generated state within the environment boundaries.
        """
        return np.array(
            [
                np.random.uniform(dim_min, dim_max)
                for (dim_min, dim_max) in self.dimensions
            ]
        )


class StaticEnvironment(EmptyEnvironment):
    """
    An environment with static polygonal obstacles using a KDTree for efficient queries.

    Parameters
    ----------
    dimensions : list of tuple
        Boundaries for each dimension in the environment.
    nb_obstacles : int
        Number of obstacles to generate.
    obstacle_size : float
        Size of the obstacles.
    random_seed : int, optional
        Seed for the random number generator.

    Attributes
    ----------
    obstacles : list of Obstacle
        List of obstacles in the environment.
    kdtree : KDTree
        KDTree for efficient nearest neighbor queries.

    Methods
    -------
    is_free(state)
        Check if a given state is free (not within any obstacle and within boundaries).
    close_obstacles(x, y, nb_obstacles=1)
        Retrieve obstacles close to a given point.
    random_free_space()
        Generate a random state within the free space.
    """

    def __init__(
        self, dimensions, nb_obstacles, obstacle_size, random_seed=None
    ) -> None:
        if random_seed is not None:
            np.random.seed(random_seed)
        self.dimensions = dimensions
        self.obstacles = [
            Obstacle(dimensions, obstacle_size, 4) for _ in range(nb_obstacles)
        ]
        self.kdtree = KDTree([obs.center[:2] for obs in self.obstacles])

    def is_free(self, state: np.ndarray) -> bool:
        """
        Check if the state is free (not within any obstacle and within boundaries).

        Parameters
        ----------
        state : np.ndarray
            The state to check.

        Returns
        -------
        bool
            True if the state is free, False otherwise.
        """
        if not super().is_free(state):
            return False
        x, y, *_ = state
        for obstacle in self.close_obstacles(x, y, nb_obstacles=5):
            if obstacle.collides(x, y):
                return False
        return True

    def close_obstacles(
        self, x: float, y: float, nb_obstacles: int = 1
    ) -> list[Obstacle]:
        """
        Retrieve a list of obstacles close to the given point.

        Parameters
        ----------
        x : float
            The x-coordinate of the point.
        y : float
            The y-coordinate of the point.
        nb_obstacles : int, optional
            Number of close obstacles to retrieve. Defaults to 1.

        Returns
        -------
        list of Obstacle
            Obstacles close to the specified point.

        Notes
        -----
        To ensure collisions are detected, the search radius must satisfy:
        R_search > R_obs_Max
        where R_obs_Max is the maximum distance from an obstacle's center to any of its
        vertices.
        """
        return [
            self.obstacles[index]
            for index in self.kdtree.query((x, y), nb_obstacles)[1]
        ]

    def random_free_space(self) -> np.ndarray:
        """
        Generate a random state within the free space (not inside any obstacle).

        Returns
        -------
        np.ndarray
            A randomly generated state within the free space.
        """
        state = super().random_free_space()
        while not self.is_free(state):
            state = super().random_free_space()
        return state
