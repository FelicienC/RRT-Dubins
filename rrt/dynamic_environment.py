"""
The environement where the obstacles can move
"""

from collections import deque
import numpy as np
import matplotlib.pyplot as plt

from rrt.obstacle import Wall


class DynamicEnvironment:
    """
    Class implementing a simple dynamic bounded 2D world, containing square
    obstacles placed in an ordered manner in order to always permit passage
    from the bottom to the top of the space.

    Attributes
    ----------
    dimensions : tuple
        (dim_x, dim_y) The x and y dimension of the rectangular world.
    obstacles : list
        List of walls, instances of the Wall class.
    moving : bool
        If the position of the holes in the walls is variying over time.
    center : list
        The coordinates of the center of the current scene.

    Methods
    -------
    plot
        Draws the environnement using matplotlib.
    is_free
        Returns False if a point is within an obstacle or outside of the
        boundaries of the environnement. Checks at a specific instant.
    random_free_space
        Selects an element of the free space, and returns it. Does not check at
        a specific instant.
    update
        Moves the center based on the position of the vehicle within the
        environment.
    """

    def __init__(self, dimensions, nb_walls, moving=False):
        self.moving = moving
        self.dimensions = dimensions
        # nb_walls represent the number of walls on the screen
        self.obstacles = deque()
        for i in range(nb_walls * 2):
            self.obstacles.append(
                Wall(
                    dimensions[0],
                    dimensions[1] * (0.1 + i / nb_walls),
                    thickness=5,
                    moving=moving,
                )
            )
        self.center = [dimensions[0] / 2, dimensions[1] / 2]

    def plot(self, time=0, close=False, display=True):
        """
        Creates a figure and plots the environement on it.

        Parameters
        ----------
        time : float
            The instant at which the environment must be drawn.
        close : bool
            If the plot needs to be automaticaly closed after the drawing.
        display : bool
            If the view pops up or not (used when generating many images)
        """
        plt.ion() if display else plt.ioff()
        plt.figure()
        for obstacle in self.obstacles:
            obstacle.plot(time)
        plt.gca().set_xlim(
            self.center[0] - self.dimensions[0] / 2,
            self.center[0] + self.dimensions[0] / 2,
        )
        plt.gca().set_ylim(
            self.center[1] - self.dimensions[1] / 2,
            self.center[1] + self.dimensions[1] / 2,
        )
        if close:
            plt.close()

    def is_free(self, x, y, time=0):
        """
        Returns False if a point is within an obstacle or outside of the
        boundaries of the environnement.
        """

        if (
            x < self.center[0] - self.dimensions[0] / 2
            or x > self.center[0] + self.dimensions[0] / 2
            or y < self.center[1] - self.dimensions[1] / 2
            or y > self.center[1] + self.dimensions[1] / 2
        ):
            return False
        for obstacle in self.obstacles:
            if obstacle.colides(x, y, time):
                return False
        return True

    def random_free_space(self):
        """
        Returns a randomly selected point in the free space.
        """

        x = np.random.rand() * self.dimensions[0]
        y = (np.random.rand() - 0.5) * self.dimensions[1] + self.center[1]
        while not self.is_free(x, y):
            x = np.random.rand() * self.dimensions[0]
            y = (np.random.rand() - 0.5) * self.dimensions[1] + self.center[1]
        return x, y, np.random.rand() * np.pi * 2

    def update(self, position):
        """
        Refreshs the active walls as well as the position of the camera.

        Parameters
        ----------
        position : float
            The position of the vehicle in the environment.
        """

        # every time an obstacles gets out, we add one on the other side
        self.center = [self.center[0], position[1] + 40]
        for wall in self.obstacles:
            if wall.bottom_y + wall.thickness < self.center[1] - self.dimensions[1]:
                self.obstacles.append(
                    Wall(
                        self.dimensions[0],
                        position[1] + 50 + self.dimensions[1],
                        thickness=5,
                        moving=self.moving,
                    )
                )
                self.obstacles.popleft()
                break
