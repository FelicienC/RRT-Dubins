"""
Implementation of the polygonal obstacles as well as of the walls
"""

import numpy as np

from shapely.geometry import Polygon, Point


class Obstacle:
    """
    Class implementing simple 2D polygonal obstacles.

    Attributes
    ----------
    points : list
        List of (x, y) coordinates in the frame of the environnement
        representing the obstacle.
    bounding_box : 4-tuple
        Coordinates of the lower left and upper right corners of the bounding
        box containing the obstacle.
    center : tuple
        Coordinates of the center of the bounding box.
    polygon : shapely.geometry.Polygon
        The polygon representing the obstacle.

    """

    def __init__(self, map_dimensions, size, nb_pts):
        self.center = np.array(
            [np.random.rand() * map_dimensions[0], np.random.rand() * map_dimensions[1]]
        )
        # We use very simple convex polygons, generated with a radius
        # and randomly selected angles.
        angles = sorted((np.random.rand() * 2 * np.pi for _ in range(nb_pts)))
        self.points = np.array(
            [
                self.center + np.array([size * np.cos(angle), size * np.sin(angle)])
                for angle in angles
            ]
        )
        self.bounding_box = (
            min(self.points, key=lambda x: x[0])[0],
            min(self.points, key=lambda x: x[1])[1],
            max(self.points, key=lambda x: x[0])[0],
            max(self.points, key=lambda x: x[1])[1],
        )
        self.polygon = Polygon(self.points)

    def colides(self, x, y):
        """
        Checks if the given point is in the obstacle or not.
        """

        return self.polygon.contains(Point(x, y))


class Wall:
    """
    Class implementing a wall with a moving hole in it

    Attributes
    ----------
    width : float
        The total width of the wall.
    bottom_y : float
        The position of the bottom of the wall.
    hole : float
        The position of the hole in the wall.
    thickness : float
        The thickness of the wall.
    speed : float
        The speed of the hole, if the obstacles are choosen to be dynamic.

    Methods
    -------
    colides
        Checks if a point is in the wall or not.
    visible
        Checks if the wall is in the field of view
    """

    def __init__(self, width, bottom_y, thickness, moving=False):
        self.width = width
        self.bottom_y = bottom_y
        self.hole = width * np.random.rand()
        self.thickness = thickness
        self.speed = (np.random.rand() - 1 / 2) * 2 if moving else 0

    def colides(self, x, y, time=0):
        """
        Checks if the given point is in the obstacle or not.
        """

        if time == 0:
            return (
                x < self.hole - self.width * 0.05 or x > self.hole + self.width * 0.05
            ) and (self.bottom_y <= y <= self.bottom_y + self.thickness)

        hole = (self.hole + self.speed * time) % (self.width)
        return (x < hole - self.width * 0.05 or x > hole + self.width * 0.05) and (
            self.bottom_y <= y <= self.bottom_y + self.thickness
        )

    def visible(self, view_top, view_bottom):
        """
        Checks if the wall is in the field of view
        """

        return (
            self.bottom_y <= view_top and self.bottom_y + self.thickness >= view_bottom
        )
