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
        List of (x, y) coordinates in the frame of the environment
        representing the obstacle.
    bounding_box : 4-tuple
        Coordinates of the lower left and upper right corners of the bounding
        box containing the obstacle.
    center : tuple
        Coordinates of the center of the bounding box.
    polygon : shapely.geometry.Polygon
        The polygon representing the obstacle.

    """

    def __init__(self, map_dimensions: list, size: float, nb_pts: int) -> None:
        """
        Initializes an Obstacle instance.

        Parameters
        ----------
        map_dimensions : list
            List of (min, max) tuples for each dimension of the environment.
        size : float
            Size (radius) of the obstacle.
        nb_pts : int
            Number of points defining the polygonal obstacle.
        """
        self.center = [
            np.random.uniform(dim_min, dim_max) for (dim_min, dim_max) in map_dimensions
        ]
        # We use very simple convex polygons, generated with a radius
        # and randomly selected angles.
        angles = sorted(np.random.uniform(0, 2 * np.pi, nb_pts))
        self.points = np.array(
            [
                self.center[:2] + np.array([size * np.cos(angle), size * np.sin(angle)])
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

    def collides(self, x: float, y: float) -> bool:
        """
        Checks if the given point is inside the obstacle.

        Parameters
        ----------
        x : float
            X-coordinate of the point.
        y : float
            Y-coordinate of the point.

        Returns
        -------
        bool
            True if the point is inside the obstacle, False otherwise.
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
        The speed of the hole, if the obstacles are chosen to be dynamic.

    Methods
    -------
    collides
        Checks if a point is in the wall or not.
    visible
        Checks if the wall is in the field of view
    """

    def __init__(self, width: float, bottom_y: float, thickness: float, moving: bool = False) -> None:
        """
        Initializes a Wall instance with an optional moving hole.

        Parameters
        ----------
        width : float
            Total width of the wall.
        bottom_y : float
            Y-coordinate of the bottom of the wall.
        thickness : float
            Thickness (height) of the wall.
        moving : bool, optional
            If True, the hole in the wall moves over time. Defaults to False.
        """
        self.width = width
        self.bottom_y = bottom_y
        self.hole = width * np.random.rand()
        self.thickness = thickness
        self.speed = (np.random.rand() - 1 / 2) * 2 if moving else 0

    def collides(self, x: float, y: float, time: float = 0) -> bool:
        """
        Checks if the given point collides with the wall at a specific time.

        Parameters
        ----------
        x : float
            X-coordinate of the point.
        y : float
            Y-coordinate of the point.
        time : float, optional
            Time at which to check for collision. Defaults to 0.

        Returns
        -------
        bool
            True if the point collides with the wall, False otherwise.
        """

        if time == 0:
            return (
                x < self.hole - self.width * 0.05 or x > self.hole + self.width * 0.05
            ) and (self.bottom_y <= y <= self.bottom_y + self.thickness)

        hole = (self.hole + self.speed * time) % (self.width)
        return (x < hole - self.width * 0.05 or x > hole + self.width * 0.05) and (
            self.bottom_y <= y <= self.bottom_y + self.thickness
        )

    def visible(self, view_top: float, view_bottom: float) -> bool:
        """
        Determines if the wall is within the given field of view.

        Parameters
        ----------
        view_top : float
            Top boundary of the field of view.
        view_bottom : float
            Bottom boundary of the field of view.

        Returns
        -------
        bool
            True if the wall is visible within the field of view, False otherwise.
        """

        return (
            self.bottom_y <= view_top and self.bottom_y + self.thickness >= view_bottom
        )
