import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pat

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

    Methods
    -------
    plot
        Displays the polygon on screen

    """

    def __init__(self, map_dimensions, size, nb_pts):
        self.center = np.array([np.random.rand()*map_dimensions[0],
                                np.random.rand()*map_dimensions[1]])
        # We use very simple convex polygons, generated with a radius
        # and randomly selected angles.
        angles = sorted((np.random.rand()*2*np.pi for _ in range(nb_pts)))
        self.points = \
            np.array([self.center +\
                      np.array([size*np.cos(angle), size*np.sin(angle)])\
                      for angle in angles])
        self.bounding_box = (min(self.points, key=lambda x: x[0])[0],
                             min(self.points, key=lambda x: x[1])[1],
                             max(self.points, key=lambda x: x[0])[0],
                             max(self.points, key=lambda x: x[1])[1])
        self.polygon = Polygon(self.points)

    def colides(self, x, y):
        """
        Checks if the given point is in the obstacle or not.
        """
        return self.polygon.contains(Point(x, y))

    def plot(self):
        """
        Draws the polygon on screen using lines only.
        """
        plt.gca().add_patch(pat.Polygon(self.points, color='black', fill=True))
