"""
Test module, making sure the main functionalities are always functioning
"""

from rrt.dubins import Dubins


def test_dubins():
    """
    Tests that the class Dubins initializes
    """

    Dubins(10, 1)
