"""
Default planner
"""

import numpy as np


class DefaultPlanner:
    """
    Default planner trying to connect two points in a straight line in their state space.
    """

    def __init__(self, point_separation=1) -> None:
        self.point_separation = point_separation

    def get_options(self, state1, state2) -> list[list]:
        """
        Given two states, it returns the options to connect them.

        Args:
            state1 (np.ndarray): The first state
            state2 (np.ndarray): The second state

        Returns:
            list[list]: A list containing a single option to connect the two states in
                the form [cost, path].
        """
        return [[0, self.get_path(state1, state2)]]

    def get_path(self, state1, state2) -> list[np.ndarray]:
        """
        Given two states, it returns the path [state1, stateX].

        stateX is the point obtained by starting at state1 and moving along the vetor
        vect(state1, state2) for the distance point_separation.

        Args:
            state1 (np.ndarray): The first state
            state2 (np.ndarray): The second state

        Returns:
            list[np.ndarray]: The path [state1, stateX]
        """
        return [
            state1,
            state1
            + self.point_separation
            * (state2 - state1)
            / np.linalg.norm(state2 - state1),
        ]
