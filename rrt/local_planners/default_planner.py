"""
Default planner
"""

import numpy as np


class DefaultPlanner:
    """
    Default planner trying to connect two points in a straight line in their state space.
    """

    def __init__(self, point_separation=1) -> None:
        if point_separation <= 0:
            raise ValueError("point_separation must be a positive number")
        self.point_separation = point_separation

    def get_next_state(self, state1, state2) -> list[np.ndarray]:
        """
        Given two states, it computes the path from state 1 to state 2.

        The computed path does not include state1. Here, the path only contains one
        point, stateX. It is the point obtained by starting at state1 and moving along
        the vector vect(state1, state2) for the distance point_separation.

        Args:
            state1 (np.ndarray): The first state
            state2 (np.ndarray): The second state

        Returns:
            list[np.ndarray]: The path [stateX]
        """
        return [
            state1,
            state1
            + self.point_separation
            * (state2 - state1)
            / np.linalg.norm(state2 - state1),
        ]
