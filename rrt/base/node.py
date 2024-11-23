"""
Node class for the Rapidly Exploring Random Tree algorithm.
"""

from typing import List, Tuple


class Node:
    """
    Node of the rapidly exploring random tree.

    Attributes
    ----------
    index : int
        The index of the node.
    state : tuple
        The state represented by the node.
    cost : float
        The cost needed to reach this node.
    depth : int
        The depth of the node in the tree.
    parent_index : int, optional
        The index of the parent node.
    destination_list : list
        The indexes of reachable nodes from the current one.
    paths : list
        The paths from this node to its children.
    children_count : int
        The number of children nodes.
    children_max_depth : int
        The maximum depth among the children nodes.
    """

    __slots__ = [
        "index",
        "state",
        "cost",
        "depth",
        "parent_index",
        "destination_list",
        "paths",
        "children_count",
        "children_max_depth",
    ]

    def __init__(
        self,
        index: int,
        state: Tuple,
        cost: float,
        depth: int,
        parent_index: int = None,
    ):
        self.index: int = index
        self.state: Tuple = state
        self.cost: float = cost
        self.depth: int = depth
        self.parent_index: int = parent_index
        self.destination_list: List[int] = []
        self.paths: List[Tuple] = []
        self.children_count: int = 0
        self.children_max_depth: int = 0
