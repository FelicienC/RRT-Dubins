"""
Construction of the Rapidely Exploring Random Tree
"""

import numpy as np
from rtree.index import Index as RTreeIndex
from rtree.index import Property
from rrt.environments.environment import Environment
from rrt.local_planners.default_planner import DefaultPlanner
from typing import List


class Node:
    """
    Node of the rapidly exploring random tree.

    Attributes
    ----------
    destination_list : list
        The reachable nodes from the current one.
    state : tuple
        The state represented by the node.
    cost : float
        The cost needed to reach this node.
    """

    __slots__ = [
        "destination_list",
        "state",
        "cost",
        "depth",
        "parent_index",
        "index",
        "children_count",
        "children_max_depth",
        "paths",
    ]

    def __init__(self, index, state, cost, depth, parent_index=None):
        self.destination_list: List[int] = []
        self.paths: List[tuple] = []
        self.state: tuple = state
        self.cost: float = cost
        self.parent_index: int = parent_index
        self.index: int = index
        self.depth = depth
        self.children_max_depth = 0
        self.children_count = 0


class RRT:
    """
    Class implementing a Rapidely Exploring Random Tree algorithm.

    Attributes
    ----------
    nodes : dict
        Dictionary containing all the nodes of the tree. The keys are hence
        simply the reached state, i.e. tuples.
    environment : Environment
        Instance of the Environment class.
    goal_rate : float
        The frequency at which the randomly selected node is chosen among
        the goal zone.
    precision : tuple
        The precision needed to stop the algorithm. It is a tuple of the same
        dimension as the state space.
    goal : tuple
        The position of the goal (the center of the goal zone), in the form of
        a tuple.
    root : tuple
        The state of the root of the tree, (the initial state).
    local_planner : Planner
        The planner used for the expansion of the tree, it needs to implement
        the method get_options and generate_points.

    Methods
    -------
    in_goal_region
        Method helping to determine if a point is within a goal region or not.
    run
        Executes the algorithm with an empty graph, which needs to be
        initialized with the start position at least before.
    select_options
        Explores the existing nodes of the tree to find the best option to grow
        from.
    """

    def __init__(
        self,
        environment: Environment,
        local_planner=None,
        precision=None,
    ) -> None:
        self.nodes: dict = {}
        self.root_index: int = 0
        self.goal: tuple
        if not local_planner:
            local_planner = DefaultPlanner(1)
        self.local_planner = local_planner
        if not precision:
            precision = [1] * len(environment.dimensions)
        self.precision = precision
        self.environment = environment
        self.rtree = RTreeIndex()
        self.node_index = 0
        self.max_depth = 0
        self.reached_goal = []

        self.validate()

    def validate(self) -> None:
        """
        Checks that the environment and the local planner are correctly
        implemented.
        """
        if not hasattr(self.environment, "is_free"):
            raise AttributeError(
                "The environment does not implement the method is_free"
            )
        if not hasattr(self.environment, "random_free_space"):
            raise AttributeError(
                "The environment does not implement the method random_free_space"
            )
        if not hasattr(self.local_planner, "get_next_state"):
            raise AttributeError(
                "The local planner does not implement the method get_next_state(state1, state2)"
            )
        if len(self.precision) != len(self.environment.dimensions):
            raise AttributeError(
                "The precision does not have the same dimension as the state space"
            )

    def set_start(self, start) -> None:
        """
        Resets the graph, and sets the start node as root of the tree.

        Parameters
        ----------
        start: tuple
            The initial state, used as root.
        """

        self.nodes = {}
        self.reached_goal = []
        self.rtree = RTreeIndex(properties=Property(dimension=len(start)))
        self.node_index = 0

        if self.is_valid_state(start):
            self.nodes[self.node_index] = Node(
                index=self.node_index,
                state=start,
                cost=0,
                depth=0,
            )

            self.root_index = 0
            self.rtree.add(self.node_index, start)

    def is_valid_state(self, state: tuple) -> bool:
        """
        Checks that the provided state is within the boundaries of the
        environment and in free space.
        """
        if len(state) != len(self.environment.dimensions):
            raise ValueError(
                "The provided state does not have the same dimension as the state space",
                f", expected {len(self.environment.dimensions)} got {len(state)}",
            )
        if not all(
            self.environment.dimensions[i][0]
            <= state[i]
            <= self.environment.dimensions[i][1]
            for i in range(len(state))
        ):
            raise ValueError(
                "The provided state is not within the boundaries of the environment",
                state,
            )
        if not self.environment.is_free(state):
            raise ValueError("The provided state is not in free space")
        return True

    def grow(
        self,
        goal,
        nb_iteration=100,
        goal_rate=0.05,
        metric="euclidean",
    ) -> None:
        """
        Executes the algorithm with an empty graph, initialized with the start
        position at least.

        Parameters
        ----------
        goal : tuple
            The final requested state as a tuple.
        nb_iteration : int
            The number of maximal iterations (not using the number of nodes as
            potentially the start is in a region of unavoidable collision).
        goal_rate : float
            The probability to expand towards the goal rather than towards a
            randomly selected sample.
        metric : string
            One of 'local' or 'euclidean'.
            The method used to select the closest node on the tree from which a
            path will be grown towards a sample.

        Notes
        -----
        It is not necessary to use several nodes to try and connect a sample to
        the existing graph; The closest node only could be chosen. The notion
        of "closest" can also be simply the euclidean distance, which would make
        the computation faster and the code a simpler, this is why several
        metrics are available.
        """
        if self.is_valid_state(goal):
            self.goal = goal

        for _ in range(nb_iteration):
            # Randomly select a sample, with a probability of goal_rate to be the goal.
            sample = (
                self.environment.random_free_space()
                if np.random.rand() > goal_rate
                else goal
            )

            # Find the closest node of the tree to the sample
            node, cost = self.get_closest_node(sample, metric=metric)

            # Try to connect the node to the sample
            path = self.local_planner.get_next_state(node.state, sample)
            for state in path:
                if not self.environment.is_free(state):
                    break
            else:
                # Adding the node to the tree
                self.add_node(state, parent_index=node.index, path=path)
                if self.in_goal_region(state):
                    self.reached_goal.append(self.node_index)

    def add_node(self, state, parent_index, path) -> None:
        """
        Adds a node to the tree, without checking for collisions.
        """
        self.node_index += 1
        index = self.node_index

        self.nodes[index] = Node(
            index=index,
            state=state,
            cost=0,  # As the cost, we use the distance
            parent_index=parent_index,
            depth=self.nodes[parent_index].depth + 1,
        )

        # updating the max depth and the deepest node
        if self.nodes[index].depth > self.max_depth:
            self.max_depth = self.nodes[index].depth
            self.deepest_node = index

        self.nodes[parent_index].destination_list.append(index)
        self.nodes[parent_index].paths.append(path)
        self.rtree.add(index, state)

        # Updating the number of children of the parents
        while parent_index != self.root_index:
            self.nodes[parent_index].children_count += 1
            self.nodes[parent_index].children_max_depth = max(
                self.nodes[parent_index].children_max_depth, self.nodes[index].depth
            )
            parent_index = self.nodes[parent_index].parent_index

    def get_closest_node(self, sample, metric="local") -> tuple:
        """
        Chooses the best nodes for the expansion of the tree, and returns
        them in a list ordered by increasing cost.

        Parameters
        ----------
        sample : tuple
            The state of the node we wish to connect to the tree.
        metric : str
            One of 'local', 'euclidean'. The euclidean metric is a lot faster
            but is also less precise and can't be used with an RRT star.

        Returns
        -------
        closest_node : Node
        length : float, the distance between the closest node and the sample according
                 to the metric

        """
        closest_node = self.nodes[next(self.rtree.nearest(sample))]
        if metric == "local":
            length = (
                len(self.local_planner.get_next_state(closest_node.state, sample))
                * self.local_planner.point_separation
            )
            return closest_node, length
        return closest_node, np.linalg.norm(
            np.array(closest_node.state) - np.array(sample)
        )

    def in_goal_region(self, sample) -> bool:
        """
        Method to determine if a point is within a goal region or not.

        Parameters
        ----------
        sample : tuple
            The state of the point which needs to be tested.
        """

        for i, value in enumerate(sample):
            if abs(self.goal[i] - value) > self.precision[i]:
                return False
        return True

    def select_largest_subtree(self) -> None:
        """
        Selects the best edge of the tree among the ones leaving from the root.
        Uses the number of children to determine the best option.

        Returns
        -------
        edge :Edge
            The best edge.
        """

        node_index = max(
            [
                (child, self.nodes[child].children_count)
                for child in self.nodes[self.root_index].destination_list
            ],
            key=lambda x: x[1],
        )[0]

        self.nodes[self.root_index].destination_list.remove(node_index)

        self.delete_all_children(self.root_index)
        self.rtree.delete(
            id=self.root_index, coordinates=self.nodes[self.root_index].state
        )
        self.nodes.pop(self.root_index)
        self.root_index = node_index
        self.nodes[node_index].parent_index = None

    def select_deepest_subtree(self) -> None:
        """
        Selects the best edge of the tree among the ones leaving from the root.
        Uses the number of children to determine the best option.

        Returns
        -------
        edge :Edge
            The best edge.
        """

        node_index = max(
            [
                (child, self.nodes[child].children_max_depth)
                for child in self.nodes[self.root_index].destination_list
            ],
            key=lambda x: x[1],
        )[0]

        self.nodes[self.root_index].destination_list.remove(node_index)

        self.delete_all_children(self.root_index)
        self.rtree.delete(
            id=self.root_index, coordinates=self.nodes[self.root_index].state
        )
        self.nodes.pop(self.root_index)
        self.root_index = node_index
        self.nodes[node_index].parent_index = None

    def delete_all_children(self, node_index) -> None:
        """
        Removes all the nodes of the tree below the requested node.
        """
        if self.nodes[node_index].destination_list:
            for child_index in self.nodes[node_index].destination_list:
                self.delete_all_children(child_index)
                self.rtree.delete(
                    id=child_index, coordinates=self.nodes[child_index].state
                )
                self.nodes.pop(child_index)

    def get_path_to_node(self, node_index: int) -> list[tuple]:
        """
        Returns the path from the root to the requested node.

        Parameters
        ----------
        node_index : int
            The index of the node from which the path is requested.

        Returns
        -------
        path : list
            The path from the root to the requested node. It is a list of
            successive states.
        """

        path = []
        while node_index > self.root_index:
            parent_index = self.nodes[node_index].parent_index
            for path_segment in self.nodes[parent_index].paths:
                if np.array_equal(path_segment[-1], self.nodes[node_index].state):
                    path.extend(path_segment[::-1])
                    break
            node_index = parent_index
        return path

    def get_path_to_node_vertices_only(self, node_index: int) -> list[tuple]:
        """
        Returns the path from the root to the requested node, but only the
        vertices of the path are returned.

        Parameters
        ----------
        node_index : int
            The index of the node from which the path is requested.

        Returns
        -------
        path : list
            The path from the root to the requested node. It is a list of
            successive states.
        """

        path = []
        while node_index > self.root_index:
            path.append(self.nodes[node_index].state)
            node_index = self.nodes[node_index].parent_index
        path.append(self.nodes[node_index].state)
        return path
