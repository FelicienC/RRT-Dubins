"""
Construction of the Rapidely Exploring Random Tree
"""

import numpy as np
from rtree.index import Index as RTreeIndex
from rtree.index import Property
from rrt.environments.environment import Environment
from rrt.local_planners.default_planner import DefaultPlanner
from typing import List, Tuple
from rrt.base.node import Node  # Import the Node class


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
    precision : list
        The precision needed to stop the algorithm. It is a list of the same
        dimension as the state space.
    goal : tuple
        The position of the goal (the center of the goal zone), in the form of
        a tuple.
    root_index : int
        The index of the root of the tree, (the initial state).
    local_planner : Planner
        The planner used for the expansion of the tree, it needs to implement
        the method get_next_state.
    node_index : int
        The current index for the next node to be added.
    rtree : RTreeIndex
        The R-tree index for spatial indexing of nodes.
    max_depth : int
        The maximum depth of the tree.
    deepest_node : int
        The index of the deepest node in the tree.
    reached_goal : list
        List of node indices that have reached the goal.

    Methods
    -------
    set_start
        Resets the graph, and sets the start node as root of the tree.
    set_goal
        Sets the goal of the algorithm.
    grow
        Expands the tree by adding new nodes, starting from the initial state.
    add_node
        Adds a node to the tree, without checking for collisions.
    get_closest_node
        Chooses the best nodes for the expansion of the tree, and returns them in a list ordered by increasing cost.
    in_goal_region
        Method to determine if a point is within a goal region or not.
    select_largest_subtree
        Selects the best edge of the tree among the ones leaving from the root using the number of children.
    select_deepest_subtree
        Selects the best edge of the tree among the ones leaving from the root using the maximum depth of children.
    get_path_to_node
        Returns the path from the root to the requested node.
    get_path_to_node_vertices_only
        Returns the path from the root to the requested node, but only the vertices of the path are returned.
    """

    def __init__(
        self,
        environment: Environment,
        local_planner=None,
        precision: List[float] = None,
    ) -> None:
        # Environment and Precision
        self.environment = environment
        if not precision:
            precision = [1] * len(environment.dimensions)
        self.precision = precision
        self.goal: Tuple

        # Local Planner
        if not local_planner:
            local_planner = DefaultPlanner(1)
        self.local_planner = local_planner

        # Tree Structure
        self.nodes: dict[Node] = {}
        self.node_index = 0
        self.root_index: int = 0
        self.rtree = RTreeIndex()

        # Algorithm Parameters
        self.max_depth = 0
        self.deepest_node = self.root_index
        self.reached_goal = []

        # Validation of the correct instantiation
        self._validate()

    def _validate(self) -> None:
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

    def set_start(self, start: Tuple) -> None:
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

        if self._is_valid_state(start):
            self.nodes[self.node_index] = Node(
                index=self.node_index,
                state=start,
                cost=0,
                depth=0,
            )

            self.root_index = 0
            self.rtree.add(self.node_index, start)

    def set_goal(self, goal: Tuple) -> None:
        """
        Sets the goal of the algorithm.

        Parameters
        ----------
        goal : tuple
            The final requested state as a tuple.
        """
        if self._is_valid_state(goal):
            self.goal = goal

    def _is_valid_state(self, state: Tuple) -> bool:
        """
        Checks that the provided state is within the boundaries of the
        environment and in free space.

        Parameters
        ----------
        state : tuple
            The state to be checked.

        Returns
        -------
        bool
            True if the state is valid, raises ValueError otherwise.
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
        nb_iteration: int = 100,
        goal_rate: float = 0.05,
        metric: str = "euclidean",
    ) -> None:
        """
        Expands the tree by adding new nodes, starting from the initial state.

        Parameters
        ----------
        nb_iteration : int
            The maximum number of iterations to run the algorithm.
        goal_rate : float
            The probability of selecting the goal as the target sample for expansion.
        metric : string
            The metric used to select the closest node in the tree. One of 'local' or 'euclidean'.

        Notes
        -----
        The algorithm attempts to connect randomly selected samples to the tree.
        With a probability of `goal_rate`, the goal is selected as the target sample.
        The closest node in the tree is determined based on the specified metric.
        If a valid path is found, the new node is added to the tree.
        """
        for _ in range(nb_iteration):
            # Randomly select a sample, with a probability of goal_rate to be the goal.
            sample = (
                self.environment.random_free_space()
                if np.random.rand() > goal_rate
                else self.goal
            )

            # Find the closest node of the tree to the sample
            node, cost = self.get_closest_node(sample, metric=metric)

            # Try to connect the sample to the tree
            path = self.local_planner.get_next_state(node.state, sample)
            for state in path:
                if not self.environment.is_free(state):
                    break
            else:
                self.add_node(state, parent_index=node.index, path=path)
                if self.in_goal_region(state):
                    self.reached_goal.append(self.node_index)

    def add_node(self, state: Tuple, parent_index: int, path: List[Tuple]) -> None:
        """
        Adds a node to the tree, without checking for collisions.

        Parameters
        ----------
        state : tuple
            The state of the new node.
        parent_index : int
            The index of the parent node.
        path : list
            The path from the parent node to the new node.
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

    def get_closest_node(
        self, sample: Tuple, metric: str = "local"
    ) -> Tuple[Node, float]:
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
            The closest node to the sample.
        length : float
            The distance between the closest node and the sample according to the metric.
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

    def in_goal_region(self, sample: Tuple) -> bool:
        """
        Method to determine if a point is within a goal region or not.

        Parameters
        ----------
        sample : tuple
            The state of the point which needs to be tested.

        Returns
        -------
        bool
            True if the sample is within the goal region, False otherwise.
        """
        for i, value in enumerate(sample):
            if abs(self.goal[i] - value) > self.precision[i]:
                return False
        return True

    def _key_func_largest(self, x):
        return self.nodes[x].children_count

    def _key_func_deepest(self, x):
        return self.nodes[x].children_max_depth

    def _select_subtree(self, criterion: str) -> None:
        """
        Selects the best edge of the tree among the ones leaving from the root.
        Uses the specified criterion to determine the best option.

        Parameters
        ----------
        criterion : str
            The criterion to use for selecting the subtree. One of 'largest' or 'deepest'.
        """
        if criterion == "largest":
            key_func = self._key_func_largest
        elif criterion == "deepest":
            key_func = self._key_func_deepest
        else:
            raise ValueError("Invalid criterion. Use 'largest' or 'deepest'.")

        node_index = max(
            [
                (child, key_func(child))
                for child in self.nodes[self.root_index].destination_list
            ],
            key=lambda x: x[1],
        )[0]

        self.nodes[self.root_index].destination_list.remove(node_index)

        self._delete_all_children(self.root_index)
        self.rtree.delete(
            id=self.root_index, coordinates=self.nodes[self.root_index].state
        )
        self.nodes.pop(self.root_index)
        self.root_index = node_index
        self.nodes[node_index].parent_index = None

    def select_largest_subtree(self) -> None:
        """
        Selects the best edge of the tree among the ones leaving from the root.
        Uses the number of children to determine the best option.
        """
        self._select_subtree(criterion="largest")

    def select_deepest_subtree(self) -> None:
        """
        Selects the best edge of the tree among the ones leaving from the root.
        Uses the maximum depth of children to determine the best option.
        """
        self._select_subtree(criterion="deepest")

    def _delete_all_children(self, node_index) -> None:
        """
        Removes all the nodes of the tree below the requested node.

        Parameters
        ----------
        node_index : int
            The index of the node whose children are to be deleted.
        """
        if self.nodes[node_index].destination_list:
            for child_index in self.nodes[node_index].destination_list:
                self._delete_all_children(child_index)
                self.rtree.delete(
                    id=child_index, coordinates=self.nodes[child_index].state
                )
                self.nodes.pop(child_index)

    def get_path_to_node(self, node_index: int) -> List[Tuple]:
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

    def get_path_to_node_vertices_only(self, node_index: int) -> List[Tuple]:
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
