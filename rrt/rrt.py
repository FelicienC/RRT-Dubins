"""
Construction of the Rapidely Exploring Random Tree
"""

from collections import deque
import numpy as np
from rrt.dubins import Dubins, dist
from rtree.index import Index as RTreeIndex
from rtree.index import Property

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

    def __init__(self, index, state, cost, parent_index=None):
        self.destination_list = []
        self.state = state
        self.cost = cost
        self.parent = parent_index
        self.index = index


class Edge:
    """
    Edge of the rapidly exploring random tree.

    Attributes
    ----------
    node_from : tuple
        Id of the starting node of the edge.
    node_to : tuple
        Id of the end node of the edge.
    path : list
        The successive positions yielded by the local planner representing the
        path between the nodes.
    cost : float
        Cost associated to the transition between the two nodes.

    """

    def __init__(self, node_from, node_to, path, cost):
        self.node_from = node_from
        self.node_to = node_to
        self.path = deque(path)
        self.cost = cost


class RRT:
    """
    Class implementing a Rapidely Exploring Random Tree algorithm.

    Attributes
    ----------
    nodes : dict
        Dictionnary containing all the nodes of the tree. The keys are hence
        simply the reached state, i.e. tuples.
    environment : Environment
        Instance of the Environment class.
    goal_rate : float
        The frequency at which the randomly selected node is choosen among
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

    def __init__(self, environment, local_planner, precision):
        self.nodes: dict = {}
        self.edges: dict = {}
        self.root: tuple
        self.goal: tuple
        self.precision = precision
        self.environment = environment
        self.local_planner = local_planner
        self.rtree = RTreeIndex()
        self.node_index = 0
        self.reached_goal = []

        self.validate()

    def validate(self):
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
        if not hasattr(self.local_planner, "get_path"):
            raise AttributeError(
                "The local planner does not implement the method get_path(state1, state2)"
            )
        if len(self.precision) != len(self.environment.dimensions):
            raise AttributeError(
                "The precision does not have the same dimension as the state space"
            )

    def set_start(self, start):
        """
        Resets the graph, and sets the start node as root of the tree.

        Parameters
        ----------
        start: tuple
            The initial state, used as root.
        """

        self.nodes = {}
        self.edges = {}
        self.reached_goal = []
        self.rtree = RTreeIndex(properties=Property(dimension=len(start)))
        self.node_index = 0

        if self.is_valid_state(start):
            self.nodes[self.node_index] = Node(
                index=self.node_index, state=start, cost=0
            )
            self.root = start
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
            0 <= state[i] <= self.environment.dimensions[i] for i in range(len(state))
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
        metric="local",
    ) -> List:
        """
        Executes the algorithm with an empty graph, initialized with the start
        position at least.

        Parameters
        ----------
        goal : tuple
            The final requested state as a tuple.
        nb_iteration : int
            The number of maximal iterations (not using the number of nodes as
            potentialy the start is in a region of unavoidable collision).
        goal_rate : float
            The probability to expand towards the goal rather than towards a
            randomly selected sample.
        metric : string
            One of 'local' or 'euclidian'.
            The method used to select the closest node on the tree from which a
            path will be grown towards a sample.

        Notes
        -----
        It is not necessary to use several nodes to try and connect a sample to
        the existing graph; The closest node only could be choosen. The notion
        of "closest" can also be simpy the euclidian distance, which would make
        the computation faster and the code a simpler, this is why several
        metrics are available.
        """
        if self.is_valid_state(goal):
            self.goal = goal

        for _ in range(nb_iteration):

            # Randomly select a sample, with a probability of goal_rate to be
            # the goal.
            sample = (
                self.environment.random_free_space()
                if np.random.rand() > goal_rate
                else goal
            )

            # Find the closest node of the tree to the sample
            node, cost = self.get_closest_node(sample, metric=metric)

            # Try to connect the node to the sample
            path = self.local_planner.get_path(node.state, sample)
            for state in path:
                if not self.environment.is_free(state):
                    break
            else:
                # Adding the node to the tree
                self.add_node(state, parent_index=node.index, path=path)
                if self.in_goal_region(state):
                    self.reached_goal.append(self.node_index - 1)

    def add_node(self, state, parent_index, path):
        """
        Adds a node to the tree, without checking for collisions.
        """
        index = self.node_index
        self.nodes[index] = Node(
            index=index,
            state=state,
            cost=0,  # As the cost, we use the distance
            parent_index=parent_index,
        )
        self.nodes[index].destination_list.append(state)
        self.rtree.add(index, state)
        self.edges[parent_index, index] = Edge(parent_index, index, path, 1)
        self.node_index += 1

    def get_closest_node(self, sample, metric="local"):
        """
        Chooses the best nodes for the expansion of the tree, and returns
        them in a list ordered by increasing cost.

        Parameters
        ----------
        sample : tuple
            The state of the node we wish to connect to the tree.
        metric : str
            One of 'local', 'euclidian'. The euclidian metric is a lot faster
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
                len(self.local_planner.get_path(closest_node.state, sample))
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

    def selet_largest_subtree(self) -> Edge:
        """
        Selects the best edge of the tree among the ones leaving from the root.
        Uses the number of children to determine the best option.

        Returns
        -------
        edge :Edge
            The best edge.
        """

        node = max(
            [
                (child, self.children_count(child))
                for child in self.nodes[0].destination_list
            ],
            key=lambda x: x[1],
        )[0]
        best_edge = self.edges[(self.root, node)]
        # we update the tree to remove all the other siblings of the old root
        for child in self.nodes[0].destination_list:
            if child == node:
                continue
            self.edges.pop((self.root, child))
            self.delete_all_children(child)
        self.nodes.pop(self.root)
        self.root = node
        return best_edge

    def delete_all_children(self, node) -> None:
        """
        Removes all the nodes of the tree below the requested node.
        """

        if self.nodes[node].destination_list:
            for child in self.nodes[node].destination_list:
                self.edges.pop((node, child))
                self.delete_all_children(child)
        self.nodes.pop(node)

    def children_count(self, node) -> int:
        """
        Not optimal at all as it recounts a lot of the tree every time a path
        needs to be selected.
        """

        if not self.nodes[node].destination_list:
            return 0
        total = 0
        for child in self.nodes[node].destination_list:
            total += 1 + self.children_count(child)
        return total

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
        while node_index > 0:
            path.extend(
                list(self.edges[(self.nodes[node_index].parent, node_index)].path)[::-1]
            )
            node_index = self.nodes[node_index].parent
        return path
