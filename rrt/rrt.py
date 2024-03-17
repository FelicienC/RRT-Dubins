"""
Construction of the Rapidely Exploring Random Tree
"""

from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from rrt.dubins import Dubins, dist


class Node:
    """
    Node of the rapidly exploring random tree.

    Attributes
    ----------
    destination_list : list
        The reachable nodes from the current one.
    position : tuple
        The position of the node.
    time : float
        The instant at which this node is reached.
    cost : float
        The cost needed to reach this node.
    """

    def __init__(self, position, time, cost):
        self.destination_list = []
        self.position = position
        self.time = time
        self.cost = cost


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
    Class implementing a Rapidely Exploring Random Tree in two dimensions using
    dubins paths as an expansion method. The state space considered here is
    straightforward, as every state can be represented by a simple tuple made
    of three continuous variables: (x, y, psi)

    Attributes
    ----------
    nodes : dict
        Dictionnary containing all the nodes of the tree. The keys are hence
        simply the reached state, i.e. tuples of the form (x, y, psi).
    environment : Environment
        Instance of the Environment class.
    goal_rate : float
        The frequency at which the randomly selected node is choosen among
        the goal zone.
    precision : tuple
            The precision needed to stop the algorithm. In the form
            (delta_x, delta_y, delta_psi).
    goal : tuple
        The position of the goal (the center of the goal zone), in the form of
        a tuple (x, y, psi).
    root : tuple
        The position of the root of the tree, (the initial position of the
        vehicle), in the form of a tuple (x, y, psi).
    local_planner : Planner
        The planner used for the expansion of the tree, here it is a Dubins
        path planner.

    Methods
    -------
    in_goal_region
        Method helping to determine if a point is within a goal region or not.
    run
        Executes the algorithm with an empty graph, which needs to be
        initialized with the start position at least before.
    plot_tree
        Displays the RRT using matplotlib.
    select_options
        Explores the existing nodes of the tree to find the best option to grow
        from.
    """

    def __init__(self, environment, precision=(5, 5, 1)):
        self.nodes = {}
        self.edges = {}
        self.environment = environment
        self.local_planner = Dubins(4, 1)
        self.goal = (0, 0, 0)
        self.root = (0, 0, 0)
        self.precision = precision

    def set_start(self, start):
        """
        Resets the graph, and sets the start node as root of the tree.

        Parameters
        ----------
        start: tuple
            The initial position (x, y, psi), used as root.
        """

        self.nodes = {}
        self.edges = {}
        self.nodes[start] = Node(start, 0, 0)
        self.root = start

    def run(self, goal, nb_iteration=100, goal_rate=0.1, metric="local"):
        """
        Executes the algorithm with an empty graph, initialized with the start
        position at least.

        Parameters
        ----------
        goal : tuple
            The final requested position (x, y, psi).
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

        assert len(goal) == len(self.precision)
        self.goal = goal

        for _ in range(nb_iteration):
            # Select sample : either the goal, or a sample of free space
            if np.random.rand() > 1 - goal_rate:
                sample = goal
            else:
                sample = self.environment.random_free_space()
            # Find the closest Node in the tree, with the selected metric
            options = self.select_options(sample, 10, metric)

            # Now that all the options are sorted from the shortest to the
            # longest, we can try to connect one node after the other. We stop
            # after 10 in order to limit computations.
            for node, option in options:
                if option[0] == float("inf"):
                    break
                path = self.local_planner.generate_points(
                    node, sample, option[1], option[2]
                )
                for i, point in enumerate(path):
                    if not self.environment.is_free(
                        point[0], point[1], self.nodes[node].time + i
                    ):
                        break
                else:
                    # Adding the node
                    # To compute the time, we use a constant speed of 1 m/s
                    # As the cost, we use the distance
                    self.nodes[sample] = Node(
                        sample,
                        self.nodes[node].time + option[0],
                        self.nodes[node].cost + option[0],
                    )
                    self.nodes[node].destination_list.append(sample)
                    # Adding the Edge
                    self.edges[node, sample] = Edge(node, sample, path, option[0])
                    if self.in_goal_region(sample):
                        return
                    break

    def select_options(self, sample, nb_options, metric="local"):
        """
        Chooses the best nodes for the expansion of the tree, and returns
        them in a list ordered by increasing cost.

        Parameters
        ----------
        sample : tuple
            The (x, y, psi) coordinates of the node we wish to connect to the
            tree.
        nb_options : int
            The number of options requested.
        metric : str
            One of 'local', 'euclidian'. The euclidian metric is a lot faster
            but is also less precise and can't be used with an RRT star.

        Returns
        -------
        options : list
            Sorted list of the options, by increasing cost.
        """

        if metric == "local":
            # The local planner is used to measure the real distance needed
            options = []
            for node in self.nodes:
                options.extend(
                    [
                        (node, opt)
                        for opt in self.local_planner.all_options(node, sample)
                    ]
                )
            # sorted by cost
            options.sort(key=lambda x: x[1][0])
            options = options[:nb_options]
        else:
            # Euclidian distance as a metric
            options = [(node, dist(node, sample)) for node in self.nodes]
            options.sort(key=lambda x: x[1])
            options = options[:nb_options]
            new_opt = []
            for node, _ in options:
                db_options = self.local_planner.all_options(node, sample)
                new_opt.append((node, min(db_options, key=lambda x: x[0])))
            options = new_opt
        return options

    def in_goal_region(self, sample):
        """
        Method to determine if a point is within a goal region or not.

        Parameters
        ----------
        sample : tuple
            (x, y, psi) the position of the point which needs to be tested.
        """

        for i, value in enumerate(sample):
            if abs(self.goal[i] - value) > self.precision[i]:
                return False
        return True

    def plot(self, file_name="", close=False, nodes=False):
        """
        Displays the tree using matplotlib, on a currently open figure.

        Parameters
        ----------
        file_name : string
            The name of the file used to save an image of the tree.
        close : bool
            If the plot needs to be automaticaly closed after the drawing.
        nodes : bool
            If the nodes need to be displayed as well.
        """

        if nodes and self.nodes:
            nodes = np.array(list(self.nodes.keys()))
            plt.scatter(nodes[:, 0], nodes[:, 1])
            plt.scatter(self.root[0], self.root[1], c="g")
            plt.scatter(self.goal[0], self.goal[1], c="r")
        for _, val in self.edges.items():
            if val.path:
                path = np.array(val.path)
                plt.plot(path[:, 0], path[:, 1], "r")
        if file_name:
            plt.savefig(file_name)
        if close:
            plt.close()

    def select_best_edge(self):
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
                for child in self.nodes[self.root].destination_list
            ],
            key=lambda x: x[1],
        )[0]
        best_edge = self.edges[(self.root, node)]
        # we update the tree to remove all the other siblings of the old root
        for child in self.nodes[self.root].destination_list:
            if child == node:
                continue
            self.edges.pop((self.root, child))
            self.delete_all_children(child)
        self.nodes.pop(self.root)
        self.root = node
        return best_edge

    def delete_all_children(self, node):
        """
        Removes all the nodes of the tree below the requested node.
        """

        if self.nodes[node].destination_list:
            for child in self.nodes[node].destination_list:
                self.edges.pop((node, child))
                self.delete_all_children(child)
        self.nodes.pop(node)

    def children_count(self, node):
        """
        Not optimal at all as it recounts a lot of the tree every time a path
        needs to b selected.
        """

        if not self.nodes[node].destination_list:
            return 0
        total = 0
        for child in self.nodes[node].destination_list:
            total += 1 + self.children_count(child)
        return total
