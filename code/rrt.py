import matplotlib.pyplot as plt
import numpy as np
from dubins import Dubins, dist

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
        self.path = path
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

    def __init__(self, environment):
        self.nodes = {}
        self.edges = {}
        self.environment = environment
        self.local_planner = Dubins(2, .5)
        self.goal = (0, 0, 0)
        self.start = (0, 0, 0)
        self.precision = (10, 10, .1)
        self.goal_rate = .2

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
        self.start = start

    def run(self, goal, precision, nb_iteration=100, metric='local'):
        """
        Executes the algorithm with an empty graph, initialized with the start
        position at least.

        Parameters
        ----------
        goal : tuple
            The final requested position (x, y, psi).
        precision : tuple
            The precision needed to stop the algorithm. In the form
            (delta_x, delta_y, delta_psi).
        nb_iteration : int
            The number of maximal iterations (not using the number of nodes as
            potentialy the start is in a region of unavoidable collision).

        Notes
        -----
        It is not necessary to use several nodes to try and connect a sample to
        the existing graph; The closest node only could be choosen. The notion
        of "closest" can also be simpy the euclidian distance, which would make
        the computation faster and the code a simpler, this is why several
        metrics are available.
        
        """
        assert len(goal) == len(precision)
        self.goal = goal
        self.precision = precision
        for _ in range(nb_iteration):
            # Select sample : either the goal, or a sample of free space
            if np.random.rand() > 1-self.goal_rate:
                sample = goal
            else:
                sample = self.environment.random_free_space()
            # Find the closest Node in the tree, with the cost defined by the
            # local planner (here we use the lenght of the path as cost)
            options = self.select_options(sample, 10, metric)

            # Now that all the options are sorted from the shortest to the
            # longest, we can try to connect one node after the other. We stop
            # after 20 in order to reduce the number of computations.
            for node, option in options:
                if option[0] == float('inf'):
                    break
                path = self.local_planner.generate_points(node,
                                                          sample,
                                                          option[1],
                                                          option[2])
                for point in path:
                    if not self.environment.is_free(point[0], point[1]):
                        break
                else:
                    # Adding the node
                    # To compute the time, we use a constant speed of 1 m/s
                    sample_time = self.nodes[node].time + option[0]
                    # We use the distance as the cost, to keep it simple
                    sample_cost = self.nodes[node].cost + option[0]
                    self.nodes[sample] = Node(sample,
                                              sample_time,
                                              sample_cost)
                    self.nodes[node].destination_list.append(sample)
                    # Adding the Edge
                    self.edges[node, sample] = \
                        Edge(node, sample, path, option[0])
                    if self.in_goal_region(sample):
                        print('success', self.nodes[sample].cost)
                        return
                    break
        print('Failure')

    def select_options(self, sample, nb_options, metric='local'):
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
        if metric == 'local':
            # The local planner is used to measure the real distance needed
            options = []
            for node in self.nodes:
                options.extend(
                    [(node, opt)\
                     for opt in self.local_planner.all_options(node, sample)])
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
            (x, y, psi) the position of the point which needs to be tested

        """
        for i, value in enumerate(sample):
            if abs(self.goal[i]-value) > self.precision[i]:
                return False
        return True

    def plot_tree(self):
        """
        Displays the tree using matplotlib
        """

        if self.nodes:
            nodes = np.array(list(self.nodes.keys()))
            plt.scatter(nodes[:, 0], nodes[:, 1])
            plt.scatter(self.start[0], self.start[1], c='g')
            plt.scatter(self.goal[0], self.goal[1], c='r')
        for _, val in self.edges.items():
            plt.plot(val.path[:, 0], val.path[:, 1], 'r')
