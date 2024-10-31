from __future__ import annotations
from typing import Dict
from time_intervals import *
from graph import *
import numpy as np

"""
SafeIntervalTable is the base constraint-container for safe interval path planning

Rather than storing constraints directly, it contains a set of intervals in which
a node or edge in a graph is safe to traverse.
"""


class SafeIntervalTable:
    def __init__(self, graph: Graph) -> None:
        """
        Initializes the Safe Interval Table for the given graph.

        Parameters:
        - graph: The MatrixGraph representing the warehouse environment.
        """
        self.graph = graph

        # Safe intervals for each graph node and edge
        # NOTE: We only store safe intervals for locations that have constraints
        #       If there are no constraints, the location is assumed safe
        #       (the default sate interval is from 0-infinity)
        self.node_intervals: Dict[Graph.Node, TimeIntervalSet] = {}
        self.edge_intervals: Dict[Graph.Edge, TimeIntervalSet] = {}

    def get_node_intervals(self, node: Graph.Node) -> TimeIntervalSet:
        """
        Retrieves the list of safe intervals for a given node.

        Parameters:
        - node: The node tuple (x, y, z).

        Returns:
        - A list of SafeInterval objects for the given node.
        - Defaults to 0->inf if no constraints have been defined
        """

        return self.node_intervals.get(node, TimeIntervalSet([TimeInterval(0, np.inf)]))

    def set_node_intervals(self, node: Graph.Node, intervals: TimeIntervalSet) -> None:
        """
        Sets the safe intervals for a given node.

        Parameters:
        - node: The node of the cell.
        - intervals: A list of SafeInterval objects to set for this node.
        """
        self.node_intervals[node] = intervals

    def add_node_constraint(self, node: Graph.Node, interval: TimeInterval) -> None:
        """
        Adds a constraint to the safe interval table, updating the safe intervals.

        Parameters:
        - node: The node of the constraint.
        - start_time: The time when the constraint starts.
        - end_time: The time when the constraint ends.
        """
        # Lazy initialize with default interval 0->inf
        if node not in self.node_intervals:
            self.node_intervals[node] = TimeIntervalSet([TimeInterval(0, np.inf)])

        # Apply constraint
        self.node_intervals[node].remove_interval(interval)
