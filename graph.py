from abc import ABC, abstractmethod
from typing import Iterator, Any, Tuple, Set, Optional, List, Generator


class Graph(ABC):
    """
    Node and edge descriptor classes
    """

    class Node(ABC):
        @abstractmethod
        def get_data(self) -> Any:
            """
            Return the data associated with the node.

            Returns:
            Data of any type associated with the node.
            """
            pass

    class Edge(ABC):
        @abstractmethod
        def get_nodes(self) -> Tuple[Any, Any]:
            """
            Return the nodes associated with the edge.

            Returns:
            A tuple containing the two nodes associated with the edge.
            """
            pass

    @abstractmethod
    def get_neighbors(self, node: Any) -> Iterator[Any]:
        """
        Yield the neighbors of the given node.

        Parameters:
        node: The node for which to get neighbors.

        Yields:
        Neighboring nodes.
        """
        yield

    @abstractmethod
    def get_in_edges(self, node: Any) -> Iterator[Any]:
        """
        Yield the incoming edges for the given node.

        Parameters:
        node: The node for which to get incoming edges.

        Yields:
        Incoming edges.
        """
        yield

    @abstractmethod
    def get_out_edges(self, node: Any) -> Iterator[Any]:
        """
        Yield the outgoing edges for the given node.

        Parameters:
        node: The node for which to get outgoing edges.

        Yields:
        Outgoing edges.
        """
        yield

    @abstractmethod
    def get_all_nodes(self) -> Iterator[Any]:
        """
        Yield all nodes in the graph.

        Yields:
        Nodes in the graph.
        """
        yield

    @abstractmethod
    def get_all_edges(self) -> Iterator[Any]:
        """
        Yield all edges in the graph.

        Yields:
        Edges in the graph.
        """
        yield

    @abstractmethod
    def get_nodes_from_edge(self, edge: Any) -> Tuple[Any, Any]:
        """
        Return the nodes associated with the given edge.

        Parameters:
        edge: The edge for which to get the nodes.

        Returns:
        A tuple containing the two nodes associated with the edge.
        """
        pass
