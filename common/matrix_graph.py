from __future__ import annotations
from typing import Tuple, Set, Optional, List, Generator
from common.graph import Graph


"""
MatrixGraph is an implicit representation 
"""
# x,y,z index to a cell in the graph
Coordinate = Tuple[int, int, int]


class MatrixGraph(Graph):
    class Node(Graph.Node):
        def __init__(self, coordinate: Coordinate) -> None:
            self.coordinate = coordinate

        def get_data(self) -> Coordinate:
            return self.coordinate

    class Edge(Graph.Edge):
        def __init__(self, node1: Coordinate, node2: Coordinate) -> None:
            self.node1 = node1
            self.node2 = node2

        def get_nodes(self) -> Tuple[Coordinate, Coordinate]:
            return (self.node1, self.node2)

    def __init__(
        self,
        dimensions: Coordinate,
        cell_size: Tuple[float, float, float] = (1.25, 1.25, 2.0),
        disabled_cells: Optional[Set[Coordinate]] = None,
    ) -> None:
        """
        Initializes MatrixGraph representing cells spanning 3d volume.

        Parameters:
        - dimensions: A tuple (nx, ny, nz) representing the number of cells in x, y, and z dimensions.
        - cell_size: A tuple (dx, dy, dz) representing the physical size of each cell (default is (1.25, 1.25, 2.0) meters).
        - disabled_cells: An optional set of coordinates representing disabled cells.
        """
        self.nx, self.ny, self.nz = dimensions
        self.dx, self.dy, self.dz = cell_size
        self.disabled_cells: Set[Coordinate] = (
            disabled_cells if disabled_cells else set()
        )

    def get_neighbors(
        self, coordinate: Coordinate
    ) -> Generator[Coordinate, None, None]:
        """
        Generates the coordinates of adjacent cells to the given coordinate.

        Parameters:
        - coordinate: A tuple (x, y, z) representing the current cell.

        Yields:
        - neighbor_coord: A coordinate tuple representing an adjacent cell.
        """
        x, y, z = coordinate
        neighbor_offsets: List[Coordinate] = [
            (1, 0, 0),
            (-1, 0, 0),  # Adjacent cells along the x-axis
            (0, 1, 0),
            (0, -1, 0),  # Adjacent cells along the y-axis
            (0, 0, 1),
            (0, 0, -1),  # Adjacent cells along the z-axis
        ]
        for dx, dy, dz in neighbor_offsets:
            nx, ny, nz = x + dx, y + dy, z + dz
            neighbor_coord: Coordinate = (nx, ny, nz)
            if (
                self._is_valid_coordinate(neighbor_coord)
                and neighbor_coord not in self.disabled_cells
            ):
                yield neighbor_coord

    def get_in_edges(self, node: Coordinate) -> Generator[MatrixGraph.Edge, None, None]:
        """
        Yield the incoming edges for the given node.

        Parameters:
        - node: The node for which to get incoming edges.

        Yields:
        - An Edge instance representing an incoming edge.
        """
        yield from (self.Edge(neighbor, node) for neighbor in self.get_neighbors(node))

    def get_out_edges(
        self, node: Coordinate
    ) -> Generator[MatrixGraph.Edge, None, None]:
        """
        Yield the outgoing edges for the given node.

        Parameters:
        - node: The node for which to get outgoing edges.

        Yields:
        - An Edge instance representing an outgoing edge.
        """
        yield from (self.Edge(node, neighbor) for neighbor in self.get_neighbors(node))

    def get_all_nodes(self) -> Generator[MatrixGraph.Node, None, None]:
        """
        Yield all nodes in the graph.

        Yields:
        - Nodes in the graph.
        """
        for x in range(self.nx):
            for y in range(self.ny):
                for z in range(self.nz):
                    coord: Coordinate = (x, y, z)
                    if coord not in self.disabled_cells:
                        yield self.Node(coord)

    def get_all_edges(self) -> Generator[MatrixGraph.Edge, None, None]:
        """
        Yield all edges in the graph.

        Yields:
        - Edges in the graph.
        """
        for node in self.get_all_nodes():
            yield from self.get_out_edges(node.get_data())

    def get_nodes_from_edge(
        self, edge: MatrixGraph.Edge
    ) -> Tuple[MatrixGraph.Node, MatrixGraph.Node]:
        """
        Return the nodes associated with the given edge.

        Parameters:
        - edge: The edge for which to get the nodes.

        Returns:
        - A tuple containing the two nodes associated with the edge.
        """
        return edge.get_nodes()

    def _is_valid_coordinate(self, coord: Coordinate) -> bool:
        """
        Checks if the coordinate is within the lattice boundaries.

        Parameters:
        - coord: A tuple (x, y, z) representing a coordinate.

        Returns:
        - True if the coordinate is within the lattice boundaries, False otherwise.
        """
        x, y, z = coord
        return (0 <= x < self.nx) and (0 <= y < self.ny) and (0 <= z < self.nz)
