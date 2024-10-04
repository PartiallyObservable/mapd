from typing import Tuple, Set, Optional, List, Generator


class MatrixGraph:
    def __init__(
        self,
        dimensions: Tuple[int, int, int],
        cell_size: Tuple[float, float, float] = (1.25, 1.25, 2.0),
        disabled_cells: Optional[Set[Tuple[int, int, int]]] = None,
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
        self.disabled_cells: Set[Tuple[int, int, int]] = (
            disabled_cells if disabled_cells else set()
        )

    def get_position(
        self, coordinate: Tuple[int, int, int]
    ) -> Tuple[float, float, float]:
        """
        Computes and returns the physical position of a cell given its coordinate.

        Parameters:
        - coordinate: A tuple (x, y, z) representing the cell coordinate.

        Returns:
        - position: A tuple (px, py, pz) representing the physical position of the cell.
        """
        x, y, z = coordinate
        if not self._is_valid_coordinate(coordinate):
            raise ValueError(f"Coordinate {coordinate} is out of bounds.")
        px = x * self.dx
        py = y * self.dy
        pz = z * self.dz
        return (px, py, pz)

    def get_neighbors(
        self, coordinate: Tuple[int, int, int]
    ) -> Generator[Tuple[int, int, int], None, None]:
        """
        Generates the coordinates of adjacent cells to the given coordinate.

        Parameters:
        - coordinate: A tuple (x, y, z) representing the current cell.

        Yields:
        - neighbor_coord: A coordinate tuple representing an adjacent cell.
        """
        x, y, z = coordinate
        neighbor_offsets: List[Tuple[int, int, int]] = [
            (1, 0, 0),
            (-1, 0, 0),  # Adjacent cells along the x-axis
            (0, 1, 0),
            (0, -1, 0),  # Adjacent cells along the y-axis
            (0, 0, 1),
            (0, 0, -1),  # Adjacent cells along the z-axis
        ]
        for dx, dy, dz in neighbor_offsets:
            nx, ny, nz = x + dx, y + dy, z + dz
            neighbor_coord: Tuple[int, int, int] = (nx, ny, nz)
            if (
                self._is_valid_coordinate(neighbor_coord)
                and neighbor_coord not in self.disabled_cells
            ):
                yield neighbor_coord

    def disable_cells(self, cells_to_disable: Set[Tuple[int, int, int]]) -> None:
        """
        Disables a subset of cells in the lattice.

        Parameters:
        - cells_to_disable: A set of coordinate tuples to disable.
        """
        for coord in cells_to_disable:
            if not self._is_valid_coordinate(coord):
                raise ValueError(f"Coordinate {coord} is out of bounds.")
            self.disabled_cells.add(coord)

    def enable_cells(self, cells_to_enable: Set[Tuple[int, int, int]]) -> None:
        """
        Enables a subset of cells in the lattice.

        Parameters:
        - cells_to_enable: A set of coordinate tuples to enable.
        """
        for coord in cells_to_enable:
            self.disabled_cells.discard(coord)

    def _is_valid_coordinate(self, coord: Tuple[int, int, int]) -> bool:
        """
        Checks if the coordinate is within the lattice boundaries.

        Parameters:
        - coord: A tuple (x, y, z) representing a coordinate.

        Returns:
        - True if the coordinate is within the lattice boundaries, False otherwise.
        """
        x, y, z = coord
        return (0 <= x < self.nx) and (0 <= y < self.ny) and (0 <= z < self.nz)
