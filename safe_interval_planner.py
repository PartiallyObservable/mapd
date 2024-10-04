from typing import Tuple, List, Dict, Optional
import heapq
from matrix_graph import MatrixGraph

# Define movement velocities (meters per second)
BOT_VXY = 1.0  # Velocity in x or y direction
BOT_VZUP = 0.3  # Velocity moving upwards in z
BOT_VZDOWN = 0.6  # Velocity moving downwards in z


class TimeInterval:
    def __init__(self, start: float, end: float) -> None:
        self.start = start
        self.end = end

    def overlaps(self, other: "TimeInterval") -> bool:
        return not (self.end < other.start or self.start > other.end)

    def __repr__(self):
        return f"[{self.start}, {self.end}]"


class SafeIntervalTable:
    def __init__(self, graph: "MatrixGraph") -> None:
        """
        Initializes the Safe Interval Table for the given graph.

        Parameters:
        - graph: The MatrixGraph representing the warehouse environment.
        """
        self.graph = graph
        # For each node, maintain a list of safe intervals
        self.safe_intervals: Dict[Tuple[int, int, int], List[TimeInterval]] = {}
        self.initialize_safe_intervals()

    def initialize_safe_intervals(self) -> None:
        """
        Initializes the safe intervals for all nodes in the graph.
        By default, all nodes are safe from time 0 to infinity unless otherwise specified.
        """
        for x in range(self.graph.nx):
            for y in range(self.graph.ny):
                for z in range(self.graph.nz):
                    coord = (x, y, z)
                    if coord not in self.graph.disabled_cells:
                        # Initially, all cells are safe from time 0 to infinity
                        self.safe_intervals[coord] = [TimeInterval(0.0, float("inf"))]

    def get_safe_intervals(
        self, coordinate: Tuple[int, int, int]
    ) -> List[TimeInterval]:
        """
        Retrieves the list of safe intervals for a given coordinate.

        Parameters:
        - coordinate: The coordinate tuple (x, y, z).

        Returns:
        - A list of SafeInterval objects for the given coordinate.
        """
        return self.safe_intervals.get(coordinate, [])

    def set_safe_intervals(
        self, coordinate: Tuple[int, int, int], intervals: List[TimeInterval]
    ) -> None:
        """
        Sets the safe intervals for a given coordinate.

        Parameters:
        - coordinate: The coordinate of the cell.
        - intervals: A list of SafeInterval objects to set for this coordinate.
        """
        self.safe_intervals[coordinate] = intervals

    def add_constraint(
        self, coordinate: Tuple[int, int, int], start_time: float, end_time: float
    ) -> None:
        """
        Adds a constraint to the safe interval table, updating the safe intervals.

        Parameters:
        - coordinate: The coordinate of the constraint.
        - start_time: The time when the constraint starts.
        - end_time: The time when the constraint ends.
        """
        intervals = self.safe_intervals.get(
            coordinate, [TimeInterval(0.0, float("inf"))]
        )
        new_intervals = []
        for interval in intervals:
            if interval.end <= start_time or interval.start >= end_time:
                # No overlap
                new_intervals.append(interval)
            else:
                # Overlap exists, split intervals if necessary
                if interval.start < start_time:
                    new_intervals.append(TimeInterval(interval.start, start_time))
                if interval.end > end_time:
                    new_intervals.append(TimeInterval(end_time, interval.end))
        self.safe_intervals[coordinate] = new_intervals


class Trajectory:
    def __init__(
        self,
        segments: List[Tuple[Tuple[int, int, int], Tuple[int, int, int], float, float]],
    ) -> None:
        """
        Initializes the Trajectory with a list of segments.

        Each segment is a tuple:
        - start_coord: Tuple[int, int, int]
        - end_coord: Tuple[int, int, int]
        - start_time: float
        - end_time: float
        """
        self.segments = segments

    def __iter__(self):
        """
        Iterates over the segments in the trajectory.
        """
        return iter(self.segments)

    def iter_time_range(self, start_time: float, end_time: float):
        """
        Generator that yields segments overlapping with the given time interval.

        Parameters:
        - start_time: The start of the time interval.
        - end_time: The end of the time interval.
        """
        for segment in self.segments:
            seg_start_time = segment[2]
            seg_end_time = segment[3]
            # Check for overlap
            if seg_end_time > start_time and seg_start_time < end_time:
                yield segment

    def __repr__(self):
        rep = [
            f"{s[0]} to {s[1]} during [{s[2]:.2f}, {s[3]:.2f}] " for s in self.segments
        ]
        return "\n".join(rep)


class SafeIntervalPlanner:
    class State:
        def __init__(
            self,
            coord: Tuple[int, int, int],
            time: float,
            interval: TimeInterval,
            g: float,
            parent: Optional["SafeIntervalPlanner.State"] = None,
        ) -> None:
            self.coord = coord
            self.time = time
            self.interval = interval
            self.g = g  # Cost to come (total time to reach this state)
            self.parent = parent

        def __lt__(self, other: "SafeIntervalPlanner.State") -> bool:
            # Comparison operator for priority queue (min-heap)
            return self.g < other.g

        def __repr__(self):
            return f"State(coord={self.coord}, time={self.time}, interval={self.interval}, g={self.g})"

    def __init__(
        self, graph: "MatrixGraph", safe_interval_table: SafeIntervalTable
    ) -> None:
        """
        Initializes the Safe Interval Planner.

        Parameters:
        - graph: The MatrixGraph representing the warehouse environment.
        - safe_interval_table: The SafeIntervalTable containing safe intervals for each node.
        """
        self.graph = graph
        self.safe_interval_table = safe_interval_table

    def plan(
        self, start: Tuple[int, int, int], goal: Tuple[int, int, int]
    ) -> Optional["Trajectory"]:
        """
        Plans a path from start to goal using Safe Interval Path Planning.

        Parameters:
        - start: The starting coordinate.
        - goal: The goal coordinate.

        Returns:
        - A Trajectory object representing the path with timing information,
          or None if no path is found.
        """
        open_list = []
        closed_list = {}

        start_intervals = self.safe_interval_table.get_safe_intervals(start)
        if not start_intervals:
            return None  # No safe intervals at the start position

        # Initialize the start state
        for interval in start_intervals:
            start_state = self.State(
                coord=start,
                time=max(0.0, interval.start),
                interval=interval,
                g=max(0.0, interval.start),
                parent=None,
            )
            estimated_total_cost = start_state.g + self.heuristic(
                start_state.coord, goal
            )
            heapq.heappush(open_list, (estimated_total_cost, start_state))

        while open_list:
            _, current_state = heapq.heappop(open_list)

            state_key = (current_state.coord, current_state.interval.start)
            if state_key in closed_list:
                continue
            closed_list[state_key] = current_state

            if current_state.coord == goal:
                return self.construct_trajectory(current_state)

            successors = self.generate_successors(current_state, goal)
            for successor in successors:
                succ_key = (successor.coord, successor.interval.start)
                if succ_key in closed_list:
                    continue
                estimated_total_cost = successor.g + self.heuristic(
                    successor.coord, goal
                )
                heapq.heappush(open_list, (estimated_total_cost, successor))

        return None  # No path found

    def generate_successors(
        self, current_state: "SafeIntervalPlanner.State", goal: Tuple[int, int, int]
    ) -> List["SafeIntervalPlanner.State"]:
        """
        Generates successor states from the current state.

        Parameters:
        - current_state: The current state from which to generate successors.
        - goal: The goal coordinate.

        Returns:
        - A list of successor State instances.
        """
        successors = []

        # Generate successors by moving to neighboring nodes
        for neighbor in self.graph.get_neighbors(current_state.coord):
            move_duration = self.move_time(current_state.coord, neighbor)
            neighbor_intervals = self.safe_interval_table.get_safe_intervals(neighbor)

            for neighbor_interval in neighbor_intervals:
                # Earliest time we can arrive at the neighbor
                earliest_arrival = max(
                    current_state.time + move_duration, neighbor_interval.start
                )
                # Corresponding departure time
                earliest_departure = earliest_arrival - move_duration

                # Ensure departure is not before current time and within current interval
                earliest_departure = max(earliest_departure, current_state.time)
                if (
                    earliest_departure < current_state.interval.start
                    or earliest_departure > current_state.interval.end
                ):
                    continue  # Cannot depart during current interval

                # Update arrival time based on adjusted departure time
                arrival_time = earliest_departure + move_duration
                if arrival_time > neighbor_interval.end:
                    continue  # Cannot arrive during neighbor's interval

                # Valid move
                successor_state = self.State(
                    coord=neighbor,
                    time=arrival_time,
                    interval=neighbor_interval,
                    g=arrival_time,
                    parent=current_state,
                )
                successors.append(successor_state)
                break  # Found valid interval for this neighbor

        # Generate successors by waiting at the current location
        # Find the earliest time when a move to a neighbor becomes possible
        min_wait_time = None
        for neighbor in self.graph.get_neighbors(current_state.coord):
            move_duration = self.move_time(current_state.coord, neighbor)
            neighbor_intervals = self.safe_interval_table.get_safe_intervals(neighbor)
            for neighbor_interval in neighbor_intervals:
                earliest_departure = neighbor_interval.start - move_duration
                if earliest_departure > current_state.time:
                    if min_wait_time is None or earliest_departure < min_wait_time:
                        min_wait_time = earliest_departure

        if min_wait_time is not None and min_wait_time > current_state.time:
            # Generate a wait state until min_wait_time
            wait_state = self.State(
                coord=current_state.coord,
                time=min_wait_time,
                interval=current_state.interval,
                g=min_wait_time,
                parent=current_state,
            )
            successors.append(wait_state)
        elif current_state.time < current_state.interval.end:
            # Wait until the end of the current interval
            wait_state = self.State(
                coord=current_state.coord,
                time=current_state.interval.end,
                interval=current_state.interval,
                g=current_state.interval.end,
                parent=current_state,
            )
            successors.append(wait_state)
        else:
            # Move to the next safe interval at current location
            current_intervals = self.safe_interval_table.get_safe_intervals(
                current_state.coord
            )
            for interval in current_intervals:
                if interval.start > current_state.time:
                    wait_state = self.State(
                        coord=current_state.coord,
                        time=interval.start,
                        interval=interval,
                        g=interval.start,
                        parent=current_state,
                    )
                    successors.append(wait_state)
                    break  # Only need to add one wait state

        return successors

    def can_wait_at(
        self, coord: Tuple[int, int, int], start_time: float, end_time: float
    ) -> bool:
        """
        Checks if the agent can wait at the current coordinate from start_time to end_time.

        Parameters:
        - coord: The coordinate where the agent wants to wait.
        - start_time: The start time of the waiting period.
        - end_time: The end time of the waiting period.

        Returns:
        - True if the agent can wait during this time, False otherwise.
        """
        intervals = self.safe_interval_table.get_safe_intervals(coord)
        for interval in intervals:
            if interval.start <= start_time and interval.end >= end_time:
                return True
        return False

    def heuristic(
        self, coord1: Tuple[int, int, int], coord2: Tuple[int, int, int]
    ) -> float:
        """
        Heuristic function for A* search.

        Parameters:
        - coord1: The first coordinate.
        - coord2: The second coordinate.

        Returns:
        - The estimated minimal time between coord1 and coord2.
        """
        x1, y1, z1 = coord1
        x2, y2, z2 = coord2
        dx = abs(x1 - x2) * self.graph.dx
        dy = abs(y1 - y2) * self.graph.dy
        dz = abs(z1 - z2) * self.graph.dz

        # Time to move in x and y directions
        time_xy = (dx + dy) / BOT_VXY

        # Time to move in z direction
        if z2 > z1:
            # Moving upwards
            time_z = dz / BOT_VZUP
        elif z2 < z1:
            # Moving downwards
            time_z = dz / BOT_VZDOWN
        else:
            time_z = 0.0  # Same z-level

        return time_xy + time_z

    def move_time(
        self, coord1: Tuple[int, int, int], coord2: Tuple[int, int, int]
    ) -> float:
        """
        Computes the time to move from coord1 to coord2.

        Parameters:
        - coord1: The starting coordinate.
        - coord2: The destination coordinate.

        Returns:
        - The movement time.
        """
        x1, y1, z1 = coord1
        x2, y2, z2 = coord2

        # Check for diagonal movement (not allowed)
        moves = sum([abs(x1 - x2), abs(y1 - y2), abs(z1 - z2)])
        if moves != 1:
            raise ValueError("Diagonal or zero movement is not allowed")

        if x1 != x2:
            # Movement in x-direction
            distance = self.graph.dx
            time = distance / BOT_VXY
        elif y1 != y2:
            # Movement in y-direction
            distance = self.graph.dy
            time = distance / BOT_VXY
        elif z1 != z2:
            # Movement in z-direction
            distance = self.graph.dz
            if z2 > z1:
                # Moving upwards
                time = distance / BOT_VZUP
            else:
                # Moving downwards
                time = distance / BOT_VZDOWN
        else:
            raise ValueError("No movement detected")

        return time

    def construct_trajectory(self, state: "SafeIntervalPlanner.State") -> "Trajectory":
        """
        Constructs the trajectory from the goal state back to the start.

        Parameters:
        - state: The goal state.

        Returns:
        - A Trajectory object representing the path with timing information.
        """
        segments = []
        while state.parent is not None:
            if state.coord == state.parent.coord:
                # Waiting at the same location
                segment = (
                    state.coord,  # start_coord (same as end_coord)
                    state.coord,  # end_coord
                    state.parent.time,  # start_time
                    state.time,  # end_time
                )
                segments.append(segment)
            else:
                # Moving between different locations
                move_duration = self.move_time(state.parent.coord, state.coord)
                expected_arrival_time = state.parent.time + move_duration
                if expected_arrival_time < state.time:
                    # There was waiting before moving
                    # Add waiting segment first
                    # waiting_segment = (
                    #     state.parent.coord,     # start_coord
                    #     state.parent.coord,     # end_coord
                    #     state.parent.time,      # start_time
                    #     state.time - move_duration  # end_time
                    # )
                    # segments.append(waiting_segment)
                    # Then add movement segment
                    movement_segment = (
                        state.parent.coord,  # start_coord
                        state.coord,  # end_coord
                        state.time - move_duration,  # start_time
                        state.time,  # end_time
                    )
                    segments.append(movement_segment)

                    waiting_segment = (
                        state.parent.coord,  # start_coord
                        state.parent.coord,  # end_coord
                        state.parent.time,  # start_time
                        state.time - move_duration,  # end_time
                    )
                    segments.append(waiting_segment)
                else:
                    # No waiting, direct movement
                    segment = (
                        state.parent.coord,  # start_coord
                        state.coord,  # end_coord
                        state.parent.time,  # start_time
                        state.time,  # end_time
                    )
                    segments.append(segment)
            state = state.parent
        segments.reverse()
        return Trajectory(segments)