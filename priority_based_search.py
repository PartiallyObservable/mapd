import heapq
from typing import Tuple, List, Dict, Optional
from safe_interval_planner import *


class PTNode:
    def __init__(
        self,
        priorities: List[int],
        constraints: Dict[int, List[Tuple]],
        solution: Dict[int, Trajectory],
        cost: float,
    ) -> None:
        """
        Initializes a node in the priority tree.

        Parameters:
        - priorities: A list of agent indices representing their priorities.
        - constraints: A dictionary mapping agent indices to their constraints.
        - solution: A dictionary mapping agent indices to their trajectories.
        - cost: The total cost of the solution.
        """
        self.priorities = priorities  # Agent indices in priority order
        self.constraints = constraints  # Constraints for each agent
        self.solution = solution  # Paths for each agent
        self.cost = cost  # Total cost of all agents' paths

    def __lt__(self, other: "PTNode") -> bool:
        return self.cost < other.cost


class PriorityBasedSearch:
    def __init__(self, graph: MatrixGraph) -> None:
        """
        Initializes the Priority-Based Search planner.

        Parameters:
        - graph: The MatrixGraph representing the environment.
        """
        self.graph = graph

    def plan(
        self,
        agent_starts: List[Tuple[int, int, int]],
        agent_goals: List[Tuple[int, int, int]],
    ) -> Optional[List[Trajectory]]:
        """
        Plans paths for multiple agents using Priority-Based Search.

        Parameters:
        - agent_starts: A list of starting coordinates for each agent.
        - agent_goals: A list of goal coordinates for each agent.

        Returns:
        - A list of Trajectory objects for each agent, or None if planning fails.
        """
        num_agents = len(agent_starts)
        if len(agent_goals) != num_agents:
            raise ValueError("The number of start and goal locations must be equal.")

        # Initial priorities (agent indices)
        priorities = list(range(num_agents))  # Fixed priority order

        # Root node with no constraints
        root_constraints = {agent_idx: [] for agent_idx in range(num_agents)}
        root_solution = {}
        root_cost = 0.0

        # Plan initial paths for all agents
        for agent_idx in priorities:
            trajectory = self.plan_for_agent(
                agent_idx,
                agent_starts[agent_idx],
                agent_goals[agent_idx],
                root_constraints[agent_idx],
            )
            if trajectory is None:
                print(f"Initial planning failed for agent {agent_idx}.")
                return None  # Planning failed
            root_solution[agent_idx] = trajectory
            root_cost += self.compute_cost(trajectory)

        # Initialize the priority queue with the root node
        open_list = []
        root_node = PTNode(
            priorities=priorities,
            constraints=root_constraints,
            solution=root_solution,
            cost=root_cost,
        )
        heapq.heappush(open_list, root_node)

        while open_list:
            current_node = heapq.heappop(open_list)

            # Check for conflicts
            conflict = self.detect_conflict(current_node.solution)
            if conflict is None:
                # No conflicts, return the solution
                return [
                    current_node.solution[agent_idx] for agent_idx in range(num_agents)
                ]

            # Get conflicting agents and the conflict details
            agent_i, agent_j, conflict_details = conflict
            print(
                f"Conflict detected between agent {agent_i} and agent {agent_j} at {conflict_details}"
            )

            # Determine which agent has higher priority
            priority_i = current_node.priorities.index(agent_i)
            priority_j = current_node.priorities.index(agent_j)
            if priority_i < priority_j:
                # Agent i has higher priority
                agent_high = agent_i
                agent_low = agent_j
            else:
                # Agent j has higher priority
                agent_high = agent_j
                agent_low = agent_i

            # Create new constraints for the lower-priority agent
            child_constraints = {
                agent_idx: constraints.copy()
                for agent_idx, constraints in current_node.constraints.items()
            }
            # Add the conflict constraint to the lower-priority agent
            if agent_low not in child_constraints:
                child_constraints[agent_low] = []
            child_constraints[agent_low].append(conflict_details)

            # Re-plan path for the lower-priority agent with new constraints
            child_solution = current_node.solution.copy()
            agent_start = agent_starts[agent_low]
            agent_goal = agent_goals[agent_low]
            trajectory = self.plan_for_agent(
                agent_low, agent_start, agent_goal, child_constraints[agent_low]
            )
            if trajectory is None:
                # Cannot find a path for the lower-priority agent, skip this node
                continue

            child_solution[agent_low] = trajectory
            # Update cost
            child_cost = sum(
                self.compute_cost(child_solution[agent_idx])
                for agent_idx in range(num_agents)
            )

            child_node = PTNode(
                priorities=current_node.priorities,  # Keep priorities consistent
                constraints=child_constraints,
                solution=child_solution,
                cost=child_cost,
            )
            heapq.heappush(open_list, child_node)

        print("Planning failed to find a conflict-free solution.")
        return None

    def plan_for_agent(
        self,
        agent_idx: int,
        start: Tuple[int, int, int],
        goal: Tuple[int, int, int],
        constraints: List[Tuple],
    ) -> Optional[Trajectory]:
        """
        Plans a path for a single agent considering its constraints.

        Parameters:
        - agent_idx: The index of the agent.
        - start: The starting coordinate of the agent.
        - goal: The goal coordinate of the agent.
        - constraints: A list of constraints for the agent.

        Returns:
        - A Trajectory object for the agent, or None if planning fails.
        """
        # Initialize the SafeIntervalTable with constraints
        safe_interval_table = SafeIntervalTable(self.graph)
        # Apply constraints to the safe intervals
        for constraint in constraints:
            coord, time_start, time_end = constraint
            safe_interval_table.add_constraint(coord, time_start, time_end)

        # Plan the path using SafeIntervalPlanner
        planner = SafeIntervalPlanner(self.graph, safe_interval_table)
        trajectory = planner.plan(start, goal)
        return trajectory

    def detect_conflict(
        self, solution: Dict[int, Trajectory]
    ) -> Optional[Tuple[int, int, Tuple]]:
        """
        Detects the first conflict between agents in the solution.

        Parameters:
        - solution: A dictionary mapping agent indices to their trajectories.

        Returns:
        - A tuple (agent_i, agent_j, conflict_details), or None if no conflict is found.
        """
        # Collect all movements with their time intervals
        agent_movements = {}
        for agent_idx, trajectory in solution.items():
            movements = []
            for segment in trajectory:
                start_coord, end_coord, start_time, end_time = segment
                movements.append((start_coord, end_coord, start_time, end_time))
            agent_movements[agent_idx] = movements

        # Check for conflicts between agents
        agents = list(solution.keys())
        for i in range(len(agents)):
            agent_i = agents[i]
            for j in range(i + 1, len(agents)):
                agent_j = agents[j]
                conflicts = self.check_conflicts_between_agents(
                    agent_movements[agent_i], agent_movements[agent_j]
                )
                if conflicts:
                    # Return the first conflict detected
                    conflict = conflicts[0]
                    return agent_i, agent_j, conflict
        return None  # No conflict found

    def check_conflicts_between_agents(
        self, movements_i: List[Tuple], movements_j: List[Tuple]
    ) -> List[Tuple]:
        """
        Checks for conflicts between two agents' movements.

        Parameters:
        - movements_i: Movements of agent i.
        - movements_j: Movements of agent j.

        Returns:
        - A list of conflict details (coord, time_start, time_end).
        """
        conflicts = []
        for move_i in movements_i:
            start_i, end_i, t_start_i, t_end_i = move_i
            for move_j in movements_j:
                start_j, end_j, t_start_j, t_end_j = move_j

                # Check for vertex conflict at the end positions
                if (end_i == end_j) and self.time_overlap(
                    t_start_i, t_end_i, t_start_j, t_end_j
                ):
                    conflict = (end_i, max(t_start_i, t_start_j), min(t_end_i, t_end_j))
                    conflicts.append(conflict)
                    return conflicts  # Return immediately for PBS

                # Check for edge conflict (swapping positions)
                if (start_i == end_j and start_j == end_i) and self.time_overlap(
                    t_start_i, t_end_i, t_start_j, t_end_j
                ):
                    conflict_time = max(t_start_i, t_start_j)
                    conflict = (
                        end_i,
                        conflict_time,
                        conflict_time + self._small_time_epsilon(),
                    )
                    conflicts.append(conflict)
                    return conflicts  # Return immediately for PBS
        return conflicts

    def time_overlap(
        self, t_start_i: float, t_end_i: float, t_start_j: float, t_end_j: float
    ) -> bool:
        """
        Checks if two time intervals overlap.

        Parameters:
        - t_start_i, t_end_i: Time interval for agent i.
        - t_start_j, t_end_j: Time interval for agent j.

        Returns:
        - True if intervals overlap, False otherwise.
        """
        return t_start_i < t_end_j and t_start_j < t_end_i

    def compute_cost(self, trajectory: Trajectory) -> float:
        """
        Computes the cost of a trajectory.

        Parameters:
        - trajectory: The Trajectory object.

        Returns:
        - The total time of the trajectory.
        """
        if not trajectory.segments:
            return 0.0
        return trajectory.segments[-1][3]  # End time of the last segment

    def _small_time_epsilon(self) -> float:
        """
        Returns a small epsilon value to adjust time intervals for conflict avoidance.

        Returns:
        - A small float value.
        """
        return 1e-5  # Adjust as needed
