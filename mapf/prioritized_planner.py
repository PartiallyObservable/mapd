from pathfinding.safe_interval_planner import *
from common.graph import *


class PrioritizedPlanner:
    def __init__(self, graph: Graph) -> None:
        """
        Initializes the PrioritizedPlanning planner.

        Parameters:
        - graph: The Graph representing the environment.
        """
        self.graph = graph

    def plan(
        self,
        agent_starts: List[Tuple[int, int, int]],
        agent_goals: List[Tuple[int, int, int]],
        initial_safe_intervals: Optional[List[SafeIntervalTable]] = None,
    ) -> Optional[List[Trajectory]]:
        """
        Plans paths for multiple agents using priority-based planning.

        Parameters:
        - agent_starts: A list of starting coordinates for each agent.
        - agent_goals: A list of goal coordinates for each agent.
        - initial_safe_intervals: Optional list of SafeIntervalTable instances for each agent.

        Returns:
        - A list of Trajectory objects for each agent, or None if planning fails.
        """
        num_agents = len(agent_starts)
        if len(agent_goals) != num_agents:
            raise ValueError("The number of start and goal locations must be equal.")

        trajectories = []
        # Initialize a global safe interval table for all agents
        global_safe_interval_table = SafeIntervalTable(self.graph)

        for agent_idx in range(num_agents):
            start = agent_starts[agent_idx]
            goal = agent_goals[agent_idx]

            # Create a copy of the safe interval table for the current agent
            agent_safe_interval_table = SafeIntervalTable(self.graph)
            # Copy the safe intervals from the global table
            agent_safe_interval_table.safe_intervals = {
                coord: [
                    TimeInterval(interval.start, interval.end) for interval in intervals
                ]
                for coord, intervals in global_safe_interval_table.safe_intervals.items()
            }

            # Initialize the planner for the current agent
            planner = SafeIntervalPlanner(self.graph, agent_safe_interval_table)
            trajectory = planner.plan(start, goal)

            if trajectory is None:
                print(f"Planning failed for agent {agent_idx}.")
                return None  # Planning failed

            # Add the agent's trajectory to the list
            trajectories.append(trajectory)

            # Update the global safe interval table with the agent's trajectory
            self._add_trajectory_constraints(global_safe_interval_table, trajectory)

        return trajectories

    def _add_trajectory_constraints(
        self, safe_interval_table: SafeIntervalTable, trajectory: Trajectory
    ) -> None:
        """
        Adds the trajectory's occupied times and locations as constraints to the safe interval table.

        Parameters:
        - safe_interval_table: The global SafeIntervalTable to be updated.
        - trajectory: The Trajectory object of the agent.
        """
        for segment in trajectory:
            start_coord, end_coord, start_time, end_time = segment

            # Add constraints for the start and end coordinates
            # Since movement is instantaneous in discrete time steps, we can block both positions
            safe_interval_table.add_node_constraint(start_coord, start_time, end_time)
            safe_interval_table.add_node_constraint(end_coord, start_time, end_time)

            # Additionally, to prevent edge conflicts, we can block the edge between the coordinates
            # This is optional and depends on how strictly you want to prevent conflicts
            # For edge conflicts, you might need to block the movement in reverse for other agents
            # This can be implemented if needed

        # Note: This simplistic approach may not handle all conflict types (e.g., edge collisions)
        # For a more comprehensive conflict avoidance, you may need to consider edge constraints
