import pytest
from common.matrix_graph import MatrixGraph
from pathfinding.safe_interval_planner import *
from common.time_intervals import TimeInterval

def test_safe_interval_planner():
    # Create a MatrixGraph
    graph = MatrixGraph(dimensions=(10, 10, 5))

    # Initialize the SafeIntervalTable
    safe_interval_table = SafeIntervalTable(graph)

    # Start and goal locations
    start = (0, 0, 0)
    goal = (2, 0, 0)

    # Add constraints to simulate other agents or obstacles
    safe_interval_table.add_node_constraint((1, 0, 0), TimeInterval(0.0,2.0))
    safe_interval_table.add_node_constraint((0, 1, 0), TimeInterval(0.0,2.0))
    safe_interval_table.add_node_constraint((0, 0, 1), TimeInterval(0.0,12.0))
    safe_interval_table.add_node_constraint((2, 0, 0), TimeInterval(0.0,5.0))

    # Initialize the planner
    planner = SafeIntervalPlanner(graph, safe_interval_table)
    trajectory = planner.plan(start, goal)

    if trajectory:
        print("Found trajectory:")
        for segment in trajectory:
            start_coord, end_coord, start_time, end_time = segment
            print(
                f"From {start_coord} to {end_coord}, time {start_time:.2f} to {end_time:.2f}"
            )
    else:
        print("No path found.")


if __name__ == "__main__":
    pytest.main(["-s", __file__])
