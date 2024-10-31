import pytest
from priority_based_search import *


def test_priority_based_search():
    graph = MatrixGraph(dimensions=(3, 3, 3))

    """
    Simple Crossing scenario a->x, b->y
      x
    b + y
      a
    """
    agent_starts = [
        (1, 0, 0),  # a
        (0, 1, 0),  # b
    ]

    agent_goals = [
        (1, 2, 0),  # x
        (2, 1, 0),  # y
    ]

    # Initialize the prioritized planner
    planner = PriorityBasedSearch(graph)

    # Plan paths for all agents
    trajectories = planner.plan(agent_starts, agent_goals)
    assert trajectories is not None

    """
    Should have one agent wait for the other at its initial location before entering intersection point
    """
    if trajectories:
        for idx, trajectory in enumerate(trajectories):
            print(f"Agent {idx} trajectory:")
            print(trajectory)
    else:
        print("Planning failed for one or more agents.")


if __name__ == "__main__":
    pytest.main(["-s", __file__])
