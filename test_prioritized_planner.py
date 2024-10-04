from prioritized_planner import *

# Create a MatrixGraph
graph = MatrixGraph(dimensions=(3, 3, 3))

"""
Crossing scenario a->x, b->y, c->z
  x z
b   y
c a
"""
agent_starts = [
    (1, 0, 0),  # a
    (0, 1, 0),  # b
    (0, 0, 0),  # c
]

agent_goals = [
    (1, 2, 0),  # x
    (2, 1, 0),  # y
    (2, 2, 0),  # z
]

# Initialize the prioritized planner
planner = PrioritizedPlanner(graph)

# Plan paths for all agents
trajectories = planner.plan(agent_starts, agent_goals)

"""
Should have one agent wait for the other at its initial location before entering intersection point
"""
if trajectories:
    for idx, trajectory in enumerate(trajectories):
        print(f"Agent {idx} trajectory:")
        print(trajectory)
else:
    print("Planning failed for one or more agents.")
