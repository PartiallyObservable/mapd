import pytest
import numpy as np
from mapd.assignment_algorithms import *


# Fixture to create cost matrix, get_cost function, and print_assignments function
@pytest.fixture
def assignment_data():
    agents = ["Agent A", "Agent B", "Agent C"]
    tasks = ["Task 1", "Task 2", "Task 3"]
    cost_matrix = np.array([[9, 2, 7], [6, 4, 3], [5, 8, 1]])

    def get_cost(agent, task):
        agent_index = agents.index(agent)
        task_index = tasks.index(task)
        return cost_matrix[agent_index, task_index]

    def print_assignments(assignments):
        all_costs = [assignments[k][1] for k in assignments]
        print(f"Assignments:\n {assignments}")
        print(f"Sum-of-costs: {sum(all_costs)}")
        print(f"Max-cost: {max(all_costs)}")

    return agents, tasks, cost_matrix, get_cost, print_assignments


# Test for greedy assignment solver
def test_greedy_assignment(assignment_data):
    agents, tasks, cost_matrix, get_cost, print_assignments = assignment_data
    greedy_assignments = greedy_assignment_solver(agents, tasks, get_cost)
    assert greedy_assignments is not None
    print("Greedy Assignment:")
    print_assignments(greedy_assignments)


# Test for linear sum assignment solver
def test_linear_sum_assignment(assignment_data):
    agents, tasks, cost_matrix, get_cost, print_assignments = assignment_data
    sum_assignments = linear_sum_assignment_solver(agents, tasks, get_cost)
    assert sum_assignments is not None
    print("Linear Sum Assignment:")
    print_assignments(sum_assignments)


# Test for bottleneck assignment solver
def test_bottleneck_assignment(assignment_data):
    agents, tasks, cost_matrix, get_cost, print_assignments = assignment_data
    bottleneck_assignments = bottleneck_assignment_solver(agents, tasks, get_cost)
    assert bottleneck_assignments is not None
    print("Bottleneck Assignment:")
    print_assignments(bottleneck_assignments)


if __name__ == "__main__":
    pytest.main(["-s", __file__])
