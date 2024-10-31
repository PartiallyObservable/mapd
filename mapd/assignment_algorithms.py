from scipy.optimize import linear_sum_assignment
import numpy as np
from typing import Callable, Any, List, Tuple


def linear_sum_assignment_solver(
    agents: List[Any], tasks: List[Any], get_cost: Callable[[Any, Any], float]
) -> Tuple[dict[Any, tuple[Any, float]]]:
    # Create a cost matrix of size len(agents) x len(tasks)
    cost_matrix = np.zeros((len(agents), len(tasks)))

    # Fill in the cost matrix using the provided get_cost function
    for i, agent in enumerate(agents):
        for j, task in enumerate(tasks):
            cost_matrix[i, j] = get_cost(agent, task)

    # Use the linear_sum_assignment function from SciPy to solve the assignment problem
    row_ind, col_ind = linear_sum_assignment(cost_matrix)

    # Create a dictionary of assignments and a dictionary for assignment costs
    assignments = {}
    for i, j in zip(row_ind, col_ind):
        assignments[agents[i]] = (tasks[j], cost_matrix[i, j])

    return assignments


def bottleneck_assignment_solver(
    agents: List[Any], tasks: List[Any], get_cost: Callable[[Any, Any], float]
) -> Tuple[dict[Any, tuple[Any, float]]]:
    # Create a cost matrix of size len(agents) x len(tasks)
    cost_matrix = np.zeros((len(agents), len(tasks)))

    # Fill in the cost matrix using the provided get_cost function
    for i, agent in enumerate(agents):
        for j, task in enumerate(tasks):
            cost_matrix[i, j] = get_cost(agent, task)

    # Use the threshold algorithm to minimize the maximum cost
    lower, upper = cost_matrix.min(), cost_matrix.max()
    best_assignment = None

    while lower <= upper:
        mid = (lower + upper) / 2
        # Create a modified cost matrix where values greater than mid are set to infinity
        threshold_cost_matrix = np.where(cost_matrix <= mid, cost_matrix, np.inf)

        # Try to find a feasible assignment within the threshold using linear_sum_assignment
        try:
            row_ind, col_ind = linear_sum_assignment(threshold_cost_matrix)
            if len(row_ind) == len(agents):
                best_assignment = (row_ind, col_ind)
                upper = mid - 1
            else:
                lower = mid + 1
        except ValueError:
            lower = mid + 1

    # Create a dictionary of assignments and a dictionary for assignment costs
    assignments = {}
    if best_assignment:
        row_ind, col_ind = best_assignment
        for i, j in zip(row_ind, col_ind):
            assignments[agents[i]] = (tasks[j], cost_matrix[i, j])

    return assignments


def greedy_assignment_solver(
    agents: List[Any], tasks: List[Any], get_cost: Callable[[Any, Any], float]
) -> Tuple[dict[Any, tuple[Any, float]]]:
    # Create a cost matrix of size len(agents) x len(tasks)
    cost_matrix = np.zeros((len(agents), len(tasks)))

    # Fill in the cost matrix using the provided get_cost function
    for i, agent in enumerate(agents):
        for j, task in enumerate(tasks):
            cost_matrix[i, j] = get_cost(agent, task)

    # Keep track of assigned agents and tasks
    assigned_agents = set()
    assigned_tasks = set()
    assignments = {}

    # Flatten the cost matrix and sort by cost
    cost_entries = [
        (i, j, cost_matrix[i, j]) for i in range(len(agents)) for j in range(len(tasks))
    ]
    cost_entries.sort(key=lambda x: x[2])

    # Iteratively select the next lowest cost and assign the agent and task
    for i, j, cost in cost_entries:
        if i not in assigned_agents and j not in assigned_tasks:
            agent = agents[i]
            task = tasks[j]
            assignments[agent] = (task, cost)
            assigned_agents.add(i)
            assigned_tasks.add(j)

            # Stop if all agents and tasks are assigned
            if len(assigned_agents) == len(agents):
                break

    return assignments
