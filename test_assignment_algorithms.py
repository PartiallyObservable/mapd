from assignment_algorithms import *

if __name__ == "__main__":

    # Named agents and tasks
    agents = ["Agent A", "Agent B", "Agent C"]
    tasks = ["Task 1", "Task 2", "Task 3"]

    # Predefined cost matrix where the expected results of linear sum and bottleneck assignments are different
    # get_cost will return values from this matrix (in pathfinding get_cost would return path cost)
    cost_matrix = np.array([[9, 2, 7], [6, 4, 3], [5, 8, 1]])

    def get_cost(agent, task):
        agent_index = agents.index(agent)
        task_index = tasks.index(task)
        return cost_matrix[agent_index, task_index]

    # Helper function to print info
    def print_assignments(assignments):
        all_costs = [assignments[k][1] for k in assignments]
        print(f"Assignments:\n {assignments}")
        print(f"Sum-of-costs: {sum(all_costs)}")
        print(f"Max-cost: {max(all_costs)}")

    print("Cost Matrix:")
    print(cost_matrix)

    print("Greedy Assignment:")
    greedy_assignments = greedy_assignment_solver(agents, tasks, get_cost)
    print_assignments(greedy_assignments)

    print("Linear Sum Assignment:")
    sum_assignments = linear_sum_assignment_solver(agents, tasks, get_cost)
    print_assignments(sum_assignments)

    print("Bottleneck Assignment:")
    bottleneck_assignments = bottleneck_assignment_solver(agents, tasks, get_cost)
    print_assignments(bottleneck_assignments)
