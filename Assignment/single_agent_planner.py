"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx

def calc_heuristics(graph, nodes_dict):
    """
    Calculates the exact heuristic dict (shortest distance between two nodes) to be used in A* search.
    INPUT:
        - graph = networkX graph object
        - nodes_dict = dictionary with nodes and node properties
    RETURNS:
        - heuristics = dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
    """
    
    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path == False:
                pass
            else:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics

def heuristicFinder(graph, start_node, goal_node):
    """
    Finds exact distance between start_node and goal_node using the NetworkX graph.
    INPUT:
        - graph = networkX graph object
        - start_node, goal_node [int] = node_ids of start and goal node
    RETURNS:
        - path = list with node_ids that are part of the shortest path
        - path_length = length of the shortest path
    """
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        path = False
        path_length = False
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def build_constraint_table(constraints, agent):
    ##############################
    # Task 2.2/2.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    # max_timestep = -1  # the maximum timestep in these constraints
    # #  collect constraints that are related to this agent
    # for constraint in constraints:
    #     if constraint['agent'] == agent:
    #         max_timestep = max(max_timestep, constraint['timestep'])

    # constraint_table = [[] for _ in range(max_timestep + 1)]

    # for constraint in constraints:
    #     if constraint['agent'] == agent:
    #         constraint_table[constraint['timestep']].append({'loc': constraint['loc']})

    # return constraint_table

    positive = []  # to collect positive constraints
    negative = []  # to collect negative constraints
    max_timestep = -1  # the maximum timestep in these constraints
    #  collect constraints that are related to this agent
    for constraint in constraints:
        if constraint['positive']:  # positive constraint is effective for everyone
            if constraint['agent'] == agent:
                positive.append(constraint)
            else:
                negative.append(constraint)
            max_timestep = max(max_timestep, constraint['timestep'])
        elif constraint['agent'] == agent:  # negative constraint is effective for only one agent
            negative.append(constraint)
            max_timestep = max(max_timestep, constraint['timestep'])

    constraint_table = [[] for _ in range(max_timestep + 1)]
    for constraint in positive:
        if len(constraint['loc']) == 1:  # positive vertex constraint
            constraint_table[constraint['timestep']].append({'loc': constraint['loc'], 'positive': True})
        else:  # positive edge constraint
            constraint_table[constraint['timestep'] - 1].append({'loc': [constraint['loc'][0]], 'positive': True})
            constraint_table[constraint['timestep']].append({'loc': [constraint['loc'][1]], 'positive': True})

    for constraint in negative:
        if len(constraint['loc']) == 1:  # vertex constraint
            constraint_table[constraint['timestep']].append({'loc': constraint['loc'], 'positive': False})
        elif constraint['positive']:  # positive edge constraint for other agents
            constraint_table[constraint['timestep'] - 1].append({'loc': [constraint['loc'][0]], 'positive': False})
            constraint_table[constraint['timestep']].append({'loc': [constraint['loc'][1]], 'positive': False})
            constraint_table[constraint['timestep']].append(
                {'loc': [constraint['loc'][1], constraint['loc'][0]], 'positive': False})
        else:  # negative edge constraint
            constraint_table[constraint['timestep']].append({'loc': constraint['loc'], 'positive': False})

    return constraint_table

def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 2.2/2.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    # if len(constraint_table) <= next_time:
    #     return False

    # for constraint in constraint_table[next_time]:
    #     if len(constraint['loc']) == 1:  # vertex constraint
    #         if constraint['loc'][0] == next_loc:
    #             return True
    #     else:  # edge constraint
    #         if constraint['loc'] == [curr_loc, next_loc]:
    #             return True

    # return False

    if len(constraint_table) <= next_time:
        return False

    for constraint in constraint_table[next_time]:
        if constraint['positive']:  # positive constraint
            if constraint['loc'][0] != next_loc:
                return True
        else:  # negative constraint
            if len(constraint['loc']) == 1:  # vertex constraint
                if constraint['loc'][0] == next_loc:
                    return True
            else:  # edge constraint
                if constraint['loc'] == [curr_loc, next_loc]:
                    return True

    return False

def simple_single_agent_astar(my_map, start_loc, goal_loc, h_values, agent, constraints=[], t_max=100):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where r  obot should or cannot go at each timestep
    """

    constraint_table = build_constraint_table(constraints, agent)
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    t_max_achieved = False
    while len(open_list) > 0 and t_max_achieved==False:
        curr = pop_node(open_list)
        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep:
            found = True
            if curr['timestep'] + 1 < len(constraint_table):
                for t in range(curr['timestep'] + 1, len(constraint_table)):
                    if is_constrained(goal_loc, goal_loc, t, constraint_table):
                        found = False
                        earliest_goal_timestep = t + 1
                        break
            if found:
                return get_path(curr)
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]] or is_constrained(curr['loc'], child_loc, curr['timestep'] + 1,
                                                                  constraint_table):
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1}
            if child['timestep'] > t_max:
                t_max_achieved = True
                break
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutionsnodes_dict, from_node, goal_node, heuristics, time_start):

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_path(goal_node):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    #print(path)
    return path