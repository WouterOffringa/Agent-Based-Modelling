import heapq
import networkx as nx

def calc_heuristics(graph, nodes_dict):
    """
    Calculates the heuristic dict (shortest distance between two nodes) to be used in A* search.
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


def build_constraint_table(constraints, agent):
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
        if len(constraint['node_id']) == 1:  # positive vertex constraint
            constraint_table[constraint['timestep']].append({'node_id': constraint['node_id'], 'positive': True})
        else:  # positive edge constraint
            constraint_table[constraint['timestep'] - 1].append({'node_id': [constraint['node_id'][0]], 'positive': True})
            constraint_table[constraint['timestep']].append({'node_id': [constraint['node_id'][1]], 'positive': True})

    for constraint in negative:
        if len(constraint['node_id']) == 1:  # vertex constraint
            constraint_table[constraint['timestep']].append({'node_id': constraint['node_id'], 'positive': False})
        elif constraint['positive']:  # positive edge constraint for other agents
            constraint_table[constraint['timestep'] - 1].append({'node_id': [constraint['node_id'][0]], 'positive': False})
            constraint_table[constraint['timestep']].append({'node_id': [constraint['node_id'][1]], 'positive': False})
            constraint_table[constraint['timestep']].append(
                {'node_id': [constraint['node_id'][1], constraint['node_id'][0]], 'positive': False})
        else:  # negative edge constraint
            constraint_table[constraint['timestep']].append({'node_id': constraint['node_id'], 'positive': False})

    return constraint_table


def is_constrained(curr_node, next_node, next_time, constraint_table):

    if len(constraint_table) <= next_time:
        return False

    for constraint in constraint_table[int(next_time)]:
        if constraint['positive']:  # positive constraint
            if constraint['node_id'][0] != next_node:
                return True
        else:  # negative constraint
            if len(constraint['node_id']) == 1:  # vertex constraint
                if constraint['node_id'][0] == next_node:
                    return True
            else:  # edge constraint
                if constraint['node_id'] == [curr_node, next_node]:
                    return True

    return False


def simple_single_agent_astar(nodedict, start_node, goal_node, h_values, agent, current_time=0, constraints = [], t_path_max = 100):
 
    t_max = t_path_max + current_time
    constraint_table = build_constraint_table(constraints, agent)
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = current_time
    h_value = h_values[start_node][start_node]
    root = {'node_id': start_node,
            'g_val': 0,
            'h_val': h_value,
            'parent': None,
            'timestep': current_time}
    push_node(open_list, root)
    closed_list[(root['node_id'], root['timestep'])] = root
    t_max_achieved = False
    while len(open_list) > 0 and t_max_achieved==False:
        curr = pop_node(open_list)
        if curr['node_id'] == goal_node and curr['timestep'] > earliest_goal_timestep:
            found = True
            if curr['timestep'] + 1 < len(constraint_table):
                for t in range(curr['timestep'] + 1, len(constraint_table)):
                    if is_constrained(goal_node, goal_node, t, constraint_table):
                        found = False
                        earliest_goal_timestep = t + 1
                        break
            if found:
                return True, get_path(curr)
        # Substitution for move()
        child_nodes = nodedict[curr['node_id']]['neighbors'] | {curr['node_id']}
        for child_node in child_nodes: 
            if is_constrained(curr['node_id'], child_node, curr['timestep'] + 1, constraint_table):
                continue
            child = {'node_id': child_node,
                    'g_val': curr['g_val'] + 0.5,
                    'h_val': h_values[start_node][child_node],
                    'parent': curr,
                    'timestep': curr['timestep'] + 0.5}
            if child['timestep'] > t_max:
                t_max_achieved = True
                break
            if (child['node_id'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['node_id'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['node_id'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['node_id'], child['timestep'])] = child
                push_node(open_list, child)                

    return False, None  # Failed to find solutionsnodes_dict, from_node, goal_node, heuristics, time_start):


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['node_id'], node))

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
        path.append((curr['node_id'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    #print(path)
    return path