def PrioritySolver(agent_lst, t, edges_dict, nodes_dict, heuristics, horizonspan):
    """Detect and resolve conflicts between agents using priority levels.

    Args:
        agent_lst (list): List containing all the agents
        t (float): Current time
        edges_dict (dict): Dictionary of the edge with weights. Unused but included for future implementation
        nodes_dict (dict): Dictionary of the nodes
        heuristics (dict): Dictionary of the heuristics
        horizonspan (float): How far the conflict horizon spans into the future
    """    
    # Create the horizon
    horizon = [t+k*0.5 for k in range(int(1 + (horizonspan // 0.5)))]

    # Detect and resolve conflicts for all relevant agents
    for agent in agent_lst:
        if agent.status == "taxiing" or agent.status == "taxiing, available" or agent.status == "taxiing, unavailable":
            agent.conflict_detection(agent_lst, horizon, t, edges_dict,nodes_dict, heuristics)
