def PriorityDetector(agent_lst, t, edges_dict, nodes_dict, heuristics):
    horizon_length = [t, t+0.5, t+1., t+1.5] #These timesteps will be checked for possible collision
    for agent in agent_lst:
        if agent.status == "taxiing" or agent.status == "taxiing, available" or agent.status == "taxiing, unavailable":
            agent.conflict_detection(agent_lst, horizon_length, t, edges_dict,nodes_dict, heuristics)
