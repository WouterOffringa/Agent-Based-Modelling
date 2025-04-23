def run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    """Run the independent planner for all relevant aircraft.

    Args:
        aircraft_lst (list): List containing all the aircraft
        nodes_dict (dict): Dictionary of the nodes
        edges_dict (dict): Dictionary of the edges with weights. Unused but included for future implementation
        heuristics (dict): Dictionary of the heuristics
        t (float): Current time
    """    
    for ac in aircraft_lst:
        # For all planning aircraft, initialize the path
        if ac.status == "planning":
            # Update relevant parameters and determine final waiting delay
            ac.status = "taxiing"
            ac.track_delay_waiting(t)
            ac.position = nodes_dict[ac.start]["xy_pos"]
            # Determine the path
            ac.plan_independent(nodes_dict, edges_dict, heuristics, t)


def run_independent_planner_taxibots(taxibot_lst, nodes_dict, edges_dict, heuristics, t, agent_lst, horizon):
    """Run the independent planner for all relevant aircraft.

    Args:
        taxibot_lst (list): List containing all the taxibots
        nodes_dict (dict): Dictionary of the nodes
        edges_dict (dict): Dictionary of the edge with weights. Unused but included for future implementation
        heuristics (dict): Dictionary of the heuristics
        t (float): Current time
        agent_lst (list): List containing all the agents
        horizon (list): Timestamps of the horizon
    """    
    for taxibot in taxibot_lst:
        # If the taxibot arrives at its goal node and its goal is to pick up an aircraft, it will start following the aircraft
        if taxibot.status == "arrived" and taxibot.Goal_AC is not None:
            taxibot.status = "following"
            taxibot.Goal_AC.status = "planning"

        # If the taxibot is following an aircraft, it will follow the location of the aircraft
        if taxibot.status == "following":
            taxibot.Follow_AC(t,heuristics)

        # If the taxibot arrives at its goal node and its goal is to hold position, it will start holding position
        if taxibot.status == "arrived" and taxibot.Goal_AC is None:
            taxibot.status = "holding"

        # Once the taxibot is available and at its holding position, it will hold its position
        if taxibot.status == "holding":
            taxibot.Hold_position(t, heuristics)

        # If the taxibot is called, it will taxi to the aircraft
        if taxibot.status == "called by aircraft":
            taxibot.Taxi_to_AC(t, nodes_dict, edges_dict, heuristics, agent_lst, horizon)
            taxibot.status = "taxiing, unavailable"

        # If the aircraft arrives at its target location, the taxibot will go to its holding position
        if taxibot.Goal_AC is not None and taxibot.Goal_AC.status == "arrived":
            taxibot.Taxi_to_holding(t, nodes_dict, edges_dict, heuristics, agent_lst, horizon)
            taxibot.status = "taxiing, available"
            taxibot.Goal_AC = None