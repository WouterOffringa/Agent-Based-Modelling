"""
This is an example planner, that calls all agents to plan their route independently.
"""

def run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints=[]):
    for ac in aircraft_lst:
        if ac.status == "planning":
            ac.status = "taxiing" 
            ac.position = nodes_dict[ac.start]["xy_pos"]
            ac.plan_independent(nodes_dict, edges_dict, heuristics, t)

            
def run_independent_planner_tugs(tugs_lst, nodes_dict, edges_dict, heuristics, t, constraints=[]):
    for tug in tugs_lst:
        #print the timestep we are in
        if tug.status == "arrived": #TODO: this should actually be done when we arrive at node, not when we will arrive next node

            tug.status = "following"
            tug.Goal_AC.status = "planning"
        #Once the tug is available and idling, it will hold its holding position
        if tug.status == "holding":
            tug.Hold_position(t, heuristics)
            #print("Tug", tug.id, "is holding position, its next states are:", tug.path_to_goal)
            #Once the tug is unavailable and idling, it mean it is following the position of an aicraft
        if tug.status == "called by aircraft":
            tug.Taxi_to_AC(t,heuristics)

        if tug.status == "following":
            tug.Follow_AC(t,heuristics)

        #Once a tug is available again and not idling, it will plan to its holding location.
        if tug.Goal_AC is not None and tug.Goal_AC.status == "arrived":
            tug.status = "planning, available"

        if tug.status == "planning, available":
            tug.Taxi_to_holding(t,heuristics)

            tug.status = "taxiing, available"
            tug.Goal_AC = None
        