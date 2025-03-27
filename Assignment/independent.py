"""
This is an example planner, that calls all agents to plan their route independently.
"""

def run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints=[]):
    for ac in aircraft_lst:
        if ac.status == "planning":
            ac.status = "taxiing"
            ac.track_delay(t)
            ac.position = nodes_dict[ac.start]["xy_pos"]
            ac.plan_independent(nodes_dict, edges_dict, heuristics, t)
            # print("Aircraft", ac.id, "is taxiing, its next states are:", ac.path_to_goal)
        # if ac.status == "arrived" and ac.position == (3.0, 6.0) or ac.position == (3.0, 5.0):
        #     print("ac should leave", ac)
        #     aircraft_lst.remove(ac)
            
            
def run_independent_planner_tugs(tugs_lst, nodes_dict, edges_dict, heuristics, t, constraints=[]):
    for tug in tugs_lst:
        # If the tug arrives at its goal node and its goal is to pick up an aircraft, it will start following the aircraft and acts as 1 agent
        if tug.status == "arrived" and tug.Goal_AC is not None:
            tug.status = "following"
            tug.Goal_AC.status = "planning"
        # If the tug arrives at its goal node and its goal is to hold position, it will start holding position
        if tug.status == "arrived" and tug.Goal_AC is None:
            tug.status = 'holding'
        # Once the tug is available and idling, it will hold its holding position
        if tug.status == "holding":
            tug.Hold_position(t, heuristics)
        # If the tug is called for, it will taxi to the aircraft
        if tug.status == "called by aircraft":
            tug.Taxi_to_AC(t, edges_dict, heuristics)
            # print("Tug", tug.id, "is taxiing to an aircraft") #, its next states are:", tug.path_to_goal)
            tug.status = "taxiing, unavailable"
        # If the tug is following an aircraft, it will follow the location of the aircraft
        if tug.status == "following":
            tug.Follow_AC(t,heuristics)
            # print("Tug", tug.id, "is following an aircraft, its next states are:", tug.path_to_goal)
        #Once a tug is available again and not idling, it will plan to its holding location.

        # If the aircraft arrives at the location, the tug will start holding position again
        if tug.Goal_AC is not None and tug.Goal_AC.status == "arrived":
            tug.Taxi_to_holding(t, edges_dict, heuristics)
            # print("Tug", tug.id, "is taxiing to holding position, its next states are:", tug.path_to_goal)
            tug.status = "taxiing, available"
            # tug.Goal_AC.status = "arrived"
            tug.Goal_AC = None
        