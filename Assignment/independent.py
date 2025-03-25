"""
This is an example planner, that calls all agents to plan their route independently.
"""

def run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints=[]):
    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing" 
            ac.position = nodes_dict[ac.start]["xy_pos"]
            ac.plan_independent(nodes_dict, edges_dict, heuristics, t)
            print("Aircraft", ac.id, "is taxiing, its next states are:", ac.path_to_goal)
            
def run_independent_planner_tugs(tugs_lst, nodes_dict, edges_dict, heuristics, t, constraints=[]):
    for tug in tugs_lst:
        if tug.idle == True and tug.status == "available":
            tug.Hold_position(t,heuristics)
            print("Tug", tug.id, "is holding position, its next states are:", tug.path_to_goal)
        if tug.idle == True and tug.status == "unavailable":
            tug.Follow_AC(t,heuristics)
            print("Tug", tug.id, "is following an aircraft, its next states are:", tug.path_to_goal)
        if tug.idle == False and tug.status == "available":
            tug.Taxi_to_holding(t,heuristics)
            print("Tug", tug.id, "is taxiing to holding position, its next states are:", tug.path_to_goal)
        if tug.idle == False and tug.status == "unavailable":
            tug.Taxi_to_AC(t,heuristics)
            print("Tug", tug.id, "is taxiing to an aircraft, its next states are:", tug.path_to_goal)
        