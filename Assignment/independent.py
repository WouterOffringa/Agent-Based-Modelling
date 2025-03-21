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
        if tug.idle == True:
            tug.Hold_position(t)
            print("Tug", tug.id, "is holding position, its next states are:", tug.path_to_goal)
        #TODO: implement all other possible states
        