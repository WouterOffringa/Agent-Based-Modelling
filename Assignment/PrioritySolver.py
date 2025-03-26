"""

Here, we will define the PrioritySolver definition, which will be used when a conflict between two agents is found.
It will call the priority function of the agents, compare them return a replanning task with an added constraint to the agent with the lowest priority.


"""

import Aircraft
import Taxibot
from independent import run_independent_planner


def PriorityDetector(agent_lst, t, edges_dict, nodes_dict, heuristics):
    horizon_length = [t+0.5, t+1., t+1.5] #These timesteps will be checked for possible collision
    for agent in agent_lst:
        if agent.status == "taxiing" or agent.status == "taxiing, available" or agent.status == "taxiing, unavailable":
            # print("Checking for conflicts for agent", agent.id)
            agent.conflict_detection(agent_lst, horizon_length, t, edges_dict,nodes_dict, heuristics)
            
                


