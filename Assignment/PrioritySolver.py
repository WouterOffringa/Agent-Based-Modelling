"""

Here, we will define the PrioritySolver definition, which will be used when a conflict between two agents is found.
It will call the priority function of the agents, compare them return a replanning task with an added constraint to the agent with the lowest priority.


"""

import Aircraft


def PriorityDetector(aircraft_lst, t):
    horizon_length = [t+0.5, t+1., t+1.5] #These timesteps will be checked for possible collision
    for ac in aircraft_lst:
        if ac.status == "taxiing":
            ac.conflict_detection(aircraft_lst, horizon_length)

