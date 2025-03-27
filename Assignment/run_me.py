"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

# Favourable method to run in the terminal, when the Assignment folder is opened

import os
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
import random
from single_agent_planner_v2 import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from Taxibot import Taxibot
from independent import run_independent_planner
from independent import run_independent_planner_tugs
from prioritized import run_prioritized_planner
from cbs import run_CBS
from PrioritySolver import PriorityDetector

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 300
planner = "Independent" #choose which planner to use (currently only Independent is implemented)

#Visualization (can also be changed)
plot_graph = False                      # show graph representation in NetworkX
visualization = True                    # pygame visualization
slow_factor = 1                         # 5 here means 5 times slower
visualization_speed = 0.1*slow_factor   # set at 0.1 as default

#%%Function definitions
def import_layout(nodes_file, edges_file):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates 
    """
    gates_xy = []   #lst with (x,y) positions of gates
    rwy_dep_xy = [] #lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = [] #lst with (x,y) positions of exit points of arrival runways
    
    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)
    
    #Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"],row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        #Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"],row["y_pos"]))

    #Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy, 
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}
    
    #Create edges_dict from df_edges
    edges_dict = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"],row["to"])
        from_node =  edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos
                           }
        edges_dict[edge_id] = edge_properties
   
    #Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)  
    
    return nodes_dict, edges_dict, start_and_goal_locations

def create_graph(nodes_dict, edges_dict, plot_graph = True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """
    
    graph = nx.DiGraph() #create directed graph in NetworkX
    
    #Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node, 
                       node_id = nodes_dict[node]["id"],
                       xy_pos = nodes_dict[node]["xy_pos"],
                       node_type = nodes_dict[node]["type"])
        
    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1], 
                       edge_id = edge,
                       from_node =  edges_dict[edge]["from"],
                       to_node = edges_dict[edge]["to"],
                       weight = edges_dict[edge]["length"])
    
    #Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph

#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

aircraft_lst = []   #List which can contain aircraft agents
tug_lst = []    #List which can contain tug agents  
agent_lst = []  #List which can contain all agents

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

# create lists of different kind of nodes
gates = [node for node in nodes_dict if nodes_dict[node]["type"] == "gate"]
rwy_dep = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_d"]
rwy_arr = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_a"]
tug_gates = [7, 9, 16, 23, 107]

# =============================================================================
# 1. While loop and visualization
# =============================================================================
 
#Start of while loop    
running=True
escape_pressed = False
time_end = simulation_time
dt = 0.1 #should be factor of 0.5 (0.5/dt should be integer)
t= 0

print("\nSimulation Started\n")
while running:

    t= round(t,2)    
    #Check conditions for termination
    if t >= time_end or escape_pressed: 
        running = False
        pg.quit()
        print("Simulation Stopped")
        break 
    
    #Visualization: Update map if visualization is true
    if visualization:
        current_states = {} #Collect current states of all aircraft
        for ac in aircraft_lst:
            if ac.status == "taxiing" or ac.status == "holding":
                current_states[ac.id] = {"type": "aircraft",
                                        "ac_id": ac.id,
                                         "xy_pos": ac.position,
                                         "heading": ac.heading}
        for tug in tug_lst:
            if tug.status != "following":
                current_states[tug.id] = {"type": "tug",
                                            "ac_id": tug.id,
                                            "xy_pos": tug.position,
                                            "heading": tug.heading}
        escape_pressed = map_running(map_properties, current_states, t)
        timer.sleep(visualization_speed) 
      
    #Spawn aircraft for this timestep (use for example a random process)
    # ==== Random Spawning ====
    spawning_time = 7
    if (t-1) % spawning_time == 0:
        i = len(aircraft_lst) + 1
        ac_type = random.choice(['A', 'D']) #randomly choose arrival or departure
        if ac_type == 'A':
            ac = Aircraft(i, 'A', random.choice(gates), random.choice(rwy_dep), t, nodes_dict)
            ac.status = "holding"
            aircraft_lst.append(ac)
            agent_lst.append(ac)
        else:
            ac = Aircraft(i, 'D', random.choice(rwy_arr), random.choice(gates), t, nodes_dict)
            ac.status = "holding"
            aircraft_lst.append(ac)
            agent_lst.append(ac)


    # ==== Fixed Spawning ====
    # spawning_time = 25
    # if (t-1) % spawning_time == 0:
    #     #case 1 - 4 aircraft which touch in the bottom right corner
    #     ac = Aircraft(1, 'A', 37,36,t, nodes_dict) 
    #     ac.status = "holding"
    #     ac1 = Aircraft(2, 'D', 36,37,t, nodes_dict)
    #     ac1.status = "holding"
    #     ac2 = Aircraft(3, 'A', 38,98,t, nodes_dict) 
    #     ac2.status = "holding"
    #     ac3 = Aircraft(4, 'D', 98,38,t, nodes_dict)
    #     ac3.status = "holding"
    #     aircraft_lst.append(ac)
    #     agent_lst.append(ac)
    #     aircraft_lst.append(ac1)
    #     agent_lst.append(ac1)
    #     aircraft_lst.append(ac2)
    #     agent_lst.append(ac2)
    #     aircraft_lst.append(ac3)
    #     agent_lst.append(ac3)
        
        # case 2 - 4 aircraft which needs to cross diagonally
        # ac = Aircraft(1, 'A', 37,34,t, nodes_dict)
        # ac.status = "holding"
        # ac1 = Aircraft(2, 'D', 38,97,t, nodes_dict)
        # ac1.status = "holding"
        # aircraft_lst.append(ac)
        # agent_lst.append(ac)
        # aircraft_lst.append(ac1)
        # agent_lst.append(ac1)
        # ac2 = Aircraft(3, 'A', 34,37,t, nodes_dict)
        # ac2.status = "holding"
        # ac3 = Aircraft(4, 'D', 97,38,t, nodes_dict)
        # ac3.status = "holding"
        # aircraft_lst.append(ac2)
        # agent_lst.append(ac2)
        # aircraft_lst.append(ac3)
        # agent_lst.append(ac3)
        


    #     constraints = []
    # if t == 25:
    #     aircraft_lst.clear()
        # this clears the aircraft list just for case 2
    
    # ==== Spawning the taxibots ====
    spawning_locations = [7, 9, 16, 23, 107]
    alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
    if t == 0:
        for i, location in enumerate(spawning_locations, start=1):
            tug = Taxibot(alphabet[i-1], location, location, nodes_dict)
            tug_lst.append(tug)
            agent_lst.append(tug)
            tug.idle = True
        constraints = []
        run_independent_planner_tugs(tug_lst, nodes_dict, edges_dict, heuristics, t, constraints=constraints)
    
    #Do planning 
    if planner == "Independent":     
        if (t-1) % spawning_time == 0: #(Hint: Think about the condition that triggers (re)planning) 
            for ac in aircraft_lst:

                # run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints=constraints)
                if ac.status == "holding":
                    ac.request_taxibot(nodes_dict, tug_lst, heuristics, t)

        if t % 0.5 == 0:
            run_independent_planner_tugs(tug_lst, nodes_dict, edges_dict, heuristics, t, constraints=constraints)

        for ac in aircraft_lst:
            if ac.status == "planning":
                run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints=constraints)
            

        #implement the check to see if two aircraft will collide with eachother
        if t % 0.5 == 0:
            PriorityDetector(agent_lst, t, edges_dict, nodes_dict, heuristics)


        #Check the planning for the taxibots

        
    elif planner == "Prioritized":
        run_prioritized_planner()
    elif planner == "CBS":
        run_CBS()
    #elif planner == -> you may introduce other planners here
    else:
        raise Exception("Planner:", planner, "is not defined.")
                       
    #Move the aircraft that are taxiing
    for ac in aircraft_lst: 
        if ac.status == "taxiing": 
            ac.move(dt, t)
    
    for tug in tug_lst:
        # if tug.status != "holding" and tug.status != "following" and tug.status != "arrived": #TODO Set to new states
        if tug.status == 'taxiing, unavailable' or tug.status == 'taxiing, available':
            tug.move(dt, t)
                           
    t = t + dt
          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?
