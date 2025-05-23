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
from datetime import datetime
import gc # garbage collector
import matplotlib
matplotlib.use('TkAgg')  # Set the backend to 'TkAgg' (interactive)
import matplotlib.pyplot as plt

#SET up the simulation tracker

t_max = 100

# results_folder = "sim_results_with_path_lengthsss"
results_folder = 'Sensitivity_results'
if not os.path.exists(results_folder):
    os.makedirs(results_folder)

columns_results = ["agent_id", "start", "goal", "arrival_time", "departure_time", "taxi_time", "optimal taxi_time", "waiting_time", "taxi_delay", "total_delay"]
simulating = True
Minimum_sims = 100
sim_no = 1
alfa = 0.05 #95% confidence interval
l = 0.5 #length of the interval for the total delay
z_value = 1.96 #fill in compared to alfa /2 (alpha 0.05, gives alfa/2 of 0.025, z_value = 1.96)
Entries_per_sim = 25 #number of entries per simulation
mean_results = []
Stdev_results = []
    
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


### Define global variables

nodes_file = "nodes_v2.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges_v2.xlsx" #xlsx file with for each edge: from  (node), to (node), length
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)

# Create lists of different kind of nodes
gates = [node for node in nodes_dict if nodes_dict[node]["type"] == "gate"]
rwy_dep = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_d"]
rwy_arr = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_a"]
tug_gates = [node for node in nodes_dict if nodes_dict[node]["type"] == "taxiparking"]


while simulating == True:
    gc.collect() # garbage collector
    sim_results = []

    #Parameters that can be changed:
    simulation_time = 200
    planner = "Independent" #choose which planner to use (currently only Independent is implemented)

    #Visualization (can also be changed)
    plot_graph = False                      # show graph representation in NetworkX
    visualization = True                    # pygame visualization
    slow_factor = 0.01                      # 5 here means 5 times slower
    visualization_speed = 0.1*slow_factor   # set at 0.1 as default



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

    graph = create_graph(nodes_dict, edges_dict, plot_graph)
    heuristics = calc_heuristics(graph, nodes_dict)

    aircraft_lst = []   #List which can contain aircraft agents
    tug_lst = []    #List which can contain tug agents  
    agent_lst = []  #List which can contain all 

    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

    # =============================================================================
    # 1. While loop and visualization
    # =============================================================================

    running=True
    escape_pressed = False
    time_end = simulation_time
    dt = 0.1 #should be factor of 0.5 (0.5/dt should be integer)
    t = 0
    i = 0
    arrival_available, dep_available = True, True
    UnsolvablePresent = True
    n_unsolvable = 0


    print("\nSimulation Started\n")
    try:
        while running:

            t= round(t,2) 

            #Check conditions for termination
            # if t >= time_end or escape_pressed:
            if escape_pressed:
                running = False
                pg.quit()
                print("Simulation Stopped")
                break 
            
            #Visualization: Update map if visualization is true
            if visualization:
                current_states = {} #Collect current states of all aircraft
                for ac in aircraft_lst:
                    if ac.status == "taxiing" or ac.status == "holding" or ac.status == "pickup":
                        current_states[ac.id] = {"type": "aircraft",
                                                "ac_id": ac.id,
                                                "xy_pos": ac.position,
                                                "heading": ac.heading,
                                                "status": ac.status}
                for tug in tug_lst:
                    if tug.status != "following":
                        current_states[tug.id] = {"type": "tug",
                                                    "ac_id": tug.id,
                                                    "xy_pos": tug.position,
                                                    "heading": tug.heading,
                                                    "status": tug.status}
                counter, escape_pressed = map_running(map_properties, current_states, t)
                timer.sleep(visualization_speed) 
            
            #Spawn aircraft for this timestep (use for example a random process)
            # ==== Random Spawning ====
            random_spawning = True
            spawning_time = 4
            if (t-1) % spawning_time == 0 and (arrival_available is not False or dep_available is not False):
                ac_type = random.choice(['A','D']) #randomly choose arrival or departure
                if ac_type == 'D':
                    available_gates = gates.copy()
                    for ac in aircraft_lst:
                        if ac.status == "holding" or ac.status == "pickup":
                            if ac.start in available_gates:
                                available_gates.remove(ac.start)
                            elif ac.from_to[0] in available_gates:
                                available_gates.remove(ac.from_to[0])
                            elif ac.goal in available_gates:
                                available_gates.remove(ac.goal)
                        elif ac.status == "taxiing":
                            if ac.goal in available_gates:
                                available_gates.remove(ac.goal)

                    #print("Available gates for departure: ", available_gates)
                    if len(available_gates) > 0:
                        i += 1
                        dep_available = True
                        spawn_gate = random.choice(available_gates)
                        ac = Aircraft(i, 'D', spawn_gate, random.choice(rwy_dep), t, nodes_dict)
                        ac.status = "holding"
                        aircraft_lst.append(ac)
                        agent_lst.append(ac)

                    else:
                        print("==No gates available for departure")
                        dep_available = False

                else:
                    rwy_arr = [37,38]
                    available_rwy = rwy_arr
                    for ac in aircraft_lst:
                        if ac.status == "holding" or ac.status == "pickup": # and not (ac.status == "arrived" or ac.status == "taxiing"):
                            if ac.start in available_rwy:
                                available_rwy.remove(ac.start)
                            elif ac.from_to[0] in available_rwy:
                                available_rwy.remove(ac.from_to[0])
                    #print("Available runways for arrival: ", available_rwy)
                    if len(available_rwy) > 0:
                        i += 1
                        arrival_available = True
                        spawn_rwy = random.choice(available_rwy)
                        ac = Aircraft(i, 'A', spawn_rwy, random.choice(gates), t, nodes_dict)
                        ac.status = "holding"
                        aircraft_lst.append(ac)
                        agent_lst.append(ac)
                    else:
                        print("==No runways available for arrival")
                        arrival_available = False


            # ==== Fixed Spawning ====
            if not random_spawning:
                spawning_time = 10
                if t == 1:
                    ac = Aircraft(1, 'A', 37,97,t, nodes_dict)
                    ac.status = "holding"
                    ac1 = Aircraft(2, 'D', 38,97,t, nodes_dict)
                    ac1.status = "holding"
                    aircraft_lst.append(ac)
                    agent_lst.append(ac)
                    aircraft_lst.append(ac1)
                    agent_lst.append(ac1)
                if t == 11:
                    ac = Aircraft(3, 'A', 35,1,t, nodes_dict)
                    ac.status = "holding"
                    ac1 = Aircraft(4, 'D', 36,1,t, nodes_dict)
                    ac1.status = "holding"
                    aircraft_lst.append(ac)
                    agent_lst.append(ac)
                    aircraft_lst.append(ac1)
                    agent_lst.append(ac1)


            
            # ==== Spawning the taxibots ====
            spawning_locations = tug_gates
            nr_taxibots = 5
            amount_deleted_taxibots = 5 - nr_taxibots
            spawning_locations = spawning_locations[amount_deleted_taxibots:]
            alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
            if t == 0:
                # print('spawning locations are', spawning_locations)
                for i, location in enumerate(spawning_locations, start=1):
                    tug = Taxibot(alphabet[i-1], location, location, nodes_dict)
                    tug_lst.append(tug)
                    agent_lst.append(tug)
                    tug.idle = True
                constraints = []
                run_independent_planner_tugs(tug_lst, nodes_dict, edges_dict, heuristics, t, agent_lst, [t, t+0.5, t+1., t+1.5], constraints=constraints)

        #Requesting taxibots for ac

            for ac in aircraft_lst:
                if ac.status == "holding" and t % 0.5 == 0:
                    ac.request_taxibot(nodes_dict, tug_lst, heuristics, t)

            #Do planning 
            if planner == "Independent":     

                if t % 0.5 == 0:
                    PriorityDetector(agent_lst, t, edges_dict, nodes_dict, heuristics)
                    run_independent_planner_tugs(tug_lst, nodes_dict, edges_dict, heuristics, t, agent_lst, [t, t+0.5, t+1], constraints=constraints)
                    run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints=constraints)

                    

            elif planner == "Prioritized":
                run_prioritized_planner()
            elif planner == "CBS":
                run_CBS()
            #elif planner == -> you may introduce other planners here
            else:
                raise Exception("Planner:", planner, "is not defined.")
                            

            #Move the aircraft that are taxiing
            ac_remove = []
            for ac in aircraft_lst: 
                if ac.status == "taxiing": 
                    ac.move(dt, t)
                if ac.status == "arrived":
                    ac_remove.append(ac)

            #Remove aircraft that have arrived ans save results
            for ac in ac_remove:
                #Track all the data of the aircraft
                taxi_time = ac.arrival_time - ac.departure_time
                optimal_taxi_time = ac.ideal_arrival_time - ac.departure_time
                taxi_delay = taxi_time - optimal_taxi_time

                ac_results = {"agent_id": ac.id,
                                "start": ac.start,
                                "goal": ac.goal,
                                "arrival_time": ac.arrival_time,
                                "departure_time": ac.departure_time,
                                "taxi_time": taxi_time,
                                "optimal taxi_time": optimal_taxi_time,
                                "waiting_time": ac.delay,
                                "taxi_delay": taxi_delay,
                                "total_delay": (taxi_delay + ac.delay)}     
                sim_results.append(ac_results)
                
                aircraft_lst.remove(ac)
                agent_lst.remove(ac)

            #Move the taxibots that are taxiing
            for tug in tug_lst:
                if tug.status == 'taxiing, unavailable' or tug.status == 'taxiing, available':
                    tug.move(dt, t)

            if len(sim_results) >= Entries_per_sim or t == time_end:
                #Save sim results to a file
                df = pd.DataFrame(sim_results, columns=columns_results)
                df.to_csv(os.path.join(results_folder, f"simulation_results_{000 + sim_no}.csv"), index=False)
                print(counter, "amount of collisions detected")

                print("Max Entries per sim reached, restarting simulation") 
                running = False

            
            t = t + dt
            if t % 10 == 0:
                gc.collect()
    except:
        UnsolvablePresent = True
    pg.quit()
    #Calculate mean and std of results, update
    if UnsolvablePresent and running:
        print('Simulation contained an unsolvable conflict resolution, skipping...')
        n_unsolvable
    # elif sim_no == 1:
    #     Mean_result = df["total_delay"].mean()
    #     Stdev_result = 0
    #     print("Mean result: ", Mean_result)
    #     mean_results.append(Mean_result)
    #     Stdev_results.append(Stdev_result)
    #     bound = 100000 #set bound to a large number as initial (no solution yet)
    # else:
    #     Mean_result_new = Mean_result + (df["total_delay"].mean() - Mean_result) / sim_no
    #     Stdev_result = ((1 - 1/(sim_no -1))*(Stdev_result**2) + (sim_no)*(Mean_result_new - Mean_result)**2)**(1/2)
    #     Mean_result = Mean_result_new
    #     print("Mean result: ", Mean_result)
    #     print("Stdev result: ", Stdev_result)
    #     mean_results.append(Mean_result)
    #     Stdev_results.append(Stdev_result)
    #     print("Mean results list: ", mean_results)
    #     print("Stdev results list: ", Stdev_results)
    #
    #     #Check if correct confidence is reached
    #     bound = 2 * z_value * Stdev_result / (sim_no**0.5)
    #     print("bound is: ", bound, "l is: ", l)
    #
    if UnsolvablePresent and running:
        pass
    
    # elif bound < l and sim_no > Minimum_sims:
    #     print("Confidence interval reached")
    #     simulating = False
    #     pg.quit()
    #     break



        
    sim_no = sim_no + 1    
    # =============================================================================
    # 2. Implement analysis of output data here
    # =============================================================================

