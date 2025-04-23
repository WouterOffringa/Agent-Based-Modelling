import os

# Import agents
from Agents.Aircraft import Aircraft
from Agents.Taxibot import Taxibot
from Agents.Functionality.single_agent_planner import calc_heuristics

# Import conflict solvers
from ConflictSolver.cbs import run_CBS
from ConflictSolver.independent import *
from ConflictSolver.prioritized import run_prioritized_planner
from ConflictSolver.PrioritySolver import PrioritySolver

# Import airport
from Airport.airportlayout import *

# Import visualisation
from Visualization.visualization import *
import matplotlib
matplotlib.use('TkAgg')  # Set the backend to 'TkAgg' (interactive)
import matplotlib.pyplot as plt


def TaxiingSimulation(n_taxibots, 
                        nodes_file="nodes_v2.xlsx",
                        edges_file="edges_v2.xlsx"):
    
    # === Initialization ===
    # Set simulation parameters
    t_max = 100             # Maximum simulation time
    horizonspan = 1.5       # Size of the conflict detection horizon
    planner = "Independent" # Selected planner
    random_spawning = True  # Whether to do random spawning or not
    running = True          # Termination parameter for the simulation
    escape_pressed = False  # Manual stopping input
    dt = 0.1                # Should be a factor of 0.5 (0.5/dt should be integer)
    t = 0

    # Set visualization parameters
    plot_graph = False                      # Show graph representation in NetworkX
    visualization = True                    # Pygame visualization
    slow_factor = 0.01                      # 5 here means 5 times slower
    visualization_speed = 0.1*slow_factor   # Set at 0.1 as default

    # Load the airport map
    nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)

    gates = [node for node in nodes_dict if nodes_dict[node]["type"] == "gate"]
    rwy_dep = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_d"]
    rwy_arr = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_a"]
    tug_gates = [node for node in nodes_dict if nodes_dict[node]["type"] == "taxiparking"]

    graph = create_graph(nodes_dict, edges_dict, plot_graph)
    heuristics = calc_heuristics(graph, nodes_dict)

    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict)

    # Initialize agent lists
    aircraft_lst = []   #List which can contain aircraft agents
    tug_lst = []    #List which can contain tug agents  
    agent_lst = []  #List which can contain all


    # === Simulation ===
    while running:
        t = round(t,2)

        # Check termination conditions
        if t >= t_max or escape_pressed: 
            running = False
            pg.quit()
            print("Simulation Stopped")
            break 

        # Update the map if visualization is true
        if visualization:
            current_states = {} # Collect current states of all agents
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
        
        # Spawn aircraft
