# Import basic functionality
import random
import gc

# Import agents
from AgentBasedModel.Agents.Aircraft import Aircraft
from AgentBasedModel.Agents.Taxibot import Taxibot
from AgentBasedModel.Agents.Functionality.single_agent_planner import calc_heuristics

# Import conflict solvers
from AgentBasedModel.ConflictSolver.cbs import run_CBS
from AgentBasedModel.ConflictSolver.independent import *
from AgentBasedModel.ConflictSolver.prioritized import run_prioritized_planner
from AgentBasedModel.ConflictSolver.PrioritySolver import PrioritySolver

# Import airport
from AgentBasedModel.Airport.airportlayout import *

# Import visualisation
from AgentBasedModel.Visualization.visualization import *


def TaxiingSimulation(scenario = None,
                      n_taxibots = 5,
                      t_max = 100,
                      spawntime_aircraft = 4,
                      entries_per_sim = 1000,
                      visualization = True,
                      nodes_file = "AgentBasedModel\\Airport\\nodes_v2.xlsx",
                      edges_file = "AgentBasedModel\\Airport\\edges_v2.xlsx"):
    """Run the agent-based model for taxi operations.

    Args:
        scenario (dict, optional): dictionary of spawn events, formatted as {id: {'spawn_time':(float), 'a_d':('a'/'d'), 'start_node':(node_id), 'goal_node':(node_id)}}. Defaults to None.
        n_taxibots (int, optional): number of taxibots in the simulation. Defaults to 5.
        t_max (int, optional): maximum simulation time. Defaults to 100.
        spawntime_aircraft (int, optional): spawning interval for aircraft. Defaults to 4.
        entries_per_sim (int, optional): maximum amount of entries per simulation. Defaults to 1000.
        visualization (bool, optional): whether to visualize the simulation. Defaults to True.
        nodes_file (str, optional): path to the nodes file. Defaults to "AgentBasedModel\Airport\nodes_v2.xlsx".
        edges_file (str, optional): path to the edges file. Defaults to "AgentBasedModel\Airport\edges_v2.xlsx".

    Returns:
        list: simulation results
    """    
    # === Initialization ===
    # Set simulation parameters
    horizonspan = 1.5           # Size of the conflict detection horizon
    planner = "Independent"     # Selected planner
    running = True              # Termination parameter for the simulation
    escape_pressed = False      # Manual stopping input
    dt = 0.1                    # Should be a factor of 0.5 (0.5/dt should be integer)
    t = 0                       # Initial time
    arrival_available = True    # Whether arrival is available for spawning
    dep_available = True        # Whether departure is available spawning

    # Set visualization parameters
    plot_graph = False                      # Show graph representation in NetworkX
    slow_factor = 1                         # 5 here means 5 times slower
    visualization_speed = 0.1*slow_factor   # Set at 0.1 as default

    # Load the airport map
    nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)

    gates = [node for node in nodes_dict if nodes_dict[node]["type"] == "gate"]
    rwy_dep = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_d"]
    rwy_arr = [node for node in nodes_dict if nodes_dict[node]["type"] == "rwy_a"]
    taxibot_gates = [node for node in nodes_dict if nodes_dict[node]["type"] == "taxiparking"]

    graph = create_graph(nodes_dict, edges_dict, plot_graph)
    heuristics = calc_heuristics(graph, nodes_dict)

    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict)

    # Initialize agent lists
    aircraft_lst = []   # List which contains aircraft agents
    taxibot_lst = []    # List which contains tug agents  
    agent_lst = []      # List which contains all agents
    ac_id = 1           # Aircraft ID

    # === Simulation ===
    sim_results = []
    while running:
        t = round(t,2)

        # Check termination conditions
        if t >= t_max or escape_pressed: 
            running = False
            pg.quit()
            print("Simulation Stopped")
            break 
        elif len(sim_results) > entries_per_sim:
                print("Max Entries per sim reached, restarting simulation") 
                running = False

        # Update the map if visualization is true
        if visualization:
            # Collect current states of all agents
            current_states = {}
            for ac in aircraft_lst:
                if ac.status == "taxiing" or ac.status == "holding" or ac.status == "pickup":
                    current_states[ac.id] = {"type":    "aircraft",
                                            "ac_id":    ac.id,
                                            "xy_pos":   ac.position,
                                            "heading":  ac.heading,
                                            "status":   ac.status}
            for taxibot in taxibot_lst:
                if taxibot.status != "following":
                    current_states[taxibot.id] = {"type":   "tug",
                                                "ac_id":    taxibot.id,
                                                "xy_pos":   taxibot.position,
                                                "heading":  taxibot.heading,
                                                "status":   taxibot.status}
            
            # Update the map
            counter, escape_pressed = map_running(map_properties, current_states, t)
            timer.sleep(visualization_speed) 
        
        # Spawn aircraft
        if scenario == None:
            # Random spawning: check whether it is time to spawn an aircraft and whether a spawning spot is available
            if (t-1) % spawntime_aircraft == 0 and (arrival_available is not False or dep_available is not False):
                ac_type = random.choice(['A','D']) # randomly choose arrival or departure

                # Spawn the aircraft at an available gate
                if ac_type == 'D':
                    available_gates = gates.copy()
                    # Check which gates are available
                    for ac in aircraft_lst:
                        if ac.status == "holding" or ac.status == "pickup":
                            # A gate is not available if another active aircraft started from it
                            if ac.start in available_gates:
                                available_gates.remove(ac.start)
                            # A gate is not available if another aircraft is already at or just departing from it
                            elif ac.from_to[0] in available_gates:
                                available_gates.remove(ac.from_to[0])
                            # A gate is not available if another aircraft is already assigned to it
                            elif ac.goal in available_gates:
                                available_gates.remove(ac.goal)
                        elif ac.status == "taxiing":
                            # A gate is not available if another aircraft is already assigned to it
                            if ac.goal in available_gates:
                                available_gates.remove(ac.goal)
            
                    # If a gate is available, spawn the aircraft at a random, available gate
                    if len(available_gates) > 0:
                        ac_id += 1
                        dep_available = True
                        spawn_gate = random.choice(available_gates)
                        ac = Aircraft(ac_id, 'D', spawn_gate, random.choice(rwy_dep), t, nodes_dict)
                        ac.status = "holding"
                        aircraft_lst.append(ac)
                        agent_lst.append(ac)
                    else:
                        print("==No gates available for departure")
                        dep_available = False

                # Spawn the aircraft at an available arrival runway exit
                else:
                    available_rwy = rwy_arr.copy()
                    for ac in aircraft_lst:
                        if ac.status == "holding" or ac.status == "pickup":
                            # A runway exit is not available if another active aircraft started from it
                            if ac.start in available_rwy:
                                available_rwy.remove(ac.start)
                            # A runway exit is not available if another aircraft is already at or just departing from it
                            elif ac.from_to[0] in available_rwy:
                                available_rwy.remove(ac.from_to[0])

                    # If a runway exit is available, spawn the aircraft at a random, available runway exit
                    if len(available_rwy) > 0:
                        ac_id += 1
                        arrival_available = True
                        spawn_rwy = random.choice(available_rwy)
                        ac = Aircraft(ac_id, 'A', spawn_rwy, random.choice(gates), t, nodes_dict)
                        ac.status = "holding"
                        aircraft_lst.append(ac)
                        agent_lst.append(ac)
                    else:
                        print("==No runways available for arrival")
                        arrival_available = False

        else:
            # Follow the scenario by checking whether the current time is present
            for id in scenario.keys():
                if t == scenario[id]['spawn_time']:
                    # Time matches the spawning time, create the aircraft
                    ac = Aircraft(id, scenario[id]['a_d'], scenario[id]['start_node'], scenario[id]['goal_node'],t,nodes_dict)
                    ac.status = "holding"
                    aircraft_lst.append(ac)
                    agent_lst.append(ac)

        # Spawn taxibots
        if t == 0:
            # Check whether the amount of taxibots is possible
            if n_taxibots > len(taxibot_gates):
                raise ValueError('Amount of taxibots is greater than the amount of holding locations.')
            
            alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ' # Use the alphabet for the taxibot IDs
            # Spawn the taxibots
            for taxibot_idx in range(n_taxibots):
                tb = Taxibot(alphabet[taxibot_idx], taxibot_gates[taxibot_idx], taxibot_gates[taxibot_idx], nodes_dict)
                taxibot_lst.append(tb)
                agent_lst.append(tb)

        # Have the relevant aircraft request taxibots
        for ac in aircraft_lst:
            if ac.status == "holding" and t % 0.5 == 0:
                ac.request_taxibot(nodes_dict, taxibot_lst, heuristics, t)

        # Run the selected planner
        if planner == "Independent":     
            if t % 0.5 == 0:
                PrioritySolver(agent_lst, t, edges_dict, nodes_dict, heuristics, horizonspan)
                run_independent_planner_taxibots(taxibot_lst, nodes_dict, edges_dict, heuristics, t, agent_lst, [t+k*0.5 for k in range(int(1 + (horizonspan // 0.5)))])
                run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
        elif planner == "Prioritized":
            run_prioritized_planner()
        elif planner == "CBS":
            run_CBS()
        else:
            raise Exception("Planner:", planner, "is not defined.")
        
        # Run updates for the aircraft
        ac_remove = []
        for ac in aircraft_lst: 
            if ac.status == "taxiing": 
                ac.move(dt, t)
            if ac.status == "arrived":
                ac_remove.append(ac)

        # Remove aircraft that have arrived and save relevant results
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
            
            # Log results and delete the aircraft
            sim_results.append(ac_results)
            aircraft_lst.remove(ac)
            agent_lst.remove(ac)

        # Move the taxibots that are taxiing
        for taxibot in taxibot_lst:
            if taxibot.status == 'taxiing, unavailable' or taxibot.status == 'taxiing, available':
                taxibot.move(dt, t)

        # Clear RAM
        if t % 10 == 0:
            gc.collect()
    
        t += dt

    return sim_results