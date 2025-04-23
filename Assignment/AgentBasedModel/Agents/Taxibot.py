from Functionality.single_agent_planner import simple_single_agent_astar
import numpy as np


class Taxibot(object):
    """Taxibot class, should be used in the creation of new taxibots."""

    def __init__(self, taxibot_id, spawn_node, holding_location, nodes_dict):
        """Initialisation of a taxibot object.

        Args:
            taxibot_id (int or str): Unique ID for this taxibot
            spawn_node (int): Spawn node node_id of the taxibot
            holding_location (int): Holding node node_id of the taxibot
            nodes_dict (dict): Dictionary of the nodes of the airport map
        """        
        
        # Fixed parameters
        self.speed = 1                              # How much taxibot moves per unit of t
        self.id = taxibot_id                        # Taxibot ID
        self.start = spawn_node                     # Spawn node_id
        self.holding_location = holding_location    # Holding location node_id
        self.nodes_dict = nodes_dict                # Set the nodes dictionary as a class property

        # Route related
        self.status = "holding" # Initialize status
        self.path_to_goal = []  # Planned path from current location to the goal node
        self.from_to = [0,0]    # Edge currently being travelled
        self.constraints = []   # Log of constraints

        # State related
        self.idle = True                                    # Initialize idling to true, used for persistance on the map
        self.Goal_AC = None                                 # Initialize the goal aircraft to None
        self.heading = 0                                    # Initialize the heading to north
        self.goal_node = self.holding_location              # Initialize the goal to the holding location
        self.position = nodes_dict[self.start]["xy_pos"]    # (x,y) position on the map

        # Replanning related
        self.last_node = None   # Last visited node
        self.replan = False     # Boolean, whether replanning or not


    def get_heading(self, xy_start, xy_next):
        """Determines heading of a taxibot based on a start and end xy position. Heading is stored in self.heading.

        Args:
            xy_start (tuple): tuple with (x,y) position of start node
            xy_next (tuple): tuple with (x,y) position of next node
        """        
        
        if xy_start[0] == xy_next[0]: #moving up or down
            if xy_start[1] > xy_next[1]: #moving down
                heading = 180
            elif xy_start[1] < xy_next[1]: #moving up
                heading = 0
            else:
                heading=self.heading

        elif xy_start[1] == xy_next[1]: #moving right or left
            if xy_start[0] > xy_next[0]: #moving left
                heading = 90
            elif xy_start[0] < xy_next[0]: #moving right
                heading = 270
            else:
                heading=self.heading
        else: 
            raise Exception("Invalid movement")
    
        self.heading = heading


    def move(self, dt, t): 
        """Moves an aircraft between from_node and to_node and checks if to_node or goal is reached.

        Args:
            dt (float): Size of the timestep
            t (float): Current time
        """          
        
        #Determine nodes between which the ac is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        xy_from = self.nodes_dict[from_node]["xy_pos"] #xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"] #xy position of to node
        distance_to_move = self.speed*dt #distance to move in this timestep
  
        #Update position with rounded values
        x = xy_to[0] - xy_from[0]
        y = xy_to[1] - xy_from[1]
        norm = np.sqrt(x**2 + y**2)
        if norm != 0:
            x_normalized = x / norm
            y_normalized = y / norm
        else:
            x_normalized = 0
            y_normalized = 0
        posx = round(self.position[0] + x_normalized * distance_to_move, 2) # round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move, 2) # round to prevent errors
        self.position = (posx, posy)
        self.get_heading(xy_from, xy_to)	

        #Check if goal is reached or if to_node is reached
        if self.position == xy_to and self.path_to_goal[0][1] == t+dt: #If with this move its current to node is reached
            if self.position == self.nodes_dict[self.goal]["xy_pos"]: #if the final goal is reached
                self.status = "arrived"
                print("\nAircraft", self.id, "has arrived at its destination at t=", t)
                if self.arrival_time == None:
                    self.arrival_time = np.ceil(t * 2) / 2  #rounds up to half a second always

            else:  #current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]
                
                new_from_id = self.from_to[1] #new from node
                new_next_id = self.path_to_goal[0][0] #new to node

                if new_from_id != self.from_to[0]:
                    self.last_node = self.from_to[0]
                
                self.from_to = [new_from_id, new_next_id] #update new from and to node


    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """Plans a path to the goal node. Traffic is not taken into account during planning.
        If planning succeeds, it is logged in self.path_to_goal.

        Args:
            nodes_dict (dict): Dictionary of the nodes
            edges_dict (dict): Dictionary of the edges with weights. Unused but included for future implementation
            heuristics (dict): Dictionary of the heuristics
            t (float): Current time
        """        

        if self.status == "taxiing":
            start_node = self.start # Node from which planning should be done
            goal_node = self.goal # Node to which planning should be done
            
            # When replanning, use the current node as the begin node of the path
            if self.replan == True:
                success, path = simple_single_agent_astar(nodes_dict, self.from_to[0], goal_node, heuristics, self.id, current_time=t, constraints=self.constraints)
                self.replan = False

            # When not replanning, use the start node as the begin node of the path
            if self.replan == False:
                success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, self.id, current_time=t, constraints=self.constraints)
        
            if success:
                # Set the path
                self.path_to_goal = path[1:]
                # Update the next node
                next_node_id = self.path_to_goal[0][0]
                self.from_to = [path[0][0], next_node_id]

                # Store the departure time and ideal arrival time if they aren't yet logged
                if self.departure_time == None:
                    self.departure_time = t
                if self.ideal_arrival_time == None:
                    self.ideal_arrival_time = path[-1][1]

            # Raise an error if no solution is found
            else:
                raise Exception("No solution found for", self.id)
            
            # Check the path timing
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")


    def broadcast_next_nodes(self, horizon):
        """Find and broadcasts the next nodes when requested to the central location.

        Args:
            horizon (list): Timestamps of the horizon

        Returns:
            list: Next steps of the aircraft within the horizon
        """        

        ac_nextsteps = [step[0] for step in self.path_to_goal if step[1] in horizon]
        extra_steps = {95.0: 4.0, 96.0: 5.0, 92.0: 30.0, 93.0: 31.0, 94.0: 32.0, 99.0: 29.0, 100.0: 33.0}

        # Append the current node in the beginning of ac_nextsteps
        ac_nextsteps.insert(0, self.from_to[0])
        
        if len(ac_nextsteps) == 3:
            ac_nextsteps.append(ac_nextsteps[1])

        if len(ac_nextsteps) == 2:
            ac_nextsteps.append(ac_nextsteps[0])
            if extra_steps.get(ac_nextsteps[0]) != None:
                ac_nextsteps.append(extra_steps[ac_nextsteps[0]])

        # Backfill the horizon with None
        ac_nextsteps.extend([None]*(3-len(ac_nextsteps)))

        return ac_nextsteps


    def conflict_detection(self, agent_lst, horizon, t, edges_dict, nodes_dict, heuristics, visionrange=3):      
        """Detects if there is a conflict between two agents.

        Args:
            agent_lst (list): List containing all the agents
            horizon (list): Timestamps of the horizon
            t (float): Current time
            edges_dict (dict): Dictionary of the edges with weights. Unused but included for future implementation
            nodes_dict (dict): Dictionary of the nodes
            heuristics (dict): Dictionary of the heuristics
            visionrange (int, optional): Conflict vision range of the agent. Defaults to 3.
        """        

        #Define own next 3 steps and request the next 3 steps of all other aircrafts that are taxxiing

        # Initialize conflict detection
        dummynode = 1000    # Create a dummy node that is not in the actual network
        other_paths = {}    # Dictionary for the node paths of other agents
        other_edges = {}    # Dictionary for the edge paths of other agents
        Agents_checked = [] # List of checked agents

        # Obtain the next steps of the agent itself and create a dummied list where None is substituted for the dummy node
        own_nextsteps = self.broadcast_next_nodes(horizon)
        own_nextsteps_d = [step if step != None else dummynode for step in own_nextsteps]

        # Obtain the travelled edges within the horizon
        own_nextsteps_d.append(dummynode)
        own_nextedges = [sorted((own_nextsteps_d[tau], own_nextsteps_d[tau+1])) for tau in range(len(horizon))]

        # Set the first step to None to avoid crashes during collision
        own_nextsteps[0] = None

        # Obtain the paths of agents within range
        for agent in agent_lst:
            # Calculate the distance to the agent being checked
            position_agent = agent.position
            position_self = self.position
            distance = np.sqrt((position_agent[0]-position_self[0])**2 + (position_agent[1]-position_self[1])**2)

            if distance > visionrange:
                pass
            elif agent.id == self.id:
                pass
            elif agent.status == "taxiing" or agent.status == "taxiing, available" or agent.status == "taxiing, unavailable":
                # Obtain the node path and the edge path
                other_nextsteps = agent.broadcast_next_nodes(horizon)
                other_paths[agent] = other_nextsteps
                other_nextsteps_d = [step if step != None else dummynode for step in other_nextsteps]
                other_nextsteps_d.append(dummynode)
                other_edges[agent] = [sorted((other_nextsteps_d[tau], other_nextsteps_d[tau+1])) for tau in range(len(horizon))]
                
                # Add the agent to the list of checked agents
                Agents_checked.append(agent)

        # Perform conflict detection for all checked agents
        for agent in Agents_checked:
            for tau in range(len(horizon)):
                # Check node conflicts
                if own_nextsteps[tau] != None and own_nextsteps[tau] == other_paths[agent][tau]:
                    # Perform conflict resolution
                    Conflicted_node = own_nextsteps[tau]
                    conflict_time = horizon[tau]
                    self.Conflict_resolution(agent, t, edges_dict, nodes_dict, [Conflicted_node], conflict_time, heuristics, agent_lst, horizon)

                # Check edge conflicts
                if dummynode not in own_nextedges[tau] and own_nextedges[tau] == other_edges[agent][tau]:
                    # Perform conflict resolution
                    Conflicted_edge = own_nextedges[tau]
                    conflict_time = horizon[tau]
                    self.Conflict_resolution(agent, t, edges_dict, nodes_dict, Conflicted_edge, conflict_time, heuristics, agent_lst, horizon)


    def Conflict_resolution(self, conflicted_agent, t, edges_dict, nodes_dict, conflicted_node, conflict_time, heuristics, agent_lst, horizon):
        """Resolves the conflict between two aircrafts.

        Args:
            conflicted_agent (Agent): Agent object that has a conflict
            t (float): Current time
            edges_dict (dict): Dictionary of the edges with weights. Unused but included for future implementation
            nodes_dict (dict): Dictionary of the nodes
            conflicted_node (list): List containing the conflicted node or nodes contained in the conflicted edge
            conflict_time (float): Time of conflict
            heuristics (dict): Dictionary of the heuristics
            agent_lst (list): List containing all the agents
            horizon (list): Timestamps of the horizon
        """       

        # Obtain the priority of the agent itself and the conflicted agent
        self_priority = self.determine_prioritylevel(t, edges_dict)
        conflicted_priority = conflicted_agent.determine_prioritylevel(t, edges_dict)

        # If the priority level is higher than the conflicted agent, do nothing
        if self_priority > conflicted_priority:
            pass
        
        # Otherwise, resolve the conflict
        else:
            # Node conflict
            if len(conflicted_node) == 1:
                # Add a constraint
                self.constraints.append({'agent': self.id, 'node_id': conflicted_node, 'timestep': conflict_time, 'positive': False})

            # Edge conflict
            else:
                # Add a constraint for the connecting nodes of the edge
                for node in conflicted_node:
                    # 3 timesteps because of the between nodes
                    for tconfl in [conflict_time, conflict_time+.5, conflict_time+1.]:
                        self.constraints.append({'agent': self.id, 'node_id': [node], 'timestep': tconfl, 'positive': False})

            # Replan
            self.replan = True # Set to true to make sure the planning is based on current location
            self.plan_independent(nodes_dict, edges_dict, heuristics, t)

            # Verify that the resolved route is conflict-free
            self.conflict_detection(agent_lst, horizon, t, edges_dict, nodes_dict, heuristics)


    def Hold_position(self, t, heuristics):
        """Updates the path continue holding.

        Args:
            t (float): Current time
            heuristics (dict): Dictionary of the heuristics. Unused but included for future implementation
        """        
        if self.from_to[1] != self.holding_location:
            raise ValueError(f'Taxibot {self.id} told to remain at the holding position, but it is not yet at the holding position.')
        
        # Update the path & goal node
        self.goal_node = self.holding_location
        self.path_to_goal = [(self.holding_location, t+0.5), (self.holding_location, t+1), (self.holding_location, t+1.5)]
        self.from_to = [self.holding_location, self.holding_location]


    def Follow_AC(self, t, heuristics):
        """Locks the taxibot position to the goal aircraft.

        Args:
            t (float): Current time
            heuristics (dict): Dictionary of the heuristics. Unused but included for future implementation
        """        
        self.position = self.Goal_AC.position
    

    def Taxi_to_holding(self, t, nodes_dict, edges_dict, heuristics, agent_lst, horizon):
        """Update the taxibot path to head to the holding position.

        Args:
            t (float): Current time
            nodes_dict (dict): Dictionary of the nodes
            edges_dict (dict): Dictionary of the edges with weights. Unused but included for future implementation
            heuristics (dict): Dictionary of the heuristics
            agent_lst (list): List containing all the agents
            horizon (list): Timestamps of the horizon
        """        
        # Plan the route to the holding position and verify it's conflict free
        self.plan_independent(nodes_dict, edges_dict, heuristics, t)
        self.conflict_detection(agent_lst, horizon, t, edges_dict, nodes_dict, heuristics)

    
    def Taxi_to_AC(self, t, nodes_dict, edges_dict, heuristics, agent_lst, horizon):  
        """Update the taxibot path to head to the goal aircraft.

        Args:
            t (float): Current time
            nodes_dict (dict): Dictionary of the nodes
            edges_dict (dict): Dictionary of the edges with weights. Unused but included for future implementation
            heuristics (dict): Dictionary of the heuristics
            agent_lst (list): List containing all the agents
            horizon (list): Timestamps of the horizon
        """        
        # Plan the route to the goal aircraft and verify it's conflict free
        self.plan_independent(nodes_dict, edges_dict, heuristics, t)
        self.conflict_detection(agent_lst, horizon, t, edges_dict, nodes_dict, heuristics)
    
        
    def determine_prioritylevel(self, t, edges_dict, weights = {'routelength': -1,
                                                                'movementoptions': -1,
                                                                }):
        """Determines the priority level of the agent.

        Args:
            t (float): Current time
            edges_dict (dict): Dictionary of the edges
            weights (dict, optional): Weights for the priority level factors. Defaults to {'routelength': -1, 'movementoptions': -1}.

        Returns:
            float: Priority level
        """     
        # Determine the amount of movement options, taking into account dead ends of the map
        dead_ends = [1,2,34,35,36,92,93,94,95,96,97,98,99,100,101,102]
        movementoptions = sum([1 for edge in edges_dict if edge[0] == self.from_to[0]])

        if int(self.from_to[0]) in dead_ends or int(self.from_to[1]) in dead_ends:
            movementoptions = 1

        if movementoptions <= 1:
            prioritylevel = 1000

        # If the taxibot is roaming without a target, set the priority level to -1000
        elif self.status == "taxiing, available":
            prioritylevel = -1000

        # Determine the remaining path length in time
        remaining_path = self.path_to_goal[-1][1] - t

        # Calculate the priority level
        prioritylevel = sum([
                            movementoptions * weights['movementoptions'],
                            remaining_path * weights['routelength']
                            ])

        return prioritylevel