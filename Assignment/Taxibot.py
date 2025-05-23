from single_agent_planner_v2 import simple_single_agent_astar
import math

class Taxibot(object):
    """Taxibot class, should be used in the creation of new taxibots."""

    def __init__(self, taxibot_id, spawn_node, holding_location, nodes_dict):
        """
        Initialisation of taxibot object.
        INPUT:
            - taxibot_id: [int] unique id for this taxibot
            - spawn_node: node_id of start node
            - holding_location: node_id of standard holding locationg
            - nodes_dict: copy of the nodes_dict
        """
        
        #Fixed parameters
        self.speed = 1                              #how much taxibot moves per unit of t
        self.id = taxibot_id                        #taxibot_id
        self.start = spawn_node                     #spawn_node_id
        self.holding_location = holding_location    #holding_location_node_id
        self.nodes_dict = nodes_dict                #keep copy of nodes dict
        self.status = "holding"                     #begin status
        #self.planning_status = None                 #ensure we dont plan when we are taxxiing already
        self.idle = True                            #If idling, to keep aircraft on the map without a driving plan
        self.Goal_AC = None

        #Route related
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [0,0]
        self.constraints = []

        #State related
        self.heading = 0
        self.goal_node = self.holding_location
        self.position = self.nodes_dict[self.start]["xy_pos"]
        self.goal = self.goal_node
        self.delay = 0 #TODO: Create delay logic

        #Replanning related
        self.last_node = None
        self.replan = False

    def get_heading(self, xy_start, xy_next):
        """
        Determines heading of an aircraft based on a start and end xy position.
        INPUT:
            - xy_start = tuple with (x,y) position of start node
            - xy_next = typle with (x,y) position of next node
        RETURNS:
            - heading = heading of aircraft in degrees
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

    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing aircraft assuming that it knows the entire layout.
        Other traffic is not taken into account.
        INPUT:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
        """
        """
        success, path = simple_single_agent_astar(nodes_dict, self.from_to[0], self.goal_node, heuristics, self.id, current_time=t, constraints=self.constraints, Reversing_Possible=True)
        #Make sure the path is broadcasted to some central location

        if success:
            self.path_to_goal = path[1:]
            next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
            self.from_to = [path[0][0], next_node_id]
            #print("Path AC", self.id, ":", path)
        else:
            raise Exception("No solution found for", self.id)
        
        #Check the path
        if path[0][1] != t:
            raise Exception("Something is wrong with the timing of the path planning")
        """
        start_node = self.from_to[0] #node from which planning should be done
        goal_node = self.goal_node #node to which planning should be done
            
        if self.replan == False:
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, self.id, current_time=t, constraints=self.constraints)
        if self.replan == True:
            # print("______________Replanning for", self.id, "with", len(self.constraints), "constraints:", self.constraints)
            success, path = simple_single_agent_astar(nodes_dict, self.from_to[0], goal_node, heuristics, self.id, current_time=t, constraints=self.constraints)
            self.replan = False
        #Make sure the path is broadcasted to some central location

        if success:
            self.path_to_goal = path[1:]
            next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
            self.from_to = [path[0][0], next_node_id]
            #print("Path AC", self.id, ":", path)
        else:
            raise Exception("No solution found for", self.id)
        
        #Check the path
        if path[0][1] != t:
            raise Exception("Something is wrong with the timing of the path planning")

    def move(self, dt, t):   
        """
        Moves an aircraft between from_node and to_node and checks if to_node or goal is reached.
        INPUT:
            - dt = 
            - t = 
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
        norm = math.sqrt(x**2 + y**2)
        if norm != 0:
            x_normalized = x / norm
            y_normalized = y / norm
        else:
            x_normalized = 0
            y_normalized = 0
        posx = round(self.position[0] + x_normalized * distance_to_move ,2) #round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move ,2) #round to prevent errors
        self.position = (posx, posy)  
        self.get_heading(xy_from, xy_to)	

        #Check if goal is reached or if to_node is reached
        if self.position == xy_to and self.path_to_goal[0][1] == t+dt: #If with this move its current to node is reached
            if self.position == self.nodes_dict[self.goal_node]["xy_pos"]: #if the final goal is reached
                self.status = "arrived"

            else:  #current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]
                
                new_from_id = self.from_to[1] #new from node
                new_next_id = self.path_to_goal[0][0] #new to node

                if new_from_id != self.from_to[0]:
                    self.last_node = self.from_to[0]
                
                self.from_to = [new_from_id, new_next_id] #update new from and to node

    
    def Hold_position(self, t, heuristics):
        self.goal_node = self.holding_location
        #Set the path of the taxibot to its holding position for the next timestep
        self.path_to_goal = [(self.holding_location, t+0.5), (self.holding_location, t+1), (self.holding_location, t+1.5)]
        self.from_to = [self.holding_location, self.holding_location]

    def Follow_AC(self, t, heuristics):
        self.position = self.Goal_AC.position
        return
    
    def Taxi_to_holding(self, t, edges_dict, heuristics, agent_lst, horizon):
        # print("Im going home")
        self.goal_node = self.holding_location
        # self.start = self.from_to[0]
        self.from_to[0] = self.Goal_AC.goal
        self.plan_independent(self.nodes_dict, edges_dict, heuristics, t)
        #once it has planned a route, set the status to taxxiing
        # self.status = "taxiing, available"
        self.conflict_detection(agent_lst, horizon, t, edges_dict, self.nodes_dict, heuristics)
        return
    
    def Taxi_to_AC(self, t, edges_dict, heuristics, agent_lst, horizon):  
        self.plan_independent(self.nodes_dict, edges_dict, heuristics, t)
        # self.status = "taxiing, unavailable"
        self.conflict_detection(agent_lst, horizon, t, edges_dict, self.nodes_dict, heuristics)
        return

    def broadcast_next_nodes(self, horizon):
        """
        Find and broadcasts the next nodes when requested to the central location.
        """

        ac_nextsteps = [step[0] for step in self.path_to_goal if step[1] in horizon]
        extra_steps = {95.0: 4.0, 96.0: 5.0, 92.0: 30.0, 93.0: 31.0, 94.0: 32.0, 99.0: 29.0, 100.0: 33.0}

        #append the current node in the beginning of ac_nextsteps
        ac_nextsteps.insert(0, self.from_to[0])
        
        if len(ac_nextsteps) == 3:
            ac_nextsteps.append(ac_nextsteps[1])

        if len(ac_nextsteps) == 2:
            ac_nextsteps.append(ac_nextsteps[0])
            if extra_steps.get(ac_nextsteps[0]) != None:
                ac_nextsteps.append(extra_steps[ac_nextsteps[0]])
            else:
                ac_nextsteps.append(None)
                
        if len(ac_nextsteps) == 1:
            ac_nextsteps.append(None)
            ac_nextsteps.append(None)
            ac_nextsteps.append(None)

        return ac_nextsteps

    def conflict_detection(self, agent_lst, horizon, t, edges_dict,nodes_dict, heuristics):
        """
        Detects if there is a conflict between two agents.
        """
        #Define own next 3 steps and request the next 3 steps of all other aircrafts that are taxxiing
        horizon_length = len(horizon)
        dummynode = 1000
        other_paths = {}
        other_edges = {}
        Agents_checked = []
        own_nextsteps = self.broadcast_next_nodes(horizon)
        own_nextsteps_d = [step if step != None else dummynode for step in own_nextsteps]
        own_nextsteps_d.append(dummynode)
        own_nextedges = [sorted((own_nextsteps_d[tau], own_nextsteps_d[tau+1])) for tau in range(horizon_length)]
        for agent in agent_lst:
            #only check aicraft with certain distance to own aircraft
            position_agent = agent.position
            position_self = self.position
            distance = math.sqrt((position_agent[0]-position_self[0])**2 + (position_agent[1]-position_self[1])**2)
            if distance < 3 and (agent.status == "taxiing" or agent.status == "taxiing, available" or agent.status == "taxiing, unavailable" or agent.status == 'holding') and agent.id != self.id:
                other_nextsteps = agent.broadcast_next_nodes(horizon)
                other_paths[agent] = other_nextsteps
                other_nextsteps_d = [step if step != None else dummynode for step in other_nextsteps]
                other_nextsteps_d.append(dummynode)
                other_edges[agent] = [sorted((other_nextsteps_d[tau], other_nextsteps_d[tau+1])) for tau in range(horizon_length)]
                Agents_checked.append(agent)

        for agent in Agents_checked:
            own_nextsteps[0] = None #To avoid crashing the code at a collision, we set own current node to None
            for tau in range(horizon_length):
                if own_nextsteps[tau] != None and own_nextsteps[tau] == other_paths[agent][tau]:
                    Conflicted_agent = agent
                    Conflicted_node = own_nextsteps[tau]
                    conflict_time = horizon[tau]
                    # print("______Conflict detected between", self.id, "and", agent.id, "at node", int(Conflicted_node),". Now (t=", t, ") starting conflict resolution.")
                    self.Conflict_resolution(Conflicted_agent, t, edges_dict, nodes_dict, [Conflicted_node], conflict_time, heuristics, agent_lst, horizon)

                if dummynode not in own_nextedges[tau] and own_nextedges[tau] == other_edges[agent][tau]:
                    Conflicted_agent = agent
                    Conflicted_edge = own_nextedges[tau]
                    conflict_time = horizon[tau]
                    # print("______Conflict detected between", self.id, "and", agent.id, "at edge", Conflicted_edge,". Now starting conflict resolution.")
                    self.Conflict_resolution(Conflicted_agent, t, edges_dict, nodes_dict, Conflicted_edge, conflict_time, heuristics, agent_lst, horizon)

    def Conflict_resolution(self, conflicted_agent, t, edges_dict, nodes_dict, conflicted_node, conflict_time, heuristics, agent_lst, horizon):
        """
        Resolves the conflict between two aircrafts.
        """

        #find own priority level and that of the conflicted aircraft
        self_priority = self.determine_prioritylevel(t, edges_dict)
        conflicted_priority = conflicted_agent.determine_prioritylevel(t, edges_dict)

        if self_priority > conflicted_priority:
            #print("__________Priority of", self.id, "is higher than", conflicted_agent.id, ". No action needed.")
            return
            
        if conflicted_priority > self_priority or self_priority == conflicted_priority:
            #Add constraint to the conflicted aircraft
            if len(conflicted_node) > 1:
                for node in conflicted_node:
                    for tconfl in [conflict_time, conflict_time+.5, conflict_time+1.]:
                        self.constraints.append({'agent': self.id, 'node_id': [node], 'timestep': tconfl, 'positive': False})

            else:
                self.constraints.append({'agent': self.id, 'node_id': conflicted_node, 'timestep': conflict_time, 'positive': False})

            self.replan = True #Set to true to make sure the planning is based on current location
            self.plan_independent(nodes_dict, edges_dict, heuristics, t)
            self.conflict_detection(agent_lst, horizon, t, edges_dict, nodes_dict, heuristics)
            #self.constraints = [] #clear the constraints after they are used
            return
        return 

        
    def determine_prioritylevel(self, t, edges_dict, weights = {'routelength': -1,
                                            'movementoptions': -1,
                                            }):
        
        dead_ends = [1,2,34,35,36,92,93,94,95,96,97,98,99,100,101,102]
        movementoptions = sum([1 for edge in edges_dict if edge[0] == self.from_to[0]])

        if int(self.from_to[0]) in dead_ends or int(self.from_to[1]) in dead_ends:
            movementoptions = 1

        remaining_path = self.path_to_goal[-1][1] - t #remaining path length in time units

        prioritylevel = sum([
                            movementoptions * weights['movementoptions'],
                            remaining_path * weights['routelength']
                            ])


        if movementoptions <= 1:
            prioritylevel = 1000
        elif self.status == "taxiing, available":
            prioritylevel = -1000
        
        return prioritylevel
        