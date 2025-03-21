from single_agent_planner_v2 import simple_single_agent_astar
import math

class Aircraft(object):
    """Aircraft class, should be used in the creation of new aircraft."""

    def __init__(self, flight_id, a_d, start_node, goal_node, spawn_time, nodes_dict):
        """
        Initalisation of aircraft object.
        INPUT:
            - flight_id: [int] unique id for this aircraft
            - a_d: [str] "a" for arrival flight and "d" for departure flight
            - start_node: node_id of start node
            - goal_node: node_id of goal node
            - spawn_time: spawn_time of a/c 
            - nodes_dict: copy of the nodes_dict
        """
        
        #Fixed parameters
        self.speed = 1         #how much a/c moves per unit of t
        self.id = flight_id       #flight_id
        self.type = a_d           #arrival or departure (A/D)
        self.spawntime = spawn_time #spawntime
        self.start = start_node   #start_node_id
        self.goal = goal_node     #goal_node_id
        self.nodes_dict = nodes_dict #keep copy of nodes dict
        
        #Route related
        self.status = "holding"
        self.path_to_goal = [] #planned path left from current location
        self.from_to = [0,0]
        self.constraints = []

        #State related
        self.heading = 0
        self.position = (0,0) #xy position on map
        self.delay = 0

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
        posx = round(self.position[0] + x_normalized * distance_to_move, 2) # round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move, 2) # round to prevent errors
        self.position = (posx, posy)
        self.get_heading(xy_from, xy_to)	

        #Check if goal is reached or if to_node is reached
        if self.position == xy_to and self.path_to_goal[0][1] == t+dt: #If with this move its current to node is reached
            if self.position == self.nodes_dict[self.goal]["xy_pos"]: #if the final goal is reached
                self.status = "arrived"

            else:  #current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]
                
                new_from_id = self.from_to[1] #new from node
                new_next_id = self.path_to_goal[0][0] #new to node

                if new_from_id != self.from_to[0]:
                    self.last_node = self.from_to[0]
                
                self.from_to = [new_from_id, new_next_id] #update new from and to node

    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing aircraft assuming that it knows the entire layout.
        Other traffic is not taken into account.
        INPUT:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
        """
        
        if self.status == "taxiing":
            start_node = self.start #node from which planning should be done
            goal_node = self.goal #node to which planning should be done
            
            if self.replan == False:
                success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, self.id, current_time=t, constraints=self.constraints)
            if self.replan == True:
                print("______________Replanning for", self.id, "with", len(self.constraints), "constraints:", self.constraints)
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
            
    def broadcast_next_nodes(self, horizon_length):
        """
        Find and broadcasts the next nodes when requested to the central location.
        """
        ac_nextsteps = [step[0] for step in self.path_to_goal if step[1] in horizon_length]
        if len(ac_nextsteps) == 1:
            ac_nextsteps.append(None)
            ac_nextsteps.append(None)
        elif len(ac_nextsteps) == 2:
            ac_nextsteps.append(None)

        return ac_nextsteps

    def conflict_detection(self, aircraft_lst, horizon, t, edges_dict,nodes_dict, heuristics):
        """
        Detects if there is a conflict between two aircraft.
        """
        #Define own next 3 steps and request the next 3 steps of all other aircrafts that are taxxiing
        horizon_length = len(horizon)
        dummynode = 1000
        other_paths = {}
        other_edges = {}
        Aircrafts_checked = []
        ac_nextsteps = self.broadcast_next_nodes(horizon)
        ac_nextsteps_d = [step if step != None else dummynode for step in ac_nextsteps]
        ac_nextsteps_d.append(dummynode)
        ac_nextedges = [sorted((ac_nextsteps_d[tau], ac_nextsteps_d[tau+1])) for tau in range(horizon_length)]
        for ac in aircraft_lst:
            #only check aicraft with certain distance to own aircraft
            position_ac = ac.position
            position_self = self.position
            distance = math.sqrt((position_ac[0]-position_self[0])**2 + (position_ac[1]-position_self[1])**2)
            if distance < 3 and ac.status == "taxiing" and ac.id != self.id:            
                other_nextsteps = ac.broadcast_next_nodes(horizon)
                other_paths[ac] = other_nextsteps
                other_nextsteps_d = [step if step != None else dummynode for step in other_nextsteps]
                other_nextsteps_d.append(dummynode)
                other_edges[ac] = [sorted((other_nextsteps_d[tau], other_nextsteps_d[tau+1])) for tau in range(horizon_length)]
                # other_paths.append(other_nextsteps)
                Aircrafts_checked.append(ac)
                    
        #Check if there is a conflict, #TODO: currently only node based, not passing on other nodes based on heading
        # for i in range(len(Aircrafts_checked)):
        #     for tau in range(timehorizon):
        #         if ac_nextsteps[tau] is not None and ac_nextsteps[tau] == other_paths[i][tau]:
        #             Conflicted_aircraft = Aircrafts_checked[i]
        #             Conflicted_node = ac_nextsteps[tau]
        #             conflict_time = horizon_length[tau]
        #             print("______________Conflict detected between", self.id, "and", Aircrafts_checked[i].id, " at ", Conflicted_node,". Now starting conflict resolution.")
        #             #self.Conflict_resolution(Conflicted_aircraft, t, edges_dict, nodes_dict, Conflicted_node, conflict_time, heuristics)

        for ac in Aircrafts_checked:
            for tau in range(horizon_length):
                if ac_nextsteps[tau] != None and ac_nextsteps[tau] == other_paths[ac][tau]:
                    Conflicted_aircraft = ac
                    Conflicted_node = ac_nextsteps[tau]
                    conflict_time = horizon[tau]
                    print("______Conflict detected between", self.id, "and", ac.id, "at node", int(Conflicted_node),". Now (t=", t, ") starting conflict resolution.")
                    self.Conflict_resolution(Conflicted_aircraft, t, edges_dict, nodes_dict, Conflicted_node, conflict_time, heuristics)

                if dummynode not in ac_nextedges[tau] and ac_nextedges[tau] == other_edges[ac][tau]:
                    Conflicted_aircraft = ac
                    Conflicted_edge = ac_nextedges[tau]
                    conflict_time = horizon[tau]
                    print("______Conflict detected between", self.id, "and", ac.id, "at edge", Conflicted_edge,". Now starting conflict resolution.")
                    self.Conflict_resolution(Conflicted_aircraft, t, edges_dict, nodes_dict, int(Conflicted_edge[1]), conflict_time, heuristics)



    def Conflict_resolution(self, conflicted_aircraft, t, edges_dict, nodes_dict, conflicted_node, conflict_time, heuristics):
        """
        Resolves the conflict between two aircrafts.
        """

        # TODO add conflict resolution for edge conflicts

        #find own priority level and that of the conflicted aircraft
        print("in conflict resultion of ", self.id)
        self_priority = self.determine_prioritylevel(t, edges_dict)
        conflicted_priority = conflicted_aircraft.determine_prioritylevel(t, edges_dict)

        if self_priority > conflicted_priority:
            print("__________Priority of", self.id, "is higher than", conflicted_aircraft.id, ". No action needed.")
            
        if conflicted_priority > self_priority or self_priority == conflicted_priority:
            print("__________Priority of", self.id, "is lower than", conflicted_aircraft.id, ". Will replan.")
            
            #Add constraint to the conflicted aircraft
            self.constraints.append({'agent': self.id, 'node_id': [int(conflicted_node)], 'timestep': conflict_time, 'positive': False})
            self.replan = True #Set to true to make sure the planning is based on current location
            self.plan_independent(nodes_dict, edges_dict, heuristics, t)
            return
        return 

            
    def request_taxibot(self, nodes_dict, taxibot_list, heuristics, t):
        """
        Requests a taxibot for the aircraft.
        """
        #Initialize the list of the traveltimes from each taxibot to the aircraft
        traveltime_list = []
        for taxibot in taxibot_list:
            print("I'm in the for loop in the request function")
            if taxibot.status == "available":
                #Calculate the distance between the taxibot and the aircraft
                taxibot_pos = taxibot.from_to[0]
                print('taxibot position is', taxibot_pos)
                print('Aircraft position is', aircraft_pos)
                print("Available keys in h_values:", list(heuristics.keys()))
                path = simple_single_agent_astar(nodes_dict, taxibot_pos, aircraft_pos, heuristics, t) ### This path should be used by the taxibot to move to aircraft)
                travel_time = path[-1][1] #The final timestep arrival time
                traveltime_list.append(travel_time)
            else:
                traveltime_list.append(10000)
            #TODO Should still add that this travel_time is looked at, and lowest is the taxibot that will be assigned

    def determine_prioritylevel(self, t, edges_dict, weights = {'routelength': -1,
                                            'delay': 2,
                                            'movementoptions': 8,
                                            'pickup': 7
                                            }):

        # print("Determining priority level for", self.id)

        # Taxibots that aren't doing anything have absolute lowest priority
        if self.status == "unassigned":
            return -1000
        
        else:
            return sum([
                        (self.path_to_goal[-1][1] - t)*weights['routelength'], # Remaining route length (Agents with a shorter time to go get a smaller penalty)
                        self.delay*weights['delay'], # Agents that have already experienced delay get higher priority
                        (4-sum([1 for edge in edges_dict if edge[0] == self.from_to[1]]))*weights['movementoptions'], # Amount of connected nodes to the current node, weighs how easy it is to get out of the way
                        (self.status == "pickup") * weights["pickup"] # Taxibots that are picking up an aircraft have higher priority
                        ])

                        # TODO: Maybe change from_to[0] to from_to[1], try it if stuff breaks - Cijsouw

                        # I changed it to from_to[1] because the from_to[0] is the current node, and the from_to[1] is the next node,
                        # this gave better results for the test cases. Not sure if this is implemented completely correctly though - Offringa





