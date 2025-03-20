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
        self.status = "available"                   #begin status

        #State related
        self.heading = 0
        self.goal_node = self.holding_location
        self.position = self.nodes_dict[self.start]["xy_pos"]
        self.delay = 0 #TODO: Create delay logic

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
        
        if self.status == "taxiing":
            start_node = self.start #node from which planning should be done
            goal_node = self.goal #node to which planning should be done
            
            if self.replan == False:
                success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, self.id, current_time=t, constraints=self.constraints)
            if self.replan == True:
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
        x = xy_to[0]-xy_from[0]
        y = xy_to[1]-xy_from[1]
        x_normalized = x / math.sqrt(x**2+y**2)
        y_normalized = y / math.sqrt(x**2+y**2)
        posx = round(self.position[0] + x_normalized * distance_to_move ,2) #round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move ,2) #round to prevent errors
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
            
            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t)
            if success:
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                print("Path AC", self.id, ":", path)
            else:
                raise Exception("No solution found for", self.id)
            
            #Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")
            

    def determine_prioritylevel(self, t, edges_dict, weights = {'routelength': -1,
                                            'delay': 3,
                                            'movementoptions': 5,
                                            'pickup': 7
                                            }):

        # Taxibots that aren't doing anything have absolute lowest priority
        if self.status == "unassigned":
            return -1000
        
        else:
            return sum([
                        (self.path_to_goal[-1][1] - t)*weights['routelength'], # Remaining route length (Agents with a shorter time to go get a smaller penalty)
                        self.delay*weights['delay'], # Agents that have already experienced delay get higher priority
                        (4-sum([1 for edge in edges_dict if edge[0] == self.from_to[0]]))*weights['movementoptions'], # Amount of connected nodes to the current node, weighs how easy it is to get out of the way TODO: May be from_to[1], try it if stuff breaks
                        (self.status == "pickup") * weights["pickup"] # Taxibots that are picking up an aircraft have higher priority
                        ])
        