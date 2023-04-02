import a_star as ast
import obstacle_field
import heapq as hq
import numpy as np
import time

class wumpus:
    AOE = 10
    current_location = [0,0]
    goal_location = [0,0]
    completed_path = True
    path = []
    path_index = 0
    search_times = []

    def __init__(self, start_point,map):
        self.current_location = start_point
        self.map = map
        st = time.process_time()
        self.planner = ast.a_star(map, 0.7)
        et = time.process_time()
        self.init_time = et-st
        print("A* Init Time: %f" % self.init_time)


    #finds the next goal for the wumpus
    def find_next_goal(self):
        q = []
        obs = self.map.return_non_burning_obstacles()

        #go through list and sort by distance to the wumpus
        for ob in obs:
            value = self.euclidean(self.current_location, ob)
            hq.heappush(q,(value,ob))

        #pick a goal and validate non_occupied
        maxes = hq.nlargest(5,q)
        for m in maxes:
            coord = m[1]
            p0 = coord[0]
            p1 = coord[1]
            good_point = self.check_neighbor_validity(p0,p1)
            if good_point is not None:
                self.goal_location = [good_point[0], good_point[1]]
                return True
            
        return False

    #returns boolean to say if path was generated or not
    def generate_path(self):
        #use astar to generate a path
        st = time.process_time()
        status = self.planner.search(self.current_location,self.goal_location)
        et = time.process_time()
        self.search_times.append(et-st)
        print("A* Query Time: %f" % (et-st))


        if status == True:
            self.path = self.planner.path
            self.path_index = 0
            self.completed_path = False
        else:
            self.completed_path = True
        return status

    #sets robot state to next path point
    def next_path_point(self):
        #if path is complete set path state, and dont move along path
        if self.path_index >= len(self.path)-1:
            self.completed_path = True
        else:
            self.path_index += 1
            self.current_location = self.path[self.path_index]

        return self.path[self.path_index]
    
    #used for goal setting finds distance between points
    def euclidean(self,p1,p2):
        return ((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1]-p2[1])*(p1[1]-p2[1]))

    #ensures that the goal isn't out of bounds
    def check_neighbor_validity(self,p1,p2):
        x_delta = [1, -1, 0, 0]
        y_delta = [0, 0, -1, 1]
        max_height = self.map.height
        max_width = self.map.width

        for i in range(len(x_delta)):
            nx = p1+x_delta[i]
            ny = p2+y_delta[i]
            if nx < max_width or nx >= 0 or ny < max_height or ny > 0:
                if self.map.check_cell([nx,ny]) == 0:
                    return (nx,ny)
        
        return None
    
    #for plotting, converts robot to world coordinates
    def current_world_location(self):
        wx = (self.current_location[0]*self.map.pixel_res)+(self.map.pixel_res/2)
        wy = (self.current_location[1]*self.map.pixel_res)+(self.map.pixel_res/2)
        return [wx,wy]
    
    #turns path into a numpy array and translates to world points
    def world_path(self):
        arr = (np.array(self.path)*self.map.pixel_res)+(self.map.pixel_res/2)
        arr = np.flip(arr,1)
        return arr
