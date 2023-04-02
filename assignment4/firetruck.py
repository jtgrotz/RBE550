import robotics_toolbox_edit
from robotics_toolbox_edit.PRMPlanner import PRMPlanner
import heapq as hq
import numpy as np
import time

#fire truck captain ahab
class fire_truck:

    AOE = 20
    current_location = [0,0]
    goal_location = [0,0]
    completed_path = True
    path = []
    path_index = 0
    search_times = []
    vehicle_length = 5
    vehicle_max_speed = 15

    def __init__(self, start_point,map):
        self.current_location = start_point
        self.map = map
        st = time.process_time()
        self.planner = PRMPlanner(map.binocgrid, 1200, self.vehicle_max_speed)
        self.planner.plan()
        et = time.process_time()
        self.init_time = et-st
        print("PRM Init Time: %f" % self.init_time)

    def find_next_goal(self):
        q = []
        obs = self.map.return_burning_obstacles()

        #go through list and sort by distance to the wumpus
        for ob in obs:
            value = self.euclidean(self.current_location, ob)
            hq.heappush(q,(value,ob))

        #pick a goal and validate non_occupied
        maxes = hq.nsmallest(5,q)
        for m in maxes:
            coord = m[1]
            p0 = coord[0] * self.map.pixel_res + self.map.pixel_res/2
            p1 = coord[1] * self.map.pixel_res + self.map.pixel_res/2
            good_point = self.check_neighbor_validity(p0,p1)
            if good_point is not None:
                self.goal_location = [good_point[0], good_point[1]]
                return True
            
        return False

    #returns boolean to say if path was generated or not
    def generate_path(self):
        #use prm to generate a path
        #TODO do kinematic local planning
        st = time.process_time()
        path = self.planner.query(start = self.current_location,goal = self.goal_location)
        et = time.process_time()
        self.search_times.append(et-st)
        print("PRM query Time: %f" % (et-st))


        if path is not None:
            self.path = path
            self.path_index = 0
            self.completed_path = False
        else:
            self.completed_path = True
        return True

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
        #trying to get far away from the obstacle
        x_delta = [self.vehicle_length*2, -self.vehicle_length*2, 0, 0]
        y_delta = [0, 0, -self.vehicle_length*2, self.vehicle_length*2]
        #buffer to get away from edge of map, which always causes problems.
        max_height = self.map.height*self.map.pixel_res-20
        max_width = self.map.width*self.map.pixel_res-20
        #max_height = 235
        #max_width = 235

        for i in range(len(x_delta)):
            nx = p1+x_delta[i]
            ny = p2+y_delta[i]
            if nx < max_width or nx >= 0 or ny < max_height or ny > 0:
                if self.map.check_world_points([ny,nx]) == 0:
                    return (nx,ny)
        
        return None
    
    #for plotting, converts robot to world coordinates
    def current_world_location(self):
        return self.current_location
    
    #turns path into a numpy array and translates to world points
    def world_path(self):
        return np.array(self.path)
