from matplotlib import pyplot
import numpy as np

class simulator:
    sim_done = False
    timesteps = 0
    global_time = 0

    def __init__(self,map,robot1,robot2,time_interval,number_of_timesteps):
        self.map = map
        self.robotA = robot1 #Astar robot, enemy, fire starter
        self.robotB = robot2 #prm robot, hero, fire truck
        self.timesteps = number_of_timesteps
        self.time_interval = time_interval
        self.fig = pyplot.figure()
        self.ax = self.fig.add_subplot(111)
        self.map.ocgrid.plot(block=False, cmap='nipy_spectral',vmin=0,vmax=40)
        self.extinguishing_time = 0

    def increment_time(self):
        print(self.global_time)
        #spread fires
        self.map.increment_fire()
        #check wumpus state and start fires
        if self.robotA.completed_path == True:
            curr_point = self.robotA.current_location
            self.map.spread_fire(curr_point[0],curr_point[1],self.robotA.AOE)
            self.robotA.completed_path = False
            #find next goal for wumpus
            next_goal_A = self.robotA.find_next_goal()
            path_A = self.robotA.generate_path()
        #move wumpus
        self.robotA.next_path_point()

        #check firetruck state and extingush fire or find new goal
        if self.robotB.completed_path == True:
            curr_point = self.robotB.current_location
            grid_curr = self.map.coord_to_grid(curr_point)
            if self.extinguishing_time < 6:
                self.map.extinguish_fires(grid_curr[0],grid_curr[1],self.robotA.AOE)
                self.extinguishing_time += 1
            else:
                self.extinguishing_time = 0
                self.robotB.completed_path = False
                next_goal_B = self.robotB.find_next_goal()
                path_B_status = self.robotB.generate_path()
        else: #move firetruck
            self.robotB.next_path_point()

        #plot everything
        #map, wumpus goal, path, and current state, firetruck goal, path and current state. 
        self.visualizer()

        #increment counter
        self.global_time += self.time_interval
        #check if done
        if self.global_time > self.timesteps:
            print('all_done')
            self.sim_done = True

    def visualizer(self):
        self.ax.clear()
        #cmap and v chosen for asthetic reasons
        self.map.ocgrid.plot(block=False, cmap='nipy_spectral',vmin=0,vmax=100)
        self.ax.grid(False)
        #Robot A current position and path
        RA_current_point = self.robotA.current_world_location()
        self.ax.plot(RA_current_point[1],RA_current_point[0], 'o:r')
        path_A = self.robotA.world_path()
        self.ax.plot(*path_A.T, ':y')
        #robot B current position and path
        RB_current_point = self.robotB.current_world_location()
        self.ax.plot(RB_current_point[0],RB_current_point[1], 'x:g')
        path_B = np.array(self.robotB.path)
        self.ax.plot(*path_B.T, ':w')
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        
        