import numpy as np
import robotics_toolbox_edit.OccGrid as OG
import roboticstoolbox as rtb

#new obstacle field is a occupancy grid and a regular array.
class obstacle_field:

    shape1 = [[0,0],[0,1],[0,2],[0,3]]
    shape2 = [[0,0],[1,0],[0,1],[0,2]]
    shape3 = [[0,0],[0,1],[0,2],[1,1]]
    shape4 = [[0,0],[0,1],[1,1],[1,2]]
    onfire_color = 20
    burned_color = 100
    normal_color = 10
    burning_time = 10


    shapes = [shape1,shape2,shape3,shape4]

    ocgrid = None
    binocgrid = None

    def __init__(self, height, width, pixel_res):
        self.height = height
        self.width = width
        self.shape_size = 4
        self.pixel_res = pixel_res
        self.field = np.zeros((height,width))
        self.obstacles = {}
        self.putting_out_fires = {}
        #self.on_fire = []
        #self.not_onfire = []

    #generates field with given obstacle density.
    def generate_field(self, density):
        shape_size = 4
        #if no blocks need to be placed, separately initiate everything
        if density > 0:
            #determine how many shapes to place in the field
            squares_to_fill = np.floor(self.height*self.width*density)
            number_of_shapes = round(squares_to_fill/shape_size)

            #generate random coordinates for random shape type
            #scale random number by fieldsize
            rand_x_coord = np.ceil(self.width*np.random.rand(number_of_shapes-1,1)).astype(int)-1
            rand_y_coord = np.ceil(self.height*np.random.rand(number_of_shapes-1,1)).astype(int)-1
            rand_coords = [rand_x_coord,rand_y_coord]
            rand_shapes = np.ceil(shape_size*np.random.rand(number_of_shapes-1,1)).astype(int)-1
        else:
            actual_density = 0
            return actual_density

        good_points = 0
        bad_points = 0

     #loop through every point
        for i in range(number_of_shapes-1):
            #see if piece can be placed on board
            point_coords = [rand_x_coord[i],rand_y_coord[i]]
            #finds shape and randomly inverts and/or rotates it
            chosen_shape = self.alter_shape(self.shapes[int(rand_shapes[i])])
            #point placed flag indicates if the piece has been placed
            point_placed = 0

            while (point_placed == 0):
                bad_placement_flag = self.check_bad_placement(point_coords,chosen_shape)
                if bad_placement_flag == 0:
                    #fill the board
                    self.write_to_board(point_coords,chosen_shape)
                    point_placed = 1
                else :
                    point_coords = [np.ceil(self.width*np.random.rand(1))-1,np.ceil(self.height*np.random.rand(1))-1]
        actual_density = (np.sum(self.field))/(self.height*self.width)
        #creates occupancy grid of the proper resolution.
        self.ocgrid = OG.OccupancyGrid(self.field, cellsize=self.pixel_res)
        self.binocgrid = OG.BinaryOccupancyGrid(self.field, cellsize=self.pixel_res)
        return actual_density

    
    #inverts piece
    def invert_piece(self, piece):
        return np.array(piece)*-1

    #rotates the piece in a uniform manner
    def rotate_piece(self, piece):
        y = np.zeros(np.shape(piece))
        for i in range(self.shape_size):
            y[i][0] = piece[i][1]
            y[i][1] = piece[i][0]
        return y

    #checks in the current piece is out of bounds.
    def out_of_bounds(self,point,piece):
        status = 0
        for i in range(self.shape_size):
            x_location = int(piece[i][0]+point[0])
            y_location = int(piece[i][1]+point[1])
            if (x_location >= self.width or x_location <= 0):
                return 1
            if (y_location >= self.height or y_location <= 0):
                return 1
        return 0
        
    #writes given shape onto the board.
    def write_to_board(self, point, piece):
        for i in range(self.shape_size):
            x_location = int(piece[i][0]+point[0])
            y_location = int(piece[i][1]+point[1])
            self.field[x_location][y_location] = self.normal_color
            self.obstacles.update({(x_location,y_location) : 1})
            #self.not_onfire.append(point)
        
    #checks if spaces for the piece have already been filled.
    def already_filled(self, point, piece):
        status = 0 
        for i in range(self.shape_size):
            x_location = int(piece[i][0]+point[0])
            y_location = int(piece[i][1]+point[1])
            if self.field[x_location][y_location] == 1:
                return 1
        return 0

    #checks if the piece is oob or already been filled on the board.
    def check_bad_placement(self, point, piece):
        oob = self.out_of_bounds(point,piece)
        if oob:
            return 1
        else:
            overlap = self.already_filled(point,piece)
            return overlap
            
    #function randomly rotates or inverts piece.
    def alter_shape(self, piece):
        new_shape = piece
        if np.random.rand(1) > 0.5:
            new_shape = self.invert_piece(new_shape)
        if np.random.rand(1) > 0.5:
            new_shape = self.rotate_piece(new_shape)
        return new_shape

    #ensures the set start point is not in an obstacle
    def set_start_point(self, start):
        self.field[start[0]][start[1]] = 0.2

    #ensures the set end point is not in an obstacle
    def set_end_point(self,end):
        self.field[end[0]][end[1]] = 0.3


    def grid_to_coord(self, grid_point):
        return self.ocgrid.g2w(grid_point)

    def coord_to_grid(self, world_point):
        return self.ocgrid.w2g(world_point)

    #negative 1 is extinguished
    def extinguish_fire(self, grid_point):
        p = grid_point
        #p = self.coord_to_grid(world_point)
        self.ocgrid.grid[p[0],p[1]] = self.normal_color
        self.obstacles.update({(p[0],p[1]) : -1})

    #10 or greater is onfire
    def start_fire(self,grid_point):
        self.ocgrid.grid[grid_point[0],grid_point[1]] = 10
        self.obstacles.update({(grid_point[0],grid_point[1]) : 2})

    # 1: regular obstacle, -1 extinguished, 2 burning, 3 burned.
    def increment_fire(self):
        #fix to be better
        obs = self.obstacles.keys()
        for ob in obs:
            # if burning for defined seconds, set to burned state
            if self.ocgrid.grid[ob[0],ob[1]] >= self.burned_color:
                self.obstacles.update({(ob[0],ob[1]) : 3})
            #else if buring for 10 seconds, spread fire and increment time 
            elif self.ocgrid.grid[ob[0],ob[1]] >= self.onfire_color+self.burning_time:
                self.ocgrid.grid[ob[0],ob[1]] += 1
                self.spread_fire(ob[0],ob[1],30)
            # else increment time burning
            elif self.ocgrid.grid[ob[0],ob[1]] >= self.onfire_color:
                self.ocgrid.grid[ob[0],ob[1]] += 1
    #todo function for checking occupancy, function for setting occupancy in OCCgrid.


    #increments fire timer and spreads fires if needed.
    def spread_fire(self,px,py,radius):
        #get keys for the dictionary. 
        obs = list(self.obstacles)
        #check the location of each key in regards to the given point
        for ob in obs:
            #if within radius
            if self.euclidean_distance(px,py,ob[0],ob[1]) <= radius/self.pixel_res:
                if self.obstacles[ob] <= 1: #if not already buring
                    #set on fire (value to 10 and set dictionary.
                    self.obstacles[ob] = 2
                    self.ocgrid.grid[ob[0],ob[1]] = self.onfire_color

    #returns state of cell from dictionary
    def check_state(self, grid_point):
        return self.obstacles[(grid_point[0],grid_point[1])]
    
    #returns state of cell in grid index 
    def check_cell(self, grid_point):
        #hard check because oob check isn't working
        if grid_point[0] > self.width-1:
            grid_point[0] = 49
        if grid_point[1] > self.height-1:
            grid_point[1] = 49

        return self.field[grid_point[0]][grid_point[1]]
    
    #returns state of cell in world coordinates
    def check_world_points(self, world_point):
        p = self.coord_to_grid(world_point)
        return self.check_cell(p)

    def euclidean_distance(self,x1,y1,x2,y2):
        return np.sqrt(np.square(x2-x1)+np.square(y2-y1))

    #returns all obstacles within a given radius
    def obstacle_in_range_list(self, px,py,radius):
        obs = list(self.obstacles)
        obs_in_range = []
        for ob in obs:
            if self.euclidean_distance(px,py,ob[0],ob[1]) <= radius/self.pixel_res:
                obs_in_range.append(ob)
        return obs_in_range

    #helper function that decrements the fire counter to extinguishing fires
    def extinguish_fire_counter(self, obstacles):
        for ob in obstacles:
            curr_val = self.putting_out_fires.get(ob)
            if curr_val == None:
                self.putting_out_fires.update({ob : 5})
            elif curr_val <= 1:
                self.extinguish_fire(ob)
                self.putting_out_fires.pop(ob)
            else :
                self.putting_out_fires.update({ob : curr_val-1})

    #simulator function call that handles fire extinguishing over time. 
    def extinguish_fires(self,px,py,radius):
        obs = self.obstacle_in_range_list(px,py,radius)
        self.extinguish_fire_counter(obs)

    #returns a list of buring obstacles
    def return_burning_obstacles(self):
        obs_keys = self.obstacles.keys()
        burning_obs = []
        for k in obs_keys:
            if self.obstacles.get(k) == 2:
                burning_obs.append(k)
        return burning_obs
        
    #returns a list of non_burning obstacles
    def return_non_burning_obstacles(self):
        obs_keys = self.obstacles.keys()
        nonburning_obs = []
        for k in obs_keys:
            if self.obstacles.get(k) <= 1:
                nonburning_obs.append(k)
        return nonburning_obs
    
    #returns a list of all obstacles
    def return_obstacles(self):
        return self.obstacles.keys()


