import numpy as np
import OccGrid as OG
import roboticstoolbox as rtb

#new obstacle field is a occupancy grid and a regular array.
class obstacle_field:

    shape1 = [[0,0],[0,1],[0,2],[0,3]]
    shape2 = [[0,0],[1,0],[0,1],[0,2]]
    shape3 = [[0,0],[0,1],[0,2],[1,1]]
    shape4 = [[0,0],[0,1],[1,1],[1,2]]


    shapes = [shape1,shape2,shape3,shape4]

    ocgrid = []

    def __init__(self, height, width, pixel_res):
        self.height = height
        self.width = width
        self.shape_size = 4
        self.pixel_res = pixel_res
        #might add integer marking here if it gets weird
        self.field = np.zeros((height,width))

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
        self.ocgrid = OG.OccupancyGrid(self.field, cellsize=5)
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
            self.field[x_location][y_location] = 1
        
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
    def extinguish_fire(self, world_point):
        p = self.coord_to_grid(world_point)
        self.ocgrid.grid[p[0],p[1]] = -1

    #10 or greater is onfire
    def start_fire(self,grid_point):
        self.ocgrid.grid[grid_point[0],grid_point[1]] = 10

    def increment_fire(self):
        sz = self.ocgrid.grid.shape
        for y in range(sz[0]):
            for x in range(sz[1]):
                if self.ocgrid.grid[y,x]
                #todo function for checking occupancy, function for setting occupancy in OCCgrid.


    #increments fire timer and spreads fires if needed.
    def spread_fire(self,radius):
        return 0

    def check_state(self, grid_point):
        return self.ocgrid.grid[grid_point[0],grid_point[1]]

