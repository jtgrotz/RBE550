import numpy as np

class random_search:

    searched = []
    

    def __init__(self, field, color):
        self.field = field
        self.color = color

    def search(self, start, end):
        self.iterations = 0
        self.searched = []
        self.searched.append(start)
        goal_found = False
        stack = []
        stack.append(start)


        while (goal_found == False):
            self.iterations += 1
            #iteration check to cancel out after a certain time
            if self.iterations >= len(self.field)*len(self.field)*3:
                return False
            current_point = stack.pop()
            self.searched.append(current_point)
            #start neighbor as invalid space.
            neighbor = [-1,-1]
            if current_point == end:
                goal_found = True
            #finds neighbor until it is a valid point in the field.
            while self.check_point(neighbor) == False:
                neighbor = self.get_random_neighbors(current_point)
            stack.append(neighbor)
        return True

        
    def get_random_neighbors(self, point):
        #generate random number to determine what direction to go.
        j = np.random.rand(1)
        if j >= 0 and j < 0.25:
            y = ([point[0]+1,point[1]])
        elif j >= 0.25 and j < 0.5:
            y = ([point[0],point[1]+1])
        elif j >= 0.5 and j < 0.75:
            y = ([point[0]-1,point[1]])
        else :
            y = ([point[0],point[1]-1])
        return y


    def get_eight_neighbors(self,point):
        y = self.get_four_neighbors(point)
        #adds the four corners to the list
        y.append([point[0]+1,point[1]+1])
        y.append([point[0]-1,point[1]-1])
        y.append([point[0]-1,point[1]+1])
        y.append([point[0]+1,point[1]-1])
        return y

    def get_searched_map(self):
        blank_map = np.zeros(np.shape(self.field))
        for item in self.searched:
            blank_map[item[0]][item[1]] = self.color
        return blank_map

    def check_point(self, point):
        #out of bounds
        width = np.shape(self.field)[1]
        height = np.shape(self.field)[0]
        if (point[0] < 0 or point[0] >= height):
            return False
        if (point[1] < 0 or point[1] >= width):
            return False
        #obstacle
        if self.field[point[0]][point[1]] == 1:
            return False
        return True

    def get_iterations(self):
        return self.iterations

        

