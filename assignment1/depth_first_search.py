import numpy as np

class depth_first_search:

    searched = []
    path = []
    iterations = 0



    def __init__(self, field, color):
        self.field = field
        self.color = color

    def search(self, start, end):
        self.iterations = 0
        self.searched = []
        self.searched.append(start)
        self.path = []
        self.path.append(start)
        goal_found = False
        stack = []
        stack.append(start)


        while (goal_found == False):
            if len(stack) == 0:
                return False
            self.iterations += 1
            current_point = stack.pop()
            if current_point == end:
                goal_found = True
            neighbors = self.get_four_neighbors(current_point)
            for i in neighbors:
                if i not in self.searched:
                    #check if valid point
                    if self.check_point(i):
                        stack.append(i)
                        self.searched.append(i)
        return True

        
    def get_four_neighbors(self, point):
        y = []
        #adds the four cardinal directions to the list
        y.append([point[0]+1,point[1]])
        y.append([point[0],point[1]+1])
        y.append([point[0]-1,point[1]])
        y.append([point[0],point[1]-1])
        
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

        

