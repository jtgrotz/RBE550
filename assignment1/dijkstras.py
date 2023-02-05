import numpy as np
#using heapq as a priority queue
import heapq
#todo fix weird corner issue with pathing
class dijkstras:

    searched = []
    path = []
    iterations = 0



    def __init__(self, field, color):
        self.field = field
        self.color = color
        self.path_color = 1-color

    def search(self, start, end):
        #init cost map of infinity an invalid distance
        costmap = np.ones(np.shape(self.field))*np.inf
        #init previous point map
        height = np.shape(self.field)[0]
        width = np.shape(self.field)[1]
        prev = np.zeros([height,width,2])
        
        self.searched = []
        self.searched.append(start)

        #starting variables
        goal_found = False
        all_searched = False
        self.iterations = 0
        added_index = 1

        q = []
        heapq.heappush(q,(0,added_index,start))
        #need to populate q with all points and their heap values 

        costmap[start[0]][start[1]] = 0
        while (all_searched == False or goal_found == False):
            try:
                #tries to get a point and if empty, end the iteration
                item = heapq.heappop(q)
                point = item[2]
                #adds point to searched map for visualization
                self.searched.append(point)
                self.iterations += 1
            except IndexError: 
                all_searched = True
                break
            if point == end:
                goal_found = True
                break
            neighbors = self.get_four_neighbors(point)
            for i in neighbors:
                if self.check_point(i):
                    #for now the edge weights are always the same distance
                    #if point is new, we need to get rid of infinity
                    new_distance = costmap[point[0]][point[1]] + self.manhattan_distance(point,i)
                    old_distance = costmap[i[0]][i[1]]
                    if new_distance < old_distance:
                        costmap[i[0]][i[1]] = new_distance
                        prev[i[0]][i[1]] = point
                        added_index += 1
                        heapq.heappush(q,(new_distance,added_index,i))
        if costmap[end[0]][end[1]] == np.inf:
            return False
        else:
            self.path = self.get_path(start,end,prev)
            return True
            

    def get_path(self, start, end, previous_map):
        path = []
        #start the path at the end point
        w = list.copy(end)
        next_point = end
        while next_point != start:
            #backwards search until you find the start
            path.append(list.copy(next_point))
            w[0] = int(previous_map[next_point[0]][next_point[1]][0])
            w[1] = int(previous_map[next_point[0]][next_point[1]][1])
            next_point = list.copy(w)
        self.path = path
        return path

        
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
        for item in self.path:
            blank_map[item[0]][item[1]] = self.path_color
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

    def manhattan_distance(self, point1, point2):
        delta_x = abs(point2[1]-point1[1])
        delta_y = abs(point2[0]-point1[0])
        return delta_x + delta_y

    def euclidean_distance(self, point1, point2):
        delta_x_sq = np.square(point2[1]-point1[1])
        delta_y_sq = np.square(point2[0]-point2[0])
        return np.sqrt(delta_x_sq+delta_y_sq)

    def get_path_length(self):
        return len(self.path)
        

