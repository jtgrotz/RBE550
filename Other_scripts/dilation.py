import numpy as np
import matplotlib as mpl
import matplotlib.pyplot



def dilation(obstacles, dilation_distance,resolution):
    obstacles_copy = obstacles.copy()
    dilation_cells = int(np.round(dilation_distance/resolution))
    directions = [[0,1],[0,-1],[1,0],[-1,0]]
    for ob in obstacles:
        for d in directions:
            new_point = ob.copy();
            for i in range(dilation_cells):
                new_point[0] = new_point[0]+d[0]
                new_point[1] = new_point[1]+d[1]
                obstacles_copy.append(new_point)
                #np.append(obstacles_copy,newpoint)
    return obstacles_copy


if __name__=="__main__":
    obs = [[1,2],[2,2],[3,2],[3,4]]
    resolution = 0.05
    m = dilation(obs,0.05,resolution)
    print(m)


