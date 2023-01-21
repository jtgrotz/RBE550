import numpy as np

class obstacle_field:

    shape1 = [[0,0],[0,1],[0,2],[0,3]];
    shape2 = [[0,0],[1,0],[0,1],[0,2]];
    shape3 = [[0,0],[0,1],[0,2],[1,1]];
    shape4 = [[0,0],[0,1],[1,1],[1,2]];

    shapes = [shape1,shape2,shape3,shape4];

    def __init__(self, height, width, density):
        self.height = height
        self.width = width
        #might add integer marking here if it gets weird
        self.field = np.zeros((height,width))

    def generate_field(self, density):

    

    
