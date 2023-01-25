import numpy as np

shape1 = [[1,2],[3,4],[5,6],[7,73]]
x = np.zeros([3,3])
print((12*np.ones([3,4]))-1)
print(np.sum((12*np.ones([3,4]))-1))
number_of_shapes = 3
shape_size = 4
print(np.ceil(shape_size*np.random.rand(number_of_shapes,1)))


shape1 = [[0,0],[0,1],[0,2],[0,3]]
shape2 = [[0,0],[1,0],[0,1],[0,2]]
shape3 = [[0,0],[0,1],[0,2],[1,1]]
shape4 = [[0,0],[0,1],[1,1],[1,2]]
rand_shapes = [1.0,2.0,3.0]

shapes = [shape1,shape2,shape3,shape4]

print(shapes[rand_shapes[1]])
