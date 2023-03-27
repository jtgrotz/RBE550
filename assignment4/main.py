from obstacle_field import obstacle_field
from matplotlib import pyplot
import numpy as np
from depth_first_search import depth_first_search
from breadth_first_search import breadth_first_search
from random_search import random_search
from dijkstras import dijkstras

#different densities for testing purposes.
densities = [0,0.05,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.75] 
#densities = [0.1,0.15,0.2,0.25,0.3,0.35,0.4]
#densities = [0.60]
field_size = 128
start_point = [0,0]
end_point = [field_size-1,field_size-1]
dfs_iterations = np.zeros(len(densities))
djks_iterations = np.zeros(len(densities))
djks_path_length = np.zeros(len(densities))
bfs_iterations = np.zeros(len(densities))
rs_iterations = np.zeros(len(densities))


#loop through different densities and extract data
for i in range(len(densities)):
    curr_density = densities[i]
    print(curr_density)
    #create obstacle field and sets start and end point.
    my_field = obstacle_field(field_size,field_size)
    my_field.generate_field(curr_density)
    my_field.set_start_point(start_point)
    my_field.set_end_point(end_point)

    #set up each search method, and execute
    djks = dijkstras(my_field.field,0.7)
    dfs = depth_first_search(my_field.field,0.2)
    bfs = breadth_first_search(my_field.field, 0.3)
    rs = random_search(my_field.field,0.6)

    #search each method and get number of iterations only if it successfully finds the end
    if djks.search(start_point,end_point):
        djks_iterations[i] = djks.get_iterations()
        djks_path_length[i] = djks.get_path_length()
        print(djks.get_iterations())
    if dfs.search(start_point,end_point):
        dfs_iterations[i] = dfs.get_iterations()
        print(dfs.get_iterations())
    if bfs.search(start_point,end_point):
        bfs_iterations[i] = bfs.get_iterations()
        print(bfs.get_iterations())
    #random often takes too long, so it reaches the iteration limit
    #in the future, I might make this more elegant
    if rs.search(start_point,end_point):
        rs_iterations[i] = rs.get_iterations()
        print(rs.get_iterations())
    else:
        rs_iterations[i] = rs.get_iterations()

    #shows each searched figure independently, not always needed.
    pyplot.figure(figsize=(10,10))
    pyplot.imshow(my_field.field)
    pyplot.imshow(djks.get_searched_map(), alpha=0.4)
    pyplot.show()
    pyplot.figure(figsize=(10,10))
    pyplot.imshow(my_field.field)
    pyplot.imshow(dfs.get_searched_map(), alpha=0.2)
    pyplot.show()
    pyplot.figure(figsize=(10,10))
    pyplot.imshow(my_field.field)
    pyplot.imshow(bfs.get_searched_map(), alpha=0.2)
    pyplot.show()
    pyplot.figure(figsize=(10,10))
    pyplot.imshow(my_field.field)
    pyplot.imshow(rs.get_searched_map(), alpha=0.2)
    pyplot.show()

print('dfs')
print(dfs_iterations)
print('bfs')
print(bfs_iterations)
print('djks')
print(djks_iterations)
print('rs')
print(rs_iterations)
print('path_length')
print(djks_path_length)

pyplot.figure(figsize=(10,10))
pyplot.plot(densities,dfs_iterations, label = "DFS")
pyplot.plot(densities,bfs_iterations, label = "BFS")
pyplot.plot(densities,djks_iterations, label = "Dijkstras")
pyplot.plot(densities,rs_iterations, label = "Random")
pyplot.xlabel("Densities (0-1)")
pyplot.ylabel("Number of iterations")
pyplot.title("Search iterations vs obstacle density")
pyplot.legend()
pyplot.show()

pyplot.figure(figsize=(10,10))
pyplot.plot(densities,djks_path_length, label = "Dijkstras Path Length")
pyplot.xlabel("Densities (0-1)")
pyplot.ylabel("Number of iterations")
pyplot.title("Dijkstras Path Length vs obstacle density")
pyplot.legend()
pyplot.show()




