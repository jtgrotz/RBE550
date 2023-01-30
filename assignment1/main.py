from obstacle_field import obstacle_field
from matplotlib import pyplot
from depth_first_search import depth_first_search
from breadth_first_search import breadth_first_search
from random_search import random_search
from dijkstras import dijkstras


my_field = obstacle_field(12,12)
my_field.generate_field(0.3)
pyplot.figure(figsize=(10,10))
start_point = [0,0]
end_point = [9,9]

my_field.set_start_point(start_point)
my_field.set_end_point(end_point)
pyplot.imshow(my_field.field)

dfs = dijkstras(my_field.field,0.7)
dfs.search(start_point,end_point)
pyplot.imshow(dfs.get_searched_map(), alpha=0.6)
print(dfs.get_iterations())

pyplot.show()


