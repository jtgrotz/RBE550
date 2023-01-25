from obstacle_field import obstacle_field
from matplotlib import pyplot

my_field = obstacle_field(128,128)
my_field.generate_field(0.2)
pyplot.figure(figsize=(10,10))
pyplot.imshow(my_field.field)
print(my_field.field)
pyplot.show()