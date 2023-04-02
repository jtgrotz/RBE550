from obstacle_field import obstacle_field
from matplotlib import pyplot
import numpy as np
from wumpus import wumpus
from firetruck import fire_truck
from simulator import simulator
import time

my_map = obstacle_field(50,50,5)
my_map.generate_field(0.1)
truck = fire_truck([15,15],my_map)
wump = wumpus([45,45],my_map)

sim = simulator(my_map, wump, truck, 1, 150)

while sim.sim_done == False:
    sim.increment_time()
    time.sleep(0.1)

print("Results")
print("Wumpus")
print(wump.init_time)
print(wump.search_times)

print("Truck")
print(truck.init_time)
print(truck.search_times)

