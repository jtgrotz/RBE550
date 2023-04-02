import numpy as np
import queue
import heapq
import robotics_toolbox_edit.OccGrid as OG
import robotics_toolbox_edit.DstarPlanner as DS
import robotics_toolbox_edit.PRMPlanner as PRM
from matplotlib import pyplot

w = np.zeros((10,10))

ww = OG.BinaryOccupancyGrid(w, cellsize=2) 
www = OG.OccupancyGrid(w,cellsize=2)
#ww = OG.OccupancyGrid(w,[1,1],0,2,)
#ww = OG.OccupancyGrid(workspace=[0,4],cellsize=1,value=0)
www.grid[0,0] = 5

www.grid[1,1] = 5
www.grid[1,2] = 3
www.grid[2,2] = 5



we = ww.grid
sz = ww.grid.shape
#ww.set([1,3,1,3], 1)
print(www.grid)
print(ww.w2g([0.79,0.3]))
print(ww.w2g([1,2]))
print(ww.g2w([0,0]))
print(ww.w2g([1.5,1.5]))
#dstar = DS.DstarPlanner(ww)
#dstar.plan([8,8], animate=True)
#x = dstar.query([1,2])
#prm = PRM.PRMPlanner(ww)
#prm.plan()
#path = prm.query(start=(1,2), goal=(8,8))
#print(path)
#obs = {}
##obs.update({(1,2) : 1})
#obs.update({(1,4) : "red"})
#obs.update({(1,2) : 3})
#print(obs)
#print(obs.get((1,4)))

##www.plot(block=False)
#pyplot.grid(visible=False)
#pyplot.plot(path[0],path[1])
#pyplot.show()

q = []
heapq.heappush(q,(3,(1,2)))
heapq.heappush(q,(4,(1,4)))
heapq.heappush(q,(5,(1,5)))
heapq.heappush(q,(7,(1,7)))
heapq.heappush(q,(6,(1,6)))

maxes = heapq.nlargest(3,q)
print(maxes[0][1])

points = np.array([[1,2],[2,3],[3,4],[4,5]])
points = np.flip(points,1)
print(points)
pyplot.plot(*points.T)
pyplot.show(block=True)


