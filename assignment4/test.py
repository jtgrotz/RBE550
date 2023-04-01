import numpy as np
import queue
import heapq
import robotics_toolbox_edit.OccGrid as OG
import robotics_toolbox_edit.DstarPlanner as DS
import robotics_toolbox_edit.PRMPlanner as PRM

w = np.zeros((10,10))

ww = OG.BinaryOccupancyGrid(w, cellsize=1) 
#ww = OG.OccupancyGrid(w,[1,1],0,2,)
#ww = OG.OccupancyGrid(workspace=[0,4],cellsize=1,value=0)
ww.grid[0,0] = 5

ww.grid[1,1] = 5
ww.grid[1,2] = 3
ww.grid[2,2] = 5



we = ww.grid
sz = ww.grid.shape
#ww.set([1,3,1,3], 1)
print(ww.grid)
print(ww.w2g([0.79,0.3]))
print(ww.w2g([1,2]))
print(ww.g2w([0,0]))
print(ww.w2g([1.5,1.5]))
#dstar = DS.DstarPlanner(ww)
#dstar.plan([8,8], animate=True)
#x = dstar.query([1,2])
prm = PRM.PRMPlanner(ww)
prm.plan()
path = prm.query(start=(1,2), goal=(8,8))
print(path)

ww.plot(ww)


