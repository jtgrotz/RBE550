import numpy as np
import queue
import heapq
import OccGrid as OG

w = np.zeros((5,5))

ww = OG.OccupancyGrid(w, cellsize=1) 
ww.grid[0,0] = 5

ww.grid[1,1] = 5
ww.grid[1,2] = 5
ww.grid[2,2] = 5



we = ww.grid
sz = ww.grid.shape
#ww.set([1,3,1,3], 1)
print(ww.grid)
print(ww.w2g([1,3]))
print(ww.w2g([1,2]))
print(ww.g2w([1,2]))
print(ww.w2g([1.5,1.5]))
OG.OccupancyGrid.plot(ww, block=True)
