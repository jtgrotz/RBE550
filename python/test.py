import numpy as np
import queue
import heapq

l = queue.Queue()
l.put([1,2])
l.put([2,3])
l.put([3,4])

x = l.get()
print(x)

w = [[2,3,1],[3,4,7],[5,6,8]]
print(w[0][2])

x = np.zeros([8,8,2])
x[0][0] = [1,2]
x[1][1] = [3,4]

n = list(x[1][1])
print(x)