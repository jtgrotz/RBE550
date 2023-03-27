import numpy as np

class a_star:
    path = []
    searched = []
    
    def __init__(self,field,color):
        self.field = field
        self.color = color