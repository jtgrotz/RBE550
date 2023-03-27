import a_star
#Wumpus Moby Dick

class wumpus:

    current_location = [0,0]
    goal_location = [0,0]
    completed_path = True
    path = []
    path_index = 0

    def __init__(self, start_point,map):
        self.current_location = start_point
        self.planner = a_star(map, 0.7)

    def find_next_goal():
        return 0

    def generate_path():
        return 0

    def get_next_path_point():
        return 0



