# Basic searching algorithms

import numpy as np
 
# Class for each node in the grid
class Node:
    def __init__(self, grid, start, goal,path,h_grid):
        self.grid = grid                #grid
        self.pos = start                # position initally start
        self.pos_str = str(start)       # position converted to string
        self.pos_depth = 0              # depth (steps it have needed to reach)
        self.goal_str = str(goal)       # goal conv to string
        self.explored = {}              # explored dictonary
        self.not_explored = {}          # not_explored dictonary
        self.not_explored[str(start)] = 0 # Assigns value 0 to the key which is string in form of start
        self.path = path
        self.h_grid = h_grid
    
    
    
    def gen_moves(self, pos):                         #It generates possible moves from the current position
        ##print(pos)
        r = np.array([0,+1])
        d = np.array([+1,0])
        l = np.array([0,-1])
        u = np.array([-1,0])
        
        potential_moves = [pos + r, pos + d, pos + l, pos +u]
        #potential_moves += [pos+u+r, pos+u+l, pos+d+r, pos+d+l]
        
        return potential_moves
    
    def valid_move(self,move):                        #It Check if the move is Valid or not by boundry conditions and Obstacles.
        # Check if out of boundry
        
        if (move[0] < 0) or (move[0] > self.grid.shape[0]-1):
            return False
        if (move[1] < 0) or (move[1] > self.grid.shape[1]-1):
            return False
        
        #Check if wall or obstacles exist
        if self.grid[move[0], move[1]] ==1:
            return False
        
        return True
    
    def str_to_array(self,string):                    #Just convert string to array.
        array = [int(string[1]), int(string[3])]
        return np.array(array) 
    
    def get_possible_moves(self):
        potential_moves = self.gen_moves(self.pos)                    #gets possible moves here 8 coz we can move diagonally but changes with each nodes
        for move in potential_moves:                                  # we iterate through the moves
            # Check if potential move is valid.
            if not self.valid_move(move):                             #we check if moves is valid by boundry conditions and obstacles
                continue                                              #if move is not valid we will not move next line and skip that move
            # Check if move has already been explored.
            if (str(move) not in self.explored) and (str(move) not in self.not_explored):
                self.not_explored[str(move)] = self.pos_depth + 1      # here we make not explored dictonary by adding move as key in form of string and assigning value as 1
            
        # Since all next possible moves have been determined,
        # consider current location explored.
        self.explored[self.pos_str] = self.pos_depth                   #we mark current location in explored

        return True

    def goal_found(self):
        if self.goal_str in self.not_explored:                         #it will check if goal is there in dictonary not_explored, if it is there it will execute following code if not then will not execute and will return False
            # Add goal to path.
            self.pos = self.str_to_array(self.goal_str)                #it will add goal and convert string to array into pos
            self.pos_depth = self.not_explored.pop(self.goal_str)      # this will give corresponding value of move(key) i.e '[3,1]':5 so will return 5 and will also remove that key and values from the not_explored list
            self.path[self.pos[0], self.pos[1]] =  self.pos_depth      # this will store path as form of 10X10 matrix with the value to reach there
            return True                                                # will return True if goal is found and executed all above lines
        return False                                                   # will return False if goal is not found in if statement

   
    def explore_next_move(self):
        # Determine next move to explore.
        sorted_not_explored = sorted(                                 # sorted_not_explored will create new list based on not_explored dictonary values corresponding to keys(moves) in ascending order.
            self.not_explored,
            key=self.not_explored.get,
            reverse=False)

        # Determine the pos and depth of next move.
        key = sorted_not_explored[0]                                 # key will take 1st value from sorted_not_explored list like [3,4]
        self.pos_str = key                                           # it will convt key variable values into string 
        self.pos = self.str_to_array(key)                            # it will convert key into array as position
        self.pos_depth = self.not_explored[key]                      # it will store values from the dictonary i.e '[3,1]':5 so will return 5

        self.not_explored.pop(key)                                   # remove key and value from the dictonary
        
        # Write depth of next move onto path.
        self.path[self.pos[0], self.pos[1]] = self.pos_depth         # it will store path in 10x10 matrix value of depth to corresponding index

        return True   
    
    def exp_next_moves_for_dfs(self):
    
        # Determine next move to explore.
        sorted_not_explored = sorted(                                 # sorted_not_explored will create new list based on not_explored dictonary values corresponding to keys(moves) in ascending order.
            self.not_explored,
            key=self.not_explored.get,
            reverse=True)

        # Determine the pos and depth of next move.
        key = sorted_not_explored[0]                                 # key will take 1st value from sorted_not_explored like [3,4]
        self.pos_str = key                                           # it will convt key variable values into string 
        self.pos = self.str_to_array(key)                            # it will convert key into array as position
        self.pos_depth = self.not_explored[key]                      # it will store values from the dictonary i.e '[3,1]':5 so will return 5

        self.not_explored.pop(key)                                   # remove key and value from the dictonary
        
        # Write depth of next move onto path.
        self.path[self.pos[0], self.pos[1]] = self.pos_depth         # it will store path in 10x10 matrix value of depth to corresponding index

        return True  
    
    def get_possible_moves_astar(self):
        potential_moves = self.gen_moves(self.pos)
        for move in potential_moves:
            # Check if potential move is valid.
            if not self.valid_move(move):
                continue
            # Check if move has already been explored.
            if (str(move) not in self.explored) and (str(move) not in self.not_explored):
                self.not_explored[str(move)] = (self.pos_depth + 1) + self.heuristic(move)     #It just calculate heuristic value from the heuristic Function.
                # Visualize the Heuristic Grid
                self.h_grid[move[0], move[1]] = self.not_explored[str(move)]
        # Since all next possible moves have been determined,
        # consider current location explored.
        self.explored[self.pos_str] = self.pos_depth
        return True
    
    def goal_found_astar(self):                                                               #It is same as above goal found but here it also has Heuristic cost of the Goal.
        if self.goal_str in self.not_explored:
            # Add goal to path.
            self.pos = self.str_to_array(self.goal_str)
            heurestic_cost = self.not_explored.pop(self.goal_str)
            self.path[self.pos[0], self.pos[1]] = heurestic_cost
            return True
        return False
    
    def explore_next_move_astar(self):                                                        # It is same as explore next_move from above just pos_depth is calculated different. Pos_depth is cost to come and equals to Total cost - Heuristic cost.
        # Determine next move to explore.
        sorted_not_explored = sorted(
            self.not_explored,
            key=self.not_explored.get,
            reverse=False)
 
        # Determine the pos and depth of next move.
        self.pos_str = sorted_not_explored[0]
        self.pos = self.str_to_array(self.pos_str)
        self.pos_depth = round(self.not_explored.pop(self.pos_str) - self.heuristic(self.pos))

        
        # Write depth of next move onto path.
        x = self.pos[0]
        y = self.pos[1]
        self.path[x, y] = round(self.pos_depth, 1)

        return True

    def heuristic(self, move):                                                                #It calculates Manhattan Distance
        diff = move - self.str_to_array(self.goal_str)
        answer = np.sum(np.abs(diff))

        return round(answer, 2)

        
def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
    and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
        e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    
    # Using Numpy array insted of List
    
    
    goal = np.array(goal)
    grid = np.array(grid)
    start = np.array(start)
    steps = 0
    found = False
    
    # Use to Visualize the path and best path
    
    path = np.zeros([len(grid), len(grid)], dtype=int)
    best_path = np.zeros([len(grid), len(grid)], dtype=int)
    node= Node(grid, start, goal, path,0)
    
    while True:
        steps += 1
        # Determine next possible moves.
        node.get_possible_moves()    #this will generate moves,chk validity for all moves and stores in not_explored and current in explored
        if node.goal_found():        #if this return True and goal is found it will break while loop and gets out of it. If it is false it will move to next line which is explore next move. 
            break
        node.explore_next_move()      #it will 1st sort the moves based on depth values and will store the pos value and key and will kick out that move from not_explored. Will return as True and loop continues


    #print('Explored Path')
    #print(path)
    #print('Fully explored count ' + str(np.count_nonzero(path)))
    stepss = str(np.count_nonzero(path))

    def find_best_path(pos):            #It will give best path from goal to the start, Here it will take goal as the position.
        ##print("this is second pos", pos)
        best_path[pos[0], pos[1]] = 1                
        h_pos = path[pos[0], pos[1]]
        if h_pos == 1:
            return 1

        potential_moves = node.gen_moves(pos)
        for move in potential_moves:
            if not node.valid_move(move):
                continue
            h_move = path[move[0], move[1]]
            if h_move == (h_pos - 1):
                return find_best_path(move) + 1


    goal_count = find_best_path(goal) 
    best_path[start[0], start[1]] = 99
    #print('Best Path To Goal')
    #print('-----------------')
    #print(best_path)
    #print('Moves to Goal: ' + str(goal_count))
    #print('')
    found = True
    steps += 1
    # steps = stepss
    path_with_row_col = np.nonzero(best_path)
    ro = path_with_row_col[0]
    col = path_with_row_col[1]
    r = np.array([ro])
    c = np.array([col])
    path = np.concatenate((r.T,c.T), axis=1)
    #print("this is path------------", path)
    path = path.tolist()
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    return path, steps


def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
    and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
        e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    goal = np.array(goal)
    grid = np.array(grid)
    start = np.array(start)
    steps = 0
    found = False
    path = np.zeros([len(grid), len(grid)], dtype=int)
    best_path = np.zeros([len(grid), len(grid)], dtype=int)
    node= Node(grid, start, goal, path,0)
    
    while True:
        steps += 1
        # Determine next possible moves.
        node.get_possible_moves()    #this will generate moves,chk validity for all moves and stores in not_explored and current in explored
        if node.goal_found():        #if this return True and goal is found it will break loop and gets out of it.  
            break
        node.exp_next_moves_for_dfs()      #it will 1st sort the moves based on depth values and will store the pos value and key and will kick out that move from not_explored. Will return as True and loop continues


    #print('Explored Path')
    #print(path)
    #print('Fully explored count ' + str(np.count_nonzero(path)))
    stepss = str(np.count_nonzero(path))

    def find_best_path(pos):
        ##print("this is second pos", pos)
        best_path[pos[0], pos[1]] = 1
        h_pos = path[pos[0], pos[1]]
        if h_pos == 1:
            return 1

        potential_moves = node.gen_moves(pos)
        for move in potential_moves:
            if not node.valid_move(move):
                continue
            h_move = path[move[0], move[1]]
            if h_move == (h_pos - 1):
                return find_best_path(move) + 1

    # print(node.explored)
    goal_count = find_best_path(goal) 
    best_path[start[0], start[1]] = 99
    # print('Best Path To Goal')
    #print('-----------------')
    # print(best_path)
    #print('Moves to Goal: ' + str(goal_count))
    #print('')
    found = True
    steps += 1
    # steps = stepss
    path_with_row_col = np.nonzero(best_path)
    ro = path_with_row_col[0]
    col = path_with_row_col[1]
    r = np.array([ro])
    c = np.array([col])
    path = np.concatenate((r.T,c.T), axis=1)
    #print("this is path------------", path)
    
    path = []
    for key in node.explored.keys():
        path.append(node.str_to_array(key).tolist())
    path.append(goal.tolist())
    # path = list(node.explored.keys())
    # print(path)
    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")
    return path, steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
    and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
        e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    goal = np.array(goal)
    grid = np.array(grid)
    start = np.array(start)
    steps = 0
    found = False
    path = np.zeros([len(grid), len(grid)], dtype=int)
    best_path = np.zeros([len(grid), len(grid)], dtype=int)
    node= Node(grid, start, goal, path,0)
    
    while True:
        steps+=1
        # Determine next possible moves.
        node.get_possible_moves()    #this will generate moves,chk validity for all moves and stores in not_explored and current in explored
        if node.goal_found():        #if this return True and goal is found it will break while loop and gets out of it. If it is false it will move to next line which is explore next move. 
            break
        node.explore_next_move()      #it will 1st sort the moves based on depth values and will store the pos value and key and will kick out that move from not_explored. Will return as True and loop continues


    #print('Explored Path')
    #print(path)
    #print('Fully explored count ' + str(np.count_nonzero(path)))
    stepss = str(np.count_nonzero(path))

    def find_best_path(pos):
        ##print("this is second pos", pos)
        best_path[pos[0], pos[1]] = 1
        h_pos = path[pos[0], pos[1]]
        if h_pos == 1:
            return 1

        potential_moves = node.gen_moves(pos)
        for move in potential_moves:
            if not node.valid_move(move):
                continue
            h_move = path[move[0], move[1]]
            if h_move == (h_pos - 1):
                return find_best_path(move) + 1


    goal_count = find_best_path(goal) 
    best_path[start[0], start[1]] = 99
    #print('Best Path To Goal')
    #print('-----------------')
    #print(best_path)
    # print('Moves to Goal: ' + str(goal_count))
    #print('')
    found = True
    steps+=1
    # steps = stepss
    path_with_row_col = np.nonzero(best_path)
    ro = path_with_row_col[0]
    col = path_with_row_col[1]
    r = np.array([ro])
    c = np.array([col])
    path = np.concatenate((r.T,c.T), axis=1)
    #print("this is path------------", path)
    
    path = path.tolist()
    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
    and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
        e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    goal = np.array(goal)
    grid = np.array(grid)
    start = np.array(start)
    steps = 0
    found = False
    path = np.zeros([len(grid), len(grid)], dtype=int)
    best_path = np.zeros([len(grid), len(grid)], dtype=int)
    h_grid = np.zeros([len(grid), len(grid)], dtype=float)
    node= Node(grid, start, goal, path,h_grid)
    
    while True:
        # Determine next possible moves.
        node.get_possible_moves_astar()    #this will generate moves,chk validity for all moves and stores in not_explored and current in explored
        if node.goal_found_astar():        #if this return True and goal is found it will break loop and gets out of it.  
            break
        node.explore_next_move_astar()      #it will 1st sort the moves based on depth values and will store the pos value and key and will kick out that move from not_explored. Will return as True and loop continues

    #print('Heuristic Grid')
    #print('--------------')
    #print(h_grid)   
    #print('Explored Path')
    path[start[0], start[1]] = 9999
    #print(path)
    #print('Fully explored count ' + str(np.count_nonzero(path)))
    stepss = str(np.count_nonzero(path))
    
    def find_best_path(pos):
        best_path[pos[0], pos[1]] = 1
        h_pos = path[pos[0], pos[1]]
        if h_pos == 1:
            return 1

        potential_moves = node.gen_moves(pos)
        best_move = [0, 0]
        best_h = h_pos
        for move in potential_moves:
            if not node.valid_move(move):
                continue
            h_move = path[move[0], move[1]]
            if h_move <= best_h and h_move != 0:
                best_h = h_move
                best_move = move
        return find_best_path(best_move) + 1

    goal_count = find_best_path(goal)
    best_path[start[0], start[1]] = 9999
    #print('')
    #print('Best Path To Goal')
    #print('-----------------')
    #print(best_path)
    #print('')
    #print('Moves to Goal: ' + str(goal_count))
    found = True
    steps = stepss
    path_with_row_col = np.nonzero(best_path)
    ro = path_with_row_col[0]
    col = path_with_row_col[1]
    r = np.array([ro])
    c = np.array([col])
    path = np.concatenate((r.T,c.T), axis=1)
    #print("this is path------------", path)
    
    path = path.tolist()
    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps






# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
