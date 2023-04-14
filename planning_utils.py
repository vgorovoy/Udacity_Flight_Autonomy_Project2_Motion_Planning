from enum import Enum
from queue import PriorityQueue
import numpy as np
from math import sqrt


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    DIAGONAL_COST = sqrt(2)

    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    UPPERLEFT = (-1, -1, DIAGONAL_COST)
    UPPERRIGHT = (-1, 1, DIAGONAL_COST)
    LOWERLEFT = (1, -1, DIAGONAL_COST)
    LOWERRIGHT = (1, 1, DIAGONAL_COST)

    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
        elif self == self.UPPERRIGHT:
            return '⬈'
        elif self == self.UPPERLEFT:
            return '⬉'
        elif self == self.LOWERRIGHT:
            return '⬊'
        elif self == self.LOWERLEFT:
            return '⬋'

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = {Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN, Action.UPPERLEFT,
             Action.UPPERRIGHT, Action.LOWERLEFT, Action.LOWERRIGHT}
    #     valid = {Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN}

    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0:
        valid.remove(Action.UP)
        valid.discard(Action.UPPERLEFT)
        valid.discard(Action.UPPERRIGHT)
    if x - 1 >= 0 and grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n:
        valid.discard(Action.DOWN)
        valid.discard(Action.LOWERLEFT)
        valid.discard(Action.LOWERRIGHT)
    if x + 1 <= n and grid[x + 1, y] == 1:
        valid.discard(Action.DOWN)
    if y - 1 < 0:
        valid.remove(Action.LEFT)
        valid.discard(Action.UPPERLEFT)
        valid.discard(Action.LOWERLEFT)
    if y - 1 >= 0 and grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m:
        valid.remove(Action.RIGHT)
        valid.discard(Action.UPPERRIGHT)
        valid.discard(Action.LOWERRIGHT)
    if y + 1 <= m and grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)
    if x - 1 >= 0 and y - 1 >= 0 and grid[x - 1, y - 1] == 1:
        valid.discard(Action.UPPERLEFT)
    if x - 1 >= 0 and y + 1 <= m and grid[x - 1, y + 1] == 1:
        valid.discard(Action.UPPERRIGHT)
    if x + 1 <= n and y - 1 >= 0 and grid[x + 1, y - 1] == 1:
        valid.discard(Action.LOWERLEFT)
    if x + 1 <= n and y + 1 <= m and grid[x + 1, y + 1] == 1:
        valid.discard(Action.LOWERRIGHT)

    return valid


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def point(p):
    return np.array([p[0], p[1], 1.]).reshape(1, -1)

def collinearity_check(p1, p2, p3, epsilon=1e-6):
    m = np.concatenate((p1, p2, p3), 0)
    det = np.linalg.det(m)
    return abs(det) < epsilon

def prune_path(path):
    if path is not None:
        pruned_path = [p for p in path]
        points2remove = set()
        #         i_start = 0
        i_end = 2
        if len(pruned_path) > 2:
            while i_end < len(pruned_path):
                if collinearity_check(point(pruned_path[i_end - 2]), point(pruned_path[i_end - 1]),
                                      point(pruned_path[i_end])):
                    points2remove.add(pruned_path[i_end - 1])
                i_end += 1
        pruned_path = [p for p in pruned_path if p not in points2remove]
    else:
        pruned_path = path
    return pruned_path