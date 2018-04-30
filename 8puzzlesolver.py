import argparse
from heapq import heappush, heappop, heapify
import timeit
import resource
from collections import deque
import itertools
import random

# it could modifiable for bigger boards i guess! 
BOARD_LENGTH = 9
BOARD_EDGE = 3
GOAL_STATE = [0, 1, 2, 3, 4, 5, 6, 7, 8] # the state we're aiming for -- could be anything!
START_STATE = [] # used later
X=0

class Node:
    def __init__(self, state, parent, move, depth, cost, id):
        self.state = state
        self.parent = parent
        self.move = move
        self.depth = depth
        self.cost = cost
        self.id = id
        if self.state:
            self.map = ''.join(str(e) for e in self.state)

    def __lt__(self, other):
        return self.map < other.map

def uninformed_search(init_state,type="depth"):

    FRONTIER_SIZE_LIMIT = 0
    SEARCH_DEPTH_LIMIT = 0

    # select between dfs and bfs -- the only difference is stack/queue usage!
    data_structure = None
    if(type is "depth"):
        explored_set, data_structure = set(), list([Node(init_state, None, None, 0, 0, 0)])
    else:
        explored_set, data_structure = set(), deque([Node(init_state, None, None, 0, 0, 0)])


    while data_structure:
        if(type is "depth"):
            node = data_structure.pop()
        else:
            node = data_structure.popleft()

        explored_set.add(node.map)

        # if this is the goal, we're done! quit!
        if node.state == GOAL_STATE:
            return data_structure, node

        neighbors = reversed(generate_neighbours(node))
        for neighbor in neighbors:
            if neighbor.map not in explored_set:
                data_structure.append(neighbor)
                explored_set.add(neighbor.map)

                # if we're depth limited
                if neighbor.depth > SEARCH_DEPTH_LIMIT:
                    SEARCH_DEPTH_LIMIT += 1

        if len(data_structure) > FRONTIER_SIZE_LIMIT:
            FRONTIER_SIZE_LIMIT = len(data_structure)