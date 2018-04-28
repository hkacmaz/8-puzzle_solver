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