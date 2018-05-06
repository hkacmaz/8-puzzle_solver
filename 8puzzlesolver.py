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


def a_star_search(init_state,heuristic="manhattan"):

    explored_set, heap, heap_entry, counter = set(), list(), {}, itertools.count()

    FRONTIER_SIZE_LIMIT = 0
    SEARCH_DEPTH_LIMIT = 0

    #if(heuristic is "misplaced tiles"):
    key = displaced_tiles_h(init_state)
    #else:
    #    pass

    # set up our starting positions
    root_node = Node(init_state, None, None, 0, 0, key)
    first_entry = (key, 0, root_node)
    heappush(heap, first_entry)
    heap_entry[root_node.map] = first_entry

    while heap:
        node = heappop(heap)
        explored_set.add(node[2].map)
        if node[2].state == GOAL_STATE:
            return heap,node[2]

        neighbors = generate_neighbours(node[2])

        for neighbor in neighbors:
            heuristic_value = 0
            if(heuristic is "manhattan"):
                heuristic_value = manhattan_distance_h(neighbor.state)
            else:
                heuristic_value = displaced_tiles_h(neighbor.state)
            neighbor.id = neighbor.cost + heuristic_value
            next_entry = (neighbor.id, neighbor.move, neighbor)
            if neighbor.map not in explored_set:
                heappush(heap, next_entry)
                explored_set.add(neighbor.map)
                heap_entry[neighbor.map] = next_entry
                if neighbor.depth > SEARCH_DEPTH_LIMIT:
                    SEARCH_DEPTH_LIMIT += 1
            elif neighbor.map in heap_entry and neighbor.id < heap_entry[neighbor.map][2].id:
                hindex = heap.index((heap_entry[neighbor.map][2].id,
                                     heap_entry[neighbor.map][2].move,
                                     heap_entry[neighbor.map][2]))

                heap[int(hindex)] = next_entry
                heap_entry[neighbor.map] = next_entry
                heapify(heap)

        if len(heap) > FRONTIER_SIZE_LIMIT:
            FRONTIER_SIZE_LIMIT = len(heap)


def iterative_deepening_search(init_state):
    FRONTIER_SIZE_LIMIT = 0
    SEARCH_DEPTH_LIMIT = 0
    costs = set()
    threshold = manhattan_distance_h(init_state)
    while True: # hmm, any better way to do this?
        response = depth_limited_dfs(init_state, threshold)
        if type(response) is tuple: # yuck!!
            return response
            break
        threshold = response
        costs = set()


def depth_limited_dfs(init_state, threshold):
    explored_set, stack = set(), list([Node(init_state, None, None, 0, 0, threshold)])
    FRONTIER_SIZE_LIMIT = 0
    SEARCH_DEPTH_LIMIT = 0
    X=0
    costs = set()
    while stack:
        node = stack.pop()
        explored_set.add(node.map)
        if node.state == GOAL_STATE:
            return stack,node
        if node.id > threshold:
            costs.add(node.id)
        if node.depth < threshold:
            neighbors = reversed(generate_neighbours(node))
            for neighbor in neighbors:
                if neighbor.map not in explored_set:
                    neighbor.id = neighbor.cost + manhattan_distance_h(neighbor.state)
                    stack.append(neighbor)
                    explored_set.add(neighbor.map)
                    if neighbor.depth > SEARCH_DEPTH_LIMIT:
                        SEARCH_DEPTH_LIMIT += 1
            if len(stack) > FRONTIER_SIZE_LIMIT:
                FRONTIER_SIZE_LIMIT = len(stack)
    return min(costs)