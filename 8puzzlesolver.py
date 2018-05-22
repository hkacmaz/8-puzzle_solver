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


    

def generate_neighbours(node):
    X=0
    neighbors = []
    neighbors.append(Node(generate_new_state(node.state, 1), node, 1, node.depth + 1, node.cost + 1, 0))
    neighbors.append(Node(generate_new_state(node.state, 2), node, 2, node.depth + 1, node.cost + 1, 0))
    neighbors.append(Node(generate_new_state(node.state, 3), node, 3, node.depth + 1, node.cost + 1, 0))
    neighbors.append(Node(generate_new_state(node.state, 4), node, 4, node.depth + 1, node.cost + 1, 0))
    nodes = [neighbor for neighbor in neighbors if neighbor.state]
    return nodes


def generate_new_state(state, position):
    #X=x
    candidate_state = state[:]
    index = candidate_state.index(0)
    if position == 1:  # North
        if index not in range(0, BOARD_EDGE):
            #X=X+1
            temp = candidate_state[index - BOARD_EDGE]
            candidate_state[index - BOARD_EDGE] = candidate_state[index]
            candidate_state[index] = temp
            #if X<3:
            #    print(candidate_state)
            return candidate_state
        else:
            return None
    if position == 2:  # South
        if index not in range(BOARD_LENGTH - BOARD_EDGE, BOARD_LENGTH):
            #X=X+1
            temp = candidate_state[index + BOARD_EDGE]
            candidate_state[index + BOARD_EDGE] = candidate_state[index]
            candidate_state[index] = temp
            # if X<3:
            #   print(candidate_state)
            return candidate_state
        else:
            return None
    if position == 3:  # West
        if index not in range(0, BOARD_LENGTH, BOARD_EDGE):
            #X=X+1
            temp = candidate_state[index - 1]
            candidate_state[index - 1] = candidate_state[index]
            candidate_state[index] = temp
            #if X<3:
            #    print(candidate_state)
            return candidate_state
        else:
            return None
    if position == 4:  # East
        if index not in range(BOARD_EDGE - 1, BOARD_LENGTH, BOARD_EDGE):
            #X=X+1
            temp = candidate_state[index + 1]
            candidate_state[index + 1] = candidate_state[index]
            candidate_state[index] = temp
            #if X<3:
            #    print(candidate_state)
            return candidate_state
        else:
            return None
def displaced_tiles_h(state):

    ## effectively a Hamming distance
    displaced = 0
    for goal,cur in zip(GOAL_STATE,state):
        if goal != cur:
            displaced += 1
#    print("MINE:",displaced)
    return displaced

def manhattan_distance_h(state):
    return sum(abs(b % BOARD_EDGE - g % BOARD_EDGE) + abs(b//BOARD_EDGE - g//BOARD_EDGE)
               for b, g in ((state.index(i), GOAL_STATE.index(i)) for i in range(1, BOARD_LENGTH)))

# dumb thing -- just follow the nodes back from the goal to the start state
def extract_path(path,goal):
    current_node = goal
    moves = []

    while START_STATE != current_node.state:
        if current_node.move == 1:
            movement = 'north'
        elif current_node.move == 2:
            movement = 'south'
        elif current_node.move == 3:
            movement = 'west'
        else:
            movement = 'east'
        moves.insert(0, movement)
        current_node = current_node.parent

        # we've come to the end if we've found a node with no parent
        if(current_node is None):
            break


    return moves

def do_algorithm_eval(puzzles,algorithm):
    print("START STATE","\t\t\t","NEXT STATE","\t\t","NEXT STATE+1","\t\t","PATH LENGTH","\t\t","NODE VISITS","\t\t","CPU TIME""\t\t")
    for puzzle in puzzles:
        path = None
        START_STATE = puzzle
        start = timeit.default_timer()
        if(algorithm is "bfs"):
            path,goal = uninformed_search(puzzle,type="breadth")
        if(algorithm is "dfs"):
            path,goal = uninformed_search(puzzle,type="depth")
        if(algorithm is "ids"):
            path,goal = iterative_deepening_search(puzzle)
        if(algorithm is "asman"):
            path,goal = a_star_search(puzzle,heuristic="manhattan")
        if(algorithm is "asmis"):
            path,goal = a_star_search(puzzle,heuristic="mispaced tiles")
        stop = timeit.default_timer()

        print("-"*16)
        print(puzzle,"\t","\t\t\t","\t\t\t",len(extract_path(path,goal)),"\t\t\t",len(path),"\t\t\t",format(stop-start, '.2f')+"s")
        print("-"*16)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("puzzles")

    args = parser.parse_args()
    puzzles = []
    with open(args.puzzles) as f:
        for l in f:
            p = l.strip().strip(" ").split(",")
            p = [int(x) for x in p]
            puzzles.append(p)

    # begin the code to manage the experiments
    breadth_first_search_puzzles = puzzles[:2]
    depth_first_search_puzzles = puzzles[:2]

    # iterative deepening - any 10 from the dataset
    random.shuffle(puzzles)
    iterative_deepening_search_puzzles = puzzles[:2]

    # a star puzzles
    random.shuffle(puzzles)
    a_star_puzzles = puzzles[:2]

    print("-"*16)
    print("Algorithm: Breadth-First Search")
    print("-"*16)
    do_algorithm_eval(breadth_first_search_puzzles,algorithm="bfs")

    print("\n")
    print("-"*16)
    print("Algorithm: Depth-First Search")
    print("-"*16)
    do_algorithm_eval(depth_first_search_puzzles,algorithm="dfs")

    print("\n")
    print("-"*16)
    print("Algorithm: A* Search with Misplaced Tile Heuristic")
    print("-"*16)
    do_algorithm_eval(a_star_puzzles,algorithm="asmis")

    print("\n")
    print("-"*16)
    print("Algorithm: A* Search with Manhattan Distance Heuristic")
    print("-"*16)
    do_algorithm_eval(a_star_puzzles,algorithm="asman")

    print("\n")
    print("-"*16)
    print("Algorithm: Iterative Deepening Search")
    print("-"*16)
    do_algorithm_eval(iterative_deepening_search_puzzles,algorithm="ids")
