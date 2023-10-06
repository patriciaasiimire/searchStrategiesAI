#dfs search
def dfs_graph_search(start_state, goal_state, graph):
    stack = [(start_state, [])]
    explored = set()

    while stack:
        current_state, path = stack.pop()

        if current_state == goal_state:
            return path + [current_state]

        if current_state in explored:
            continue

        explored.add(current_state)

        for neighbor in graph.get(current_state, []):
            neighbor_state, _ = neighbor
            if neighbor_state not in explored:
                stack.append((neighbor_state, path + [current_state]))

    return None

if __name__ == "__main__":
    graph = {
        's': [('a', 3), ('b', 1)],
        'a': [('b', 2), ('c', 2)],
        'b': [('c', 3)],
        'c': [('d', 4), ('g', 4)],
        'd': [('g', 1)],
        'g': []
    }

    start_state = 's'
    goal_state = 'g'
    path = dfs_graph_search(start_state, goal_state, graph)

    if path:
        print("Path found:", path)
    else:
        print("No path found.")


#bfs search
from collections import deque

def bfs_graph_search(start_state, goal_state, graph):
    frontier = deque([(start_state, [])])
    explored = set()

    while frontier:
        current_state, path = frontier.popleft()

        if current_state == goal_state:
            return path + [current_state]

        if current_state in explored:
            continue

        explored.add(current_state)

        for neighbor, _ in graph.get(current_state, []):
            if neighbor not in explored:
                frontier.append((neighbor, path + [current_state]))

    return None


if __name__ == "__main__":
    
    graph = {
        's': [('a', 3), ('b', 1)],
        'a': [('b', 2), ('c', 2)],
        'b': [('c', 3)],
        'c': [('d', 4), ('g', 4)],
        'd': [('g', 1)],
        'g': []
    }

    start_state = 's'
    goal_state = 'g'
    path = bfs_graph_search(start_state, goal_state, graph)

    if path:
        print("Path found:", path)
    else:
        print("No path found.")


#greedy search
import heapq

class Node:
    def __init__(self, state, heuristic):
        self.state = state
        self.heuristic = heuristic

    def __lt__(self, other):
        return self.heuristic < other.heuristic

def greedy_graph_search(start_state, goal_state, graph, heuristic_fn):
    start_node = Node(start_state, heuristic_fn(start_state))
    frontier = []
    heapq.heappush(frontier, start_node)
    explored = set()

    while frontier:
        current_node = heapq.heappop(frontier)
        current_state = current_node.state

        if current_state == goal_state:
            return current_state

        if current_state in explored:
            continue

        explored.add(current_state)

        for neighbor in graph.get(current_state, []):
            neighbor_state, _ = neighbor
            neighbor_node = Node(neighbor_state, heuristic_fn(neighbor_state))
            heapq.heappush(frontier, neighbor_node)

    return None


if __name__ == "__main__":
    
    graph = {
        's': [('a', 3), ('b', 1)],
        'a': [('b', 2), ('c', 2)],
        'b': [('c', 3)],
        'c': [('d', 4), ('g', 4)],
        'd': [('g', 1)],
        'g': []
    }

    start_state = 's'
    goal_state = 'g'

    def heuristic_fn(state):
        heuristic_values = {
            's': 7,
            'a': 5,
            'b': 7,
            'c': 4,
            'd': 1,
            'g': 0
        }
        return heuristic_values[state]

    result = greedy_graph_search(start_state, goal_state, graph, heuristic_fn)

    if result:
        print("Goal state found:", result)
    else:
        print("No path found.")



#A* search
import heapq

class Node:
    def __init__(self, state, g_cost, h_cost, parent=None):
        self.state = state
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parent = parent

    def f_cost(self):
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        return self.f_cost() < other.f_cost()

def astar_graph_search(start_state, goal_state, graph, heuristic_fn):
    start_node = Node(start_state, g_cost=0, h_cost=heuristic_fn(start_state))
    frontier = []
    heapq.heappush(frontier, start_node)
    explored = set()

    while frontier:
        current_node = heapq.heappop(frontier)
        current_state = current_node.state

        if current_state == goal_state:
            return reconstruct_path(current_node)

        if current_state in explored:
            continue

        explored.add(current_state)

        for neighbor, step_cost in graph.get(current_state, []):
            g_cost = current_node.g_cost + step_cost
            h_cost = heuristic_fn(neighbor)
            neighbor_node = Node(neighbor, g_cost, h_cost, current_node)
            heapq.heappush(frontier, neighbor_node)

    return None

def reconstruct_path(node):
    path = []
    while node:
        path.insert(0, node.state)
        node = node.parent
    return path

if __name__ == "__main__":
    
    def heuristic_fn(state):
        heuristic_values = {
            's': 7,
            'a': 5,
            'b': 7,
            'c': 4,
            'd': 1,
            'g': 0,
        }
        return heuristic_values[state]

    
    graph = {
        's': [('a', 3), ('b', 1)],
        'a': [('b', 2), ('c', 2)],
        'b': [('c', 3)],
        'c': [('d', 4), ('g', 4)],
        'd': [('g', 1)],
        'g': []
    }

    start_state = 's'
    goal_state = 'g'
    path = astar_graph_search(start_state, goal_state, graph, heuristic_fn)

    if path:
        print("Path found:", path)
    else:
        print("No path found.")

#ucs search
import heapq

class Node:
    def __init__(self, state, cost, parent=None):
        self.state = state
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def uniform_cost_search(start_state, goal_state, graph):
    start_node = Node(start_state, cost=0)
    frontier = []
    heapq.heappush(frontier, start_node)
    explored = set()

    while frontier:
        current_node = heapq.heappop(frontier)
        current_state = current_node.state

        if current_state == goal_state:
            return reconstruct_path(current_node)

        if current_state in explored:
            continue

        explored.add(current_state)

        for neighbor, cost in graph.get(current_state, []):
            neighbor_cost = current_node.cost + cost
            neighbor_node = Node(neighbor, neighbor_cost, current_node)
            heapq.heappush(frontier, neighbor_node)

    return None  # No path found

def reconstruct_path(node):
    path = []
    while node:
        path.insert(0, node.state)
        node = node.parent
    return path

if __name__ == "__main__":
    
    graph = {
        's': [('a', 3), ('b', 1)],
        'a': [('b', 2), ('c', 2)],
        'b': [('c', 3)],
        'c': [('d', 4), ('g', 4)],
        'd': [('g', 1)],
        'g': [],
    }

    start_state = 's'
    goal_state = 'g'
    path = uniform_cost_search(start_state, goal_state, graph)

    if path:
        print("Path found:", path)
    else:
        print("No path found.")
