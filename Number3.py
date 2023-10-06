#dfs tree search
graph = {
  's' : ['a','b'],
  'a' : ['b', 'c'],
  'b' : ['c'],
  'c' : ['d', 'g'],
  'd' : ['g'],
  'g' : []
}

visited = set()

def dfs(visited, graph, node):
    if node not in visited:
        print (node)
        visited.add(node)
        for neighbour in graph[node]:
            dfs(visited, graph, neighbour)

print("Following is the Depth-First Search")
dfs(visited, graph, 's')


#bfs tree search
graph = {
  's' : ['a','b'],
  'a' : ['b', 'c'],
  'b' : ['c'],
  'c' : ['d', 'g'],
  'd' : ['g'],
  'g' : []
}

visited = [] 
queue = []     

def bfs(visited, graph, node):
  visited.append(node)
  queue.append(node)

  while queue:
    s = queue.pop(0) 
    print (s, end = " ") 

    for neighbour in graph[s]:
      if neighbour not in visited:
        visited.append(neighbour)
        queue.append(neighbour)

bfs(visited, graph, 's')


#ucs tree search
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

    return None  

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
        'g': []
    }

    start_state = 's'
    goal_state = 'g'
    path = uniform_cost_search(start_state, goal_state, graph)

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

def greedy_tree_search(start_state, goal_state, graph, heuristic_fn):
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

    result = greedy_tree_search(start_state, goal_state, graph, heuristic_fn)

    if result:
        print("Goal state found:", result)
    else:
        print("No path found.")



#A * search
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

def astar_tree_search(start_state, goal_state, heuristic_fn):
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

        for neighbor, step_cost in get_neighbors(current_state):
            g_cost = current_node.g_cost + step_cost
            h_cost = heuristic_fn(neighbor)
            neighbor_node = Node(neighbor, g_cost, h_cost, current_node)
            heapq.heappush(frontier, neighbor_node)

    return None  # No path found

def reconstruct_path(node):
    path = []
    while node:
        path.insert(0, node.state)
        node = node.parent
    return path

# Example usage:
if __name__ == "__main__":
    # Define the heuristic function for your specific problem (e.g., straight-line distance to the goal).
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

    # Define the function to get neighbors and step costs for a given state.
    def get_neighbors(state):
        neighbors = {
        's': [('a', 3), ('b', 1)],
        'a': [('b', 2), ('c', 2)],
        'b': [('c', 3)],
        'c': [('d', 4), ('g', 4)],
        'd': [('g', 1)],
        'g': []
        }
        return neighbors.get(state, [])

    start_state = 's'
    goal_state = 'g'
    path = astar_tree_search(start_state, goal_state, heuristic_fn)

    if path:
        print("Path found:", path)
    else:
        print("No path found.")
