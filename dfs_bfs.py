#dfs
class Graph:
    def __init__(self):
        self.graph = {}

    def add_edge(self, node, neighbors):
        self.graph[node] = neighbors

def dfs(graph, start, goal):
    visited = set()
    stack = [(start, [start])]

    while stack:
        current_node, path = stack.pop()

        if current_node not in visited:
            visited.add(current_node)

            if current_node == goal:
                return path

            for neighbor in graph[current_node]:
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))

    return None


g = Graph()
g.add_edge('s', ['a', 'b'])
g.add_edge('a', ['b', 'c'])
g.add_edge('b', ['c'])
g.add_edge('c', ['d', 'g'])
g.add_edge('d', ['g'])
g.add_edge('g', [])


dfs_path = dfs(g.graph, 's', 'g')
if dfs_path:
    print("DFS Expansion Order:", dfs_path)
    print("Path Returned by DFS:", ' -> '.join(dfs_path))
else:
    print("No path found from 's' to 'g'")

all_nodes = set(g.graph.keys())
expanded_states = set(dfs_path)
unexpanded_states = all_nodes - expanded_states
print("States Not Expanded in DFS:", unexpanded_states)


#bfs
from collections import deque

class Graph:
    def __init__(self):
        self.graph = {}

    def add_edge(self, node, neighbors):
        self.graph[node] = neighbors

def bfs(graph, start, goal):
    visited = set()
    queue = deque([(start, [start])])  

    while queue:
        current_node, path = queue.popleft()

        if current_node not in visited:
            visited.add(current_node)

            if current_node == goal:
                return path  

            for neighbor in graph[current_node]:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

    return None  


g = Graph()
g.add_edge('s', ['a', 'b'])
g.add_edge('a', ['b', 'c'])
g.add_edge('b', ['c'])
g.add_edge('c', ['d', 'g'])
g.add_edge('d', ['g'])
g.add_edge('g', [])


bfs_path = bfs(g.graph, 's', 'g')
if bfs_path:
    print("BFS Expansion Order:", bfs_path)
    print("Path Returned by BFS:", ' -> '.join(bfs_path))
else:
    print("No path found from 's' to 'g'")


all_nodes = set(g.graph.keys())
expanded_states = set(bfs_path)
unexpanded_states = all_nodes - expanded_states
print("States Not Expanded in BFS:", unexpanded_states)

