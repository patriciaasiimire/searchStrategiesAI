
from collections import defaultdict

graph = defaultdict(list)
def addArc(graph,u,v):
	graph[u].append(v)

def generate_arc(graph):
	arc = []

	for node in graph:
		
		
		for neighbour in graph[node]:
			
			arc.append((node, neighbour))
	return arc

addArc(graph, 's', 'a')
addArc(graph, 's', 'b')
addArc(graph, 'a', 'b')
addArc(graph, 'a', 'c')
addArc(graph, 'b', 'c')
addArc(graph, 'c', 'd')
addArc(graph, 'c', 'g')
addArc(graph, 'd', 'g')

print(generate_arc(graph))
