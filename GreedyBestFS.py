import heapq
import networkx as nx

g = nx.Graph()

# Tambahkan nodes
g.add_nodes_from(["Indonesia", "Malaysia", "Singapore", "Thailand", "Kamboja", "China", "Jepang", "Laos","Korea", "Timor Leste", "Vietnam", "Amerika", "Inggris", "Brunei", "FIlipina", "Zimbabwe", "Papua New Guinea"])

# Tambahkan edges
g.add_edge("Indonesia", "Singapore", weight_=132)
g.add_edge("Indonesia", "Kamboja", weight_=122)
g.add_edge("Indonesia", "Thailand", weight_=134)
g.add_edge("Singapore", "Kamboja", weight_=130)
g.add_edge("Singapore", "China", weight_=144)
g.add_edge("Kamboja", "Thailand", weight_=129)
g.add_edge("Kamboja", "Jepang", weight_=148)
g.add_edge("China", "Jepang", weight_=133)
g.add_edge("China", "Laos", weight_=170)
g.add_edge("China", "Korea", weight_=142)
g.add_edge("Jepang", "Laos", weight_=140)
g.add_edge("Korea", "Timor Leste", weight_=114)
g.add_edge("Laos", "Malaysia", weight_=172)
g.add_edge("Timor Leste", "Malaysia", weight_=112)
g.add_edge("Malaysia", "Brunei", weight_=144)
g.add_edge("Malaysia", "Vietnam", weight_=125)
g.add_edge("Brunei", "FIlipina", weight_=152)
g.add_edge("Vietnam", "Amerika", weight_=178)
g.add_edge("Amerika", "Inggris", weight_=199)
g.add_edge("FIlipina", "Zimbabwe", weight_=131)
g.add_edge("Zimbabwe", "Papua New Guinea", weight_=154)
node_sumber = "Indonesia"

def heuristic(node, goal):
    # Koordinat geografis (latitude, longitude) node
    node_coords = {
        "Indonesia": (0, 0),
        "Malaysia": (1, 1),
        "Singapore": (1, 0),
        "Thailand": (1, 2),
        "Kamboja": (1, 3),
        "China": (2, 1),
        "Jepang": (2, 3),
        "Laos": (2, 2),
        "Korea": (2, 4),
        "Timor Leste": (0, 1),
        "Vietnam": (1, 4),
        "Amerika": (3, 5),
        "Inggris": (3, 6),
        "Brunei": (1, 5),
        "FIlipina": (1, 6),
        "Zimbabwe": (4, 7),
        "Papua New Guinea": (4, 8)
    }

    # Menghitung Manhattan distance antara node dan goal
    node_coord = node_coords.get(node, (0, 0))
    goal_coord = node_coords.get(goal, (0, 0))

    distance = abs(node_coord[0] - goal_coord[0]) + abs(node_coord[1] - goal_coord[1])
   
    return distance
    

def greedy_best_first_search(graph, start, goal, heuristic):
    open_list = [(heuristic(start, goal), start)]

    # Inisialisasi dictionary came_from
    came_from = {}

    while open_list:
        _, current = open_list.pop(0)

        #goal ketemu return path

        if current == goal:
            path = []
            while current != start:
                path.insert(0, current)
                current = came_from[current]
            path.insert(0, start)
            return path

    #tidak ketemu, cek neighbor
        for neighbor in graph[current]:
            if neighbor not in came_from:
                came_from[neighbor] = current
                #Cek heuristic paling rendah
                open_list.append((heuristic(neighbor, goal), neighbor))
                open_list.sort(key=lambda x: x[0])

    return None

greedy_best_first_path = greedy_best_first_search(g, "Indonesia", "Malaysia", heuristic)

# Hitung total cost
total_cost = 0
for node1, node2 in zip(greedy_best_first_path[:-1], greedy_best_first_path[1:]):
    edge_weight = g[node1][node2]['weight_']
    total_cost += edge_weight

print("Greedy Best First Path:", greedy_best_first_path)
print("Total Cost:", total_cost)
