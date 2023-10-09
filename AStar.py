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

def a_star_search(graph, start, goal):
    # Fungsi heuristik (Manhattan distance) dari setiap node ke goal
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

    # Inisialisasi nilai g(n) dan f(n) dari setiap node dengan nilai tak terhingga
    g = {node: float('inf') for node in graph.nodes()}
    f = {node: float('inf') for node in graph.nodes()}

    # Inisialisasi heap prioritas dengan elemen awal (node sumber)
    open_list = [(0, start)]

    # Nilai g(n) dari node awal adalah 0
    g[start] = 0

    # Inisialisasi dictionary came_from
    came_from = {}

    while open_list:
        # Ambil node dengan f(n) terkecil dari heap prioritas
        f_current, current = heapq.heappop(open_list)

        # Jika sudah mencapai goal, maka selesai
        if current == goal:
            path = []
            while current != start:
                path.insert(0, current)
                current = came_from[current]
            path.insert(0, start)
            return path

        # Iterasi melalui tetangga-tetangga dari node saat ini
        for neighbor in graph[current]:
            # Hitung nilai g(n) sementara untuk tetangga ini
            tentative_g = g[current] + graph[current][neighbor]['weight_']

            # Jika nilai g(n) baru lebih baik daripada yang sebelumnya
            if tentative_g < g[neighbor]:
                # Perbarui nilai g(n) dan f(n) tetangga
                g[neighbor] = tentative_g
                f[neighbor] = tentative_g + heuristic(neighbor, goal)

                # Simpan node yang sebelumnya terbaik untuk tetangga ini
                came_from[neighbor] = current

                # Tambahkan tetangga ke heap prioritas
                heapq.heappush(open_list, (f[neighbor], neighbor))

    # Jika tidak ada jalur yang ditemukan
    return None

a_star_path = a_star_search(g, "Indonesia", "Malaysia")

total_cost = 0
for i in range(len(a_star_path) - 1):
    node1 = a_star_path[i]
    node2 = a_star_path[i + 1]
    edge_weight = g[node1][node2]['weight_']
    total_cost += edge_weight

print("A* Path:", a_star_path)
print("Total Cost:", total_cost)
