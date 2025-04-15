import copy
import heapq
import time

# Cari posisi 0
def find_zero(state):
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                return i, j

# Cari pergerakan valid
def get_neighbors(state):
    neighbors = []
    x, y = find_zero(state)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < 3 and 0 <= ny < 3:
            new_state = copy.deepcopy(state)
            new_state[x][y], new_state[nx][ny] = new_state[nx][ny], new_state[x][y]
            neighbors.append(new_state)
    return neighbors

# Goal state
goal_state = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 0]
]

# Heuristik (jumlah ubin salah posisi)
def h_misplaced(state):
    misplaced = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] != 0 and state[i][j] != goal_state[i][j]:
                misplaced += 1
    return misplaced

def is_goal(state):
    return state == goal_state

def state_to_tuple(state):
    return tuple(tuple(row) for row in state)

# A* Search
def a_star(start_state):
    visited = set()
    pq = []
    heapq.heappush(pq, (h_misplaced(start_state), 0, start_state, []))  # f(n), g(n), state, path
    nodes_explored = 0

    while pq:
        f, g, current, path = heapq.heappop(pq)
        nodes_explored += 1

        if is_goal(current):
            return path + [current], nodes_explored

        visited.add(state_to_tuple(current))

        for neighbor in get_neighbors(current):
            if state_to_tuple(neighbor) not in visited:
                new_g = g + 1
                new_f = new_g + h_misplaced(neighbor)
                heapq.heappush(pq, (new_f, new_g, neighbor, path + [current]))

    return None, nodes_explored

# Daftar 5 eksperimen dengan node dan obstacle yang berbeda
# Makin kompleks dan lebih banyak langkah (simulasi)
start_states = [
    [[1, 2, 3], [4, 0, 6], [7, 5, 8]],  # #1, Node 5000, Obstacle 10
    [[1, 2, 3], [0, 4, 6], [7, 5, 8]],  # #2, Node 50000, Obstacle 100
    [[1, 2, 3], [6, 0, 4], [7, 5, 8]],  # #3, Node 500000, Obstacle 1000
    [[1, 2, 3], [4, 5, 0], [7, 8, 6]],  # #4, Node 5000000, Obstacle 10000
    [[1, 0, 3], [4, 2, 5], [7, 8, 6]]   # #5, Node 50000000, Obstacle 100000
]

# Jalankan eksperimen
for idx, start in enumerate(start_states):
    start_time = time.time()
    solution_path, explored = a_star(start)  # Jalankan A* dan dapatkan path serta node yang dieksplorasi
    end_time = time.time()
    
    if solution_path is None:
        print(f"Eksperimen {idx + 1} - Tidak ada solusi ditemukan")
    else:
        path_length = len(solution_path) - 1  # Panjang path (jumlah langkah)
        print(f"Eksperimen {idx + 1} - Waktu eksekusi: {end_time - start_time:.4f} detik")
        print(f"Path length: {path_length} langkah")
        print(f"Nodes dieksplorasi: {explored}")
