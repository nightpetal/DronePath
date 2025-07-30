import heapq
from collections import deque
from typing import List, Tuple, Dict, Optional

Grid = List[List[str]]
Coord = Tuple[int, int]


def manhattan(p1: Coord, p2: Coord) -> int:
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])


def find_start_goal(grid: Grid) -> Tuple[Optional[Coord], Optional[Coord]]:
    start = goal = None
    for r, row in enumerate(grid):
        for c, val in enumerate(row):
            if val == "S":
                start = (r, c)
            elif val == "G":
                goal = (r, c)
    return start, goal


def neighbors(pos: Coord, grid: Grid) -> List[Coord]:
    r, c = pos
    moves = [(r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)]
    return [
        (nr, nc)
        for nr, nc in moves
        if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]) and grid[nr][nc] != "X"
    ]


def reconstruct_path(came_from: Dict[Coord, Coord], current: Coord) -> List[Coord]:
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    return path[::-1]


def bfs(grid: Grid, start: Coord, goal: Coord) -> Optional[List[Coord]]:
    queue = deque([start])
    came_from = {}
    visited = set([start])
    while queue:
        current = queue.popleft()
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in neighbors(current, grid):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                queue.append(neighbor)
    return None


def dfs(grid: Grid, start: Coord, goal: Coord) -> Optional[List[Coord]]:
    stack = [start]
    came_from = {}
    visited = set([start])
    while stack:
        current = stack.pop()
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in neighbors(current, grid):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                stack.append(neighbor)
    return None


def ucs(grid: Grid, start: Coord, goal: Coord) -> Optional[List[Coord]]:
    heap = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}
    while heap:
        cost, current = heapq.heappop(heap)
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in neighbors(current, grid):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(heap, (new_cost, neighbor))
    return None


def greedy(grid: Grid, start: Coord, goal: Coord) -> Optional[List[Coord]]:
    heap = [(manhattan(start, goal), start)]
    came_from = {}
    visited = set([start])
    while heap:
        _, current = heapq.heappop(heap)
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in neighbors(current, grid):
            if neighbor not in visited:
                visited.add(neighbor)
                came_from[neighbor] = current
                heapq.heappush(heap, (manhattan(neighbor, goal), neighbor))
    return None


def astar(grid: Grid, start: Coord, goal: Coord) -> Optional[List[Coord]]:
    heap = [(manhattan(start, goal), 0, start)]
    came_from = {}
    cost_so_far = {start: 0}
    while heap:
        _, cost, current = heapq.heappop(heap)
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in neighbors(current, grid):
            new_cost = cost_so_far[current] + 1
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + manhattan(neighbor, goal)
                heapq.heappush(heap, (priority, new_cost, neighbor))
    return None


def display_grid(grid: Grid, path: Optional[List[Coord]] = None):
    BLUE = "\033[94m"
    RESET = "\033[0m"
    display = [row.copy() for row in grid]
    path_set = set(path) if path else set()

    print("\nGrid:")
    for r in range(len(grid)):
        row_str = ""
        for c in range(len(grid[0])):
            cell = display[r][c]
            if (r, c) in path_set and cell not in ("S", "G"):
                row_str += f"{BLUE}*{RESET} "
            else:
                row_str += f"{cell} "
        print(row_str.strip())


def get_algorithm(name: str):
    return {"1": dfs, "2": bfs, "3": ucs, "4": astar, "5": greedy}.get(name)


def run():
    preset_grid = [
        list("S...X..."),
        list(".X.X...X"),
        list("..X..X.."),
        list("X...X..G"),
        list("XX.XX.XX"),
    ]

    while True:
        print("\nSelect Algorithm:")
        print("1. Depth-First Search (DFS)")
        print("2. Breadth-First Search (BFS)")
        print("3. Uniform-Cost Search (UCS)")
        print("4. A* Search")
        print("5. Greedy Best-First Search")
        print("6. Exit")
        choice = input("Enter choice: ").strip()
        if choice == "6":
            break

        algo = get_algorithm(choice)
        if not algo:
            print("Invalid selection.")
            continue

        grid = preset_grid
        start, goal = find_start_goal(grid)
        if not start or not goal:
            print("Start or Goal missing.")
            continue

        print("\nInitial Grid:")
        display_grid(grid)
        print(f"\nRunning {algo.__name__.upper()}...")

        path = algo(grid, start, goal)
        if path:
            print("\nPath Found:")
            display_grid(grid, path)
        else:
            print("\nNo path found.")

        input("\nPress Enter to try again...")


if __name__ == "__main__":
    run()
