"""
Grid-Based Pathfinding Algorithms

This module implements four pathfinding algorithms (BFS, DFS, Dijkstra, A*) 
designed to yield step-by-step states for real-time UI visualization.
"""

from collections import deque
import heapq
import time
import numpy as np
import json
import os

# ==========================================
# CONSTANTS & CONFIGURATION
# ==========================================
EMPTY = 0
WALL = 1
START = 2
GOAL = 3
VISITED = 4
FRONTIER = 5
PATH = 6

# Four-direction movement (Up, Down, Left, Right)
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


# ==========================================
# HELPER FUNCTIONS
# ==========================================
def get_start(grid):
    """Starting node is always (0,0)"""
    return (0, 0)

def get_goal(grid):
    """Goal node is always bottom-right corner."""
    rows, cols = grid.shape
    return (rows - 1, cols - 1)

def is_valid(grid, row, col):
    """Check if a cell is inside the grid and not an obstacle."""
    rows, cols = grid.shape
    return (0 <= row < rows and 0 <= col < cols and grid[row][col] != WALL)

def get_neighbors(grid, node):
    """Return all valid neighboring cells."""
    row, col = node
    neighbors = []
    for dr, dc in DIRECTIONS:
        nr, nc = row + dr, col + dc
        if is_valid(grid, nr, nc):
            neighbors.append((nr, nc))
    return neighbors

def manhattan(node, goal):
    """Manhattan distance heuristic."""
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def reconstruct_path(parent, goal):
    """Build the shortest path."""
    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path

def build_metrics(name, start_time, visited, path):
    """Calculates metrics and automatically saves them to a JSON file."""
    execution_time = round((time.perf_counter() - start_time) * 1000, 3)
    
    metrics = {
        "algorithm_name": name,
        "execution_time_ms": execution_time,
        "nodes_explored": len(visited),
        "path_length": len(path) if path else 0,
        "path_found": path is not None
    }
    
    # Save output to JSON
    os.makedirs("outputs", exist_ok=True)
    safe_name = name.replace(" ", "_").replace("'", "").replace("*", "star").lower()
    output_file = f"outputs/{safe_name}_output.json"
    
    output_data = {
        "metrics": metrics,
        "final_path": path
    }
    
    with open(output_file, "w") as f:
        json.dump(output_data, f, indent=4)
        
    return metrics


# ==========================================
# ALGORITHMS
# ==========================================
def bfs(grid):
    """Breadth-First Search (Generator)"""
    start = get_start(grid)
    goal = get_goal(grid)
    start_time = time.perf_counter()
    
    queue = deque([start])
    visited = {start}
    parent = {start: None}

    while queue:
        current = queue.popleft()

        yield {
            "current_node": current,
            "visited": visited.copy(),
            "frontier": list(queue),
            "path": None
        }

        if current == goal:
            path = reconstruct_path(parent, goal)
            yield {
                "current_node": goal,
                "visited": visited.copy(),
                "frontier": [],
                "path": path,
                "metrics": build_metrics("Breadth-First Search", start_time, visited, path)
            }
            return

        for neighbor in get_neighbors(grid, current):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current
                queue.append(neighbor)

    yield {
        "current_node": None,
        "visited": visited.copy(),
        "frontier": [],
        "path": None,
        "metrics": build_metrics("Breadth-First Search", start_time, visited, None)
    }

def dfs(grid):
    """Depth-First Search (Generator)"""
    start = get_start(grid)
    goal = get_goal(grid)
    start_time = time.perf_counter()

    stack = [start]
    visited = {start}
    parent = {start: None}

    while stack:
        current = stack.pop()

        yield {
            "current_node": current,
            "visited": visited.copy(),
            "frontier": list(stack),
            "path": None
        }

        if current == goal:
            path = reconstruct_path(parent, goal)
            yield {
                "current_node": goal,
                "visited": visited.copy(),
                "frontier": [],
                "path": path,
                "metrics": build_metrics("Depth-First Search", start_time, visited, path)
            }
            return

        for neighbor in reversed(get_neighbors(grid, current)):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = current
                stack.append(neighbor)

    yield {
        "current_node": None,
        "visited": visited.copy(),
        "frontier": [],
        "path": None,
        "metrics": build_metrics("Depth-First Search", start_time, visited, None)
    }

def dijkstra(grid):
    """Dijkstra's Algorithm (Generator)"""
    start = get_start(grid)
    goal = get_goal(grid)
    start_time = time.perf_counter()

    pq = []
    heapq.heappush(pq, (0, start))
    distances = {start: 0}
    parent = {start: None}
    visited = set()

    while pq:
        current_distance, current = heapq.heappop(pq)

        if current in visited:
            continue

        visited.add(current)

        yield {
            "current_node": current,
            "visited": visited.copy(),
            "frontier": [node for _, node in pq],
            "path": None
        }

        if current == goal:
            path = reconstruct_path(parent, goal)
            yield {
                "current_node": goal,
                "visited": visited.copy(),
                "frontier": [],
                "path": path,
                "metrics": build_metrics("Dijkstra Algorithm", start_time, visited, path)
            }
            return

        for neighbor in get_neighbors(grid, current):
            new_distance = current_distance + 1
            if neighbor not in distances or new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                parent[neighbor] = current
                heapq.heappush(pq, (new_distance, neighbor))

    yield {
        "current_node": None,
        "visited": visited.copy(),
        "frontier": [],
        "path": None,
        "metrics": build_metrics("Dijkstra Algorithm", start_time, visited, None)
    }

def astar(grid):
    """A* Search Algorithm (Generator)"""
    start = get_start(grid)
    goal = get_goal(grid)
    start_time = time.perf_counter()

    pq = []
    heapq.heappush(pq, (manhattan(start, goal), 0, start))
    g_score = {start: 0}
    parent = {start: None}
    visited = set()

    while pq:
        _, current_cost, current = heapq.heappop(pq)

        if current in visited:
            continue

        visited.add(current)

        yield {
            "current_node": current,
            "visited": visited.copy(),
            "frontier": [node for _, _, node in pq],
            "path": None
        }

        if current == goal:
            path = reconstruct_path(parent, goal)
            yield {
                "current_node": goal,
                "visited": visited.copy(),
                "frontier": [],
                "path": path,
                "metrics": build_metrics("A-Star Search", start_time, visited, path)
            }
            return

        for neighbor in get_neighbors(grid, current):
            tentative_g = current_cost + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                parent[neighbor] = current
                f_score = tentative_g + manhattan(neighbor, goal)
                heapq.heappush(pq, (f_score, tentative_g, neighbor))

    yield {
        "current_node": None,
        "visited": visited.copy(),
        "frontier": [],
        "path": None,
        "metrics": build_metrics("A-Star Search", start_time, visited, None)
    }

def load_grid(filename):
    """Parses the text file to build the grid environment."""
    if not os.path.exists(filename):
        raise FileNotFoundError(f"Cannot find {filename}. Ensure the file exists.")
        
    with open(filename, "r") as file:
        lines = [line.strip() for line in file if line.strip()]

    size = int(lines[0])
    grid = np.zeros((size, size), dtype=int)

    for line in lines[1:]:
        row, col = map(int, line.split(","))
        grid[row][col] = WALL

    return grid


# ==========================================
# ISOLATED TESTING BLOCK
# ==========================================
if __name__ == "__main__":
    # This code will ONLY execute if this file is run directly, 
    # protecting your UI from accidental execution during imports.
    try:
        test_grid = load_grid("input.txt")
        print("Grid loaded successfully. Running algorithms...\n")
        
        for algorithm in [bfs, dfs, dijkstra, astar]:
            final_state = None
            for state in algorithm(test_grid):
                final_state = state
            
            print(f"Results for {final_state['metrics']['algorithm_name']}:")
            print(final_state["metrics"])
            print("-" * 40)
            
    except FileNotFoundError as e:
        print(e)