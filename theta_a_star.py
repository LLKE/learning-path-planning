import heapq
import math
import matplotlib.pyplot as plt
import numpy as np

def line_of_sight(grid, start, end):
    """Checks if there's a line of sight between two points on the grid."""
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    err = dx - dy
    
    while (x0, y0) != (x1, y1):
        if grid[y0][x0] == 1:
            return False
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    
    return grid[y1][x1] == 0

def theta_star(grid, start, goal):
    """Theta* pathfinding algorithm on a 2D grid."""
    rows, cols = len(grid), len(grid[0])
    open_set = []  # Min-heap priority queue
    heapq.heappush(open_set, (0, start))
    g_cost = {start: 0}
    parent = {start: None}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parent[current]
            return path[::-1]
        
        for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0:
                new_g = g_cost[current] + math.dist(current, neighbor)
                
                if neighbor in g_cost and new_g >= g_cost[neighbor]:
                    continue
                
                if parent[current] and line_of_sight(grid, parent[current], neighbor):
                    parent[neighbor] = parent[current]
                    g_cost[neighbor] = g_cost[parent[current]] + math.dist(parent[current], neighbor)
                else:
                    parent[neighbor] = current
                    g_cost[neighbor] = new_g
                
                heapq.heappush(open_set, (g_cost[neighbor] + math.dist(neighbor, goal), neighbor))
    
    return None  # No path found

def a_star(grid, start, goal):
    """A* pathfinding algorithm on a 2D grid."""
    rows, cols = len(grid), len(grid[0])
    open_set = []  # Min-heap priority queue
    heapq.heappush(open_set, (0, start))
    g_cost = {start: 0}
    parent = {start: None}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parent[current]
            return path[::-1]
        
        for dx, dy in [(0, -1), (-1, 0), (0, 1), (1, 0)]:  # Only 4-directional movement
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0:
                new_g = g_cost[current] + 1
                
                if neighbor in g_cost and new_g >= g_cost[neighbor]:
                    continue
                
                parent[neighbor] = current
                g_cost[neighbor] = new_g
                
                heapq.heappush(open_set, (g_cost[neighbor] + math.dist(neighbor, goal), neighbor))
    
    return None  # No path found

def plot_grid(grid, path, title, color="red"):
    """Visualize the grid, obstacles, and path."""
    grid = np.array(grid)
    plt.imshow(grid, cmap="Greys", origin="upper")
    if path:
        x, y = zip(*path)
        plt.plot(y, x, marker="o", color=color)
    plt.title(title)
    plt.grid(True)
    plt.show()

# Example usage:
grid = [
    [0, 0, 0, 0, 1],
    [0, 1, 1, 0, 1],
    [0, 0, 0, 0, 0],
    [1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0]
]
start = (0, 0)
goal = (4, 4)
path_theta = theta_star(grid, start, goal)
path_a_star = a_star(grid, start, goal)
print("Theta* Path:", path_theta)
print("A* Path:", path_a_star)
plot_grid(grid, path_theta, "Theta* Path", color="blue")
plot_grid(grid, path_a_star, "A* Path", color="red")
