import heapq
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

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

def theta_star(grid, start, goal, steps):
    """Theta* pathfinding algorithm with animation support."""
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    g_cost = {start: 0}
    parent = {start: None}
    explored = []
    
    while open_set:
        _, current = heapq.heappop(open_set)
        explored.append(current)
        steps.append((explored.copy(), parent.copy()))
        
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
    
    return None  

def a_star(grid, start, goal, steps):
    """A* pathfinding algorithm with animation support."""
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    g_cost = {start: 0}
    parent = {start: None}
    explored = []
    
    while open_set:
        _, current = heapq.heappop(open_set)
        explored.append(current)
        steps.append((explored.copy(), parent.copy()))
        
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parent[current]
            return path[::-1]
        
        for dx, dy in [(0, -1), (-1, 0), (0, 1), (1, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0:
                new_g = g_cost[current] + 1
                
                if neighbor in g_cost and new_g >= g_cost[neighbor]:
                    continue
                
                parent[neighbor] = current
                g_cost[neighbor] = new_g
                
                heapq.heappush(open_set, (g_cost[neighbor] + math.dist(neighbor, goal), neighbor))
    
    return None

def animate_pathfinding(grid, steps):
    """Animates the pathfinding process."""
    fig, ax = plt.subplots()
    grid = np.array(grid)
    ax.imshow(grid, cmap="Greys", origin="upper")
    
    def update(frame):
        ax.clear()
        ax.imshow(grid, cmap="Greys", origin="upper")
        
        explored, parent = steps[frame]
        for (x, y) in explored:
            ax.plot(y, x, "ro", markersize=3)
        for node, p in parent.items():
            if p is not None:
                ax.plot([p[1], node[1]], [p[0], node[0]], "orange", linewidth=1)
    
    ani = animation.FuncAnimation(fig, update, frames=len(steps), repeat=False, interval=400)
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

steps_theta = []
path_theta = theta_star(grid, start, goal, steps_theta)
animate_pathfinding(grid, steps_theta)

steps_a_star = []
path_a_star = a_star(grid, start, goal, steps_a_star)
animate_pathfinding(grid, steps_a_star)
