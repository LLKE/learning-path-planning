import heapq
import math
import numpy as np
from scipy.interpolate import CubicSpline

def hybrid_a_star(grid, start, goal, steps):
    """Hybrid A* pathfinding algorithm with basic kinematic constraints."""
    ## Note: This implementation still uses a grid based graph traversal
    ## This is not technically real Hybrid A*. Implementation ongoing :)

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
            return smooth_path(path[::-1])
        
        for neighbor in get_neighbors(current, parent[current]):
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

def get_neighbors(node, parent):
    """Generate neighbors considering vehicle's kinematic constraints."""
    x, y = node
    neighbors = []
    directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    
    if parent:
        px, py = parent
        parent_direction = math.atan2(y - py, x - px)
        for dx, dy in directions:
            neighbor_direction = math.atan2(dy, dx)
            angle_diff = abs(math.degrees(neighbor_direction - parent_direction))
            if angle_diff > 180:
                angle_diff = 360 - angle_diff  # Adjust for wrap-around angles
            if angle_diff <= 45:  # Basic constraint for max 45 degrees turn
                neighbors.append((x + dx, y + dy))
    else:
        for dx, dy in directions:
            neighbors.append((x + dx, y + dy))
    
    return neighbors

def line_of_sight(grid, start, end):
    """Checks if there's a line of sight between two points on the grid."""
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    error = dx - dy
    
    while (x0, y0) != (x1, y1):
        if grid[x0][y0] == 1:
            return False
        e2 = 2 * error
        if e2 > -dy:
            error -= dy
            x0 += sx
        if e2 < dx:
            error += dx
            y0 += sy
    
    return grid[x1][y1] == 0

def smooth_path(path):
    """Smooth the path using cubic splines."""
    if len(path) < 3:
        return path
    
    x = [point[0] for point in path]
    y = [point[1] for point in path]
    
    cs_x = CubicSpline(range(len(x)), x)
    cs_y = CubicSpline(range(len(y)), y)
    
    # Generate more points along the spline
    t_new = np.linspace(0, len(x) - 1, num=len(x) * 10)
    smoothed_path = [(cs_x(t), cs_y(t)) for t in t_new]
    
    return smoothed_path