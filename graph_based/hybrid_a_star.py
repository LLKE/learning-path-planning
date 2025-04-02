import numpy as np
import heapq
from dubins import DubinsPath  # Assuming DubinsPath is in a separate file

def heuristic(node, goal, turning_radius):
    """Heuristic function using Euclidean distance."""
    return np.linalg.norm(np.array(goal[:2]) - np.array(node[:2]))

def is_valid(node, grid):
    x, y, _ = node
    if x < 0 or x >= grid.shape[0] or y < 0 or y >= grid.shape[1]:
        return False
    return grid[int(x), int(y)] == 0

def snap_to_grid(node):
    """Snap a point to the nearest grid cell. Discretizes map to avoid infinite number of nodes to check"""
    x, y, theta = node
    return (round(x), round(y), theta)

def get_neighbors(node, turning_radius):
    x, y, theta = node
    delta_theta = np.pi / 4  # Discrete steering angles
    step_size = turning_radius / 2
    neighbors = []
    
    for delta in [-1, 0, 1]:
        new_theta = theta + delta * delta_theta
        new_x = x + step_size * np.cos(new_theta)
        new_y = y + step_size * np.sin(new_theta)
        neighbors.append(snap_to_grid((new_x, new_y, new_theta)))  # Snap neighbors to the grid
    
    return neighbors
    
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def hybrid_a_star(grid, start, goal, turning_radius, steps):
    # Snap start and goal to the nearest grid cell
    start = snap_to_grid(start)
    goal = snap_to_grid(goal)

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal, turning_radius)}
    explored = []
    
    while open_set:
        _, current = heapq.heappop(open_set)
        explored.append(current)
        steps.append((explored.copy(), came_from.copy()))
        
        # Check if the current cell is the goal cell (relaxing theta condition)
        if current[:2] == goal[:2]:  # Compare only x and y coordinates
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(current, turning_radius):
            if not is_valid(neighbor, grid):
                continue
            
            tentative_g_score = g_score[current] + np.linalg.norm(np.array(current[:2]) - np.array(neighbor[:2]))
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal, turning_radius)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # No path found
