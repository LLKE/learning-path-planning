import numpy as np
import heapq
from dubins import DubinsPath
import math
import pdb

def heuristic(node, goal, turning_radius):
    """Heuristic function using Euclidean distance."""
    path = DubinsPath(node, goal, turning_radius)
    result = path.compute_shortest_path()
    return result["length"] if result else np.inf

def is_valid(node, grid):
    x, y, _ = node
    if x < 0 or x >= grid.shape[0] or y < 0 or y >= grid.shape[1]:
        return False
    return grid[int(x), int(y)] == 0

def snap_to_grid(node):
    """Snap a point to the nearest integer for x and y, and keep theta unchanged."""
    x, y, theta = node
    # snapping to 0.25 sized grid
    rounded_x = round(4 * x)/4
    rounded_y = round(4 * y)/4
    # snapping to pi/8 angles
    rounded_theta = round(theta / (math.pi/8)) * (math.pi/8)
    return (rounded_x, rounded_y, rounded_theta)

def is_arc_collision_free(x, y, theta, omega, v, delta_t, grid):
    dt_sample = 0.1
    t = 0
    while t < delta_t:
        xi = x + v*math.cos(theta + omega*t)*dt_sample
        yi = y + v*math.sin(theta + omega*t)*dt_sample
        if 0 <= int(xi) < len(grid[0]) and 0 <= int(yi) < len(grid[1]):
            if grid[int(xi), int(yi)] == 1: return False
            
        t += dt_sample
    return True

def get_neighbors(node, turning_radius, grid, v=1.0, delta_t=1.0):

    x, y, theta = node
    neighbors = []

    # Define possible steering angles between left and right
    max_delta = math.atan2(1, turning_radius)
    num_angles = 5
    steering_angles = np.linspace(-max_delta, max_delta, num_angles)

    for steering_angle in steering_angles:
        # Compute the change in orientation (dθ/dt = ω)
        omega = v * math.tan(steering_angle)  # Angular velocity
        new_theta = theta + omega * delta_t

        # Compute the new position using dx/dt and dy/dt
        if abs(steering_angle) < 1e-6: # Straight
            new_x = x + v * math.cos(theta) * delta_t
            new_y = y + v * math.sin(theta) * delta_t
        else: # Compute circular arc endpoint
            R = 1 / math.tan(steering_angle)
            new_theta = theta + (v/R) * delta_t
            new_x = x + R * (math.sin(new_theta) - math.sin(theta))
            new_y = y - R * (math.cos(new_theta) - math.cos(theta))

        # Snap the new position to the grid and add to neighbors
        neighbor = snap_to_grid((new_x, new_y, new_theta))
        if neighbor != node and is_arc_collision_free(new_x, new_y, new_theta, omega, v, delta_t, grid):  # Avoid adding the current node as its own neighbor
            neighbors.append(neighbor)
            
    return neighbors
    
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def is_on_path(goal, current, neighbor):
    """
    Check if the goal lies on the line segment between current and neighbor.
    :param goal: Goal node (x, y)
    :param neighbor: Neighbor node (x, y)
    :param current: Current node (x, y)
    :return: True if the goal lies on the line segment, False otherwise
    """
    x1, y1 = current[:2]
    x2, y2 = neighbor[:2]
    xg, yg = goal[:2]

    # Check if the goal satisfies the line equation
    if math.isclose((yg - y1) * (x2 - x1), (y2 - y1) * (xg - x1), rel_tol=1e-9):
        # Check if the goal lies within the bounds of the segment
        if min(x1, x2) <= xg <= max(x1, x2) and min(y1, y2) <= yg <= max(y1, y2):
            return True

    return False

def hybrid_a_star(grid, start, goal, turning_radius, steps):
    # Snap start and goal to the nearest grid cell
    start = snap_to_grid(start)
    goal = snap_to_grid(goal)

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal, turning_radius)}
    explored = set()
    #pdb.set_trace()
    while open_set:
        _, current = heapq.heappop(open_set)
        if current in explored:
            continue
        explored.add(current)
        steps.append((current, came_from.get(current)))
        
        # Check if the current cell is the goal cell (relaxing theta condition)
        if math.isclose(current[0], goal[0], rel_tol=1e-9) and math.isclose(current[1], goal[1], rel_tol=1e-9):  # Compare only x and y coordinates
            return reconstruct_path(came_from, current)
        
        for neighbor in get_neighbors(current, turning_radius, grid):
            if (is_on_path(goal, current, neighbor)):
                explored.add(goal)
                came_from[goal] = current
                steps.append((current, came_from.get(current)))
                return reconstruct_path(came_from, goal)

            if not is_valid(neighbor, grid):
                continue
            
            tentative_g_score = g_score[current] + np.linalg.norm(np.array(current[:2]) - np.array(neighbor[:2]))
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal, turning_radius)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # No path found
