import numpy as np
import heapq
from dubins import DubinsPath
import math
import pdb

def heuristic(node, goal, turning_radius):
    """Heuristic function using Euclidean distance."""
    return np.linalg.norm(np.array(goal[:2]) - np.array(node[:2]))

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

def get_neighbors(node, turning_radius, v=1.0, delta_t=1.0):

    x, y, theta = node
    neighbors = []

    # Define possible steering angles (left, straight, right)
    steering_angles = [math.atan2(1, turning_radius), 0, -math.atan2(1, turning_radius)]

    for steering_angle in steering_angles:
        # Compute the change in orientation (dθ/dt = ω)
        omega = v * math.tan(steering_angle)  # Angular velocity
        new_theta = theta + omega * delta_t

        # Compute the new position using dx/dt and dy/dt
        new_x = x + v * math.cos(new_theta) * delta_t
        new_y = y + v * math.sin(new_theta) * delta_t

        # Snap the new position to the grid and add to neighbors
        neighbor = snap_to_grid((new_x, new_y, new_theta))
        if neighbor != node:  # Avoid adding the current node as its own neighbor
            neighbors.append(neighbor)
            
    return neighbors
    
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def reconstruct_path_with_dubins(came_from, current, turning_radius):

    # Reconstruct the path as a sequence of nodes
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()

    # Convert the path into Dubins paths
    dubins_path = []
    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]
        dubins = DubinsPath(start, end, turning_radius)
        shortest_path = dubins.compute_shortest_path()
        if shortest_path:
            dubins_path.extend(shortest_path["segments"])  # Add the Dubins path segments

    return dubins_path

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

def is_goal_in_wedge(current, neighbors, goal, max_distance=1.0):
    
    x0, y0 = current[:2]
    xg, yg = goal[:2]

    if np.linalg.norm([xg - x0, yg - y0]) > max_distance:
        return False
    
    # Angles to neighbors
    neighbor_angles = [math.atan2(n[1] - y0, n[0] - x0) for n in neighbors]

    if len(neighbor_angles) < 2:
        return False
    
    left_angle = min(neighbor_angles)
    right_angle = max(neighbor_angles)

    # Angle to goal
    goal_angle = math.atan2(yg - y0, xg - x0)

    # Normalize angles to [0, 2pi)
    def norm_angle(a): return (a + 2 * math.pi) % (2 * math.pi)
    left_angle = norm_angle(left_angle)
    right_angle = norm_angle(right_angle)
    goal_angle = norm_angle(goal_angle)

    # Handle wedge crossing 0 radians
    if left_angle > right_angle:
        return goal_angle >= left_angle or goal_angle <= right_angle
    else:
        return left_angle <= goal_angle <= right_angle
    
    
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
    #pdb.set_trace()
    while open_set:
        _, current = heapq.heappop(open_set)
        explored.append(current)
        steps.append((explored.copy(), came_from.copy()))
        
        # Check if the current cell is the goal cell (relaxing theta condition)
        if math.isclose(current[0], goal[0], rel_tol=1e-9) and math.isclose(current[1], goal[1], rel_tol=1e-9):  # Compare only x and y coordinates
            return reconstruct_path(came_from, current)
        
        neighbors = get_neighbors(current, turning_radius)
        # Check if goal is in the wedge between leftmost and rightmost neighbor
        if is_goal_in_wedge(current, neighbors, goal):
            explored.append(goal)
            came_from[goal] = current
            steps.append((explored.copy(), came_from.copy()))
            return reconstruct_path(came_from, goal)
        
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
