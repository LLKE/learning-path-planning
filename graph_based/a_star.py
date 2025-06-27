import heapq
import math  

DIRECTIONS = [(0, -1), (-1, 0), (0, 1), (1, 0)]  # Up, Left, Down, Right

def heuristic(a, b):
    # manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(grid, start, goal, steps):
    """A* pathfinding algorithm with animation support."""
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, math.dist(start, goal), start))
    g_cost = {start: 0}
    parent = {start: None}
    explored = []
    
    while open_set:
        _, _, current = heapq.heappop(open_set)
        if current in explored:
            continue
        explored.append(current)
        steps.append((current, parent.get(current)))
        
        if current == goal:
            path = []
            while current:
                path.append(current)
                current = parent[current]
            return path[::-1]
        
        for dx, dy in DIRECTIONS:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0:
                new_g = g_cost[current] + 1
                
                if neighbor in g_cost and new_g >= g_cost[neighbor]:
                    continue
                
                parent[neighbor] = current
                g_cost[neighbor] = new_g
                
                # Here, heuristic (distance to goal) comes into play
                heapq.heappush(open_set, (g_cost[neighbor], heuristic(neighbor, goal), neighbor))
    
    return None


