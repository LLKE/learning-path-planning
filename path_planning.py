import numpy as np
import yaml
from a_star import a_star
from theta_star import theta_star
from hybrid_a_star import hybrid_a_star
from animation import animate_pathfinding

if __name__ == "__main__":
    # Load configuration from YAML file
    with open("config.yaml", "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    
    algorithm = config["algorithm"]
    dim_x = config["grid_size"]["x"]
    dim_y = config["grid_size"]["y"]
    num_obstacles = config["obstacles"]
    start = tuple(config["start"])
    goal = tuple(config["goal"])

    # Check if the map size values are feasible
    if dim_x <= 0 or dim_y <= 0:
        raise ValueError("Grid size must be greater than 0.")
    if num_obstacles >= dim_x * dim_y:
        raise ValueError("Number of obstacles must be less than the total number of grid cells.")
    if not (0 <= start[0] < dim_x and 0 <= start[1] < dim_y):
        raise ValueError("Start coordinates must be within the grid.")
    if not (0 <= goal[0] < dim_x and 0 <= goal[1] < dim_y):
        raise ValueError("Goal coordinates must be within the grid.")

    grid = np.zeros((dim_x, dim_y), dtype=int)

    # Add obstacles randomly
    gridx, gridy = grid.shape
    np.random.seed(42)
    for _ in range(num_obstacles):
        x, y = np.random.randint(0, gridx), np.random.randint(0, gridy)
        grid[x, y] = 1

    # Ensure start and goal points are not blocked
    grid[start[0], start[1]] = 0
    grid[goal[0], goal[1]] = 0

    steps = []
    if algorithm == "theta_star":
        path = theta_star(grid, start, goal, steps)
    elif algorithm == "a_star":
        path = a_star(grid, start, goal, steps)
    elif algorithm == "hybrid_a_star":
        path = hybrid_a_star(grid, start, goal, steps)
    else:
        raise ValueError("Unknown algorithm specified in config.yaml")

    path_found = path is not None
    animate_pathfinding(grid, steps, start, goal, path_found)