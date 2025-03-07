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

    grid = np.zeros((dim_x, dim_y), dtype=int)

    # Add obstacles randomly
    gridx, gridy = grid.shape
    np.random.seed(42)
    for _ in range(num_obstacles):
        x, y = np.random.randint(0, gridx), np.random.randint(0, gridy)
        grid[x, y] = 1

    start = (0, 0)
    goal = (dim_x - 1, dim_y - 1)

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