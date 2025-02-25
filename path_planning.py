import matplotlib.pyplot as plt
import numpy as np
from theta_a_star import theta_star, a_star

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