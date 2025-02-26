import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from theta_a_star import theta_star, a_star

def animate_pathfinding(grid, steps, start, goal):
    """Animates the pathfinding process with start and goal markers."""
    fig, ax = plt.subplots()
    grid = np.array(grid)
    ax.imshow(grid, cmap="Greys", origin="upper")
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which="major", color="black", linestyle='-', linewidth=0.5)
    ax.tick_params(which="both", bottom=False, left=False, labelbottom=False, labelleft=False)
    
    def update(frame):
        ax.clear()
        ax.imshow(grid, cmap="Greys", origin="upper")
        ax.set_xticks(np.arange(0, grid.shape[1], 1))
        ax.set_yticks(np.arange(0, grid.shape[0], 1))
        ax.grid(which="major", color="black", linestyle='-', linewidth=0.5)
        ax.tick_params(which="both", bottom=False, left=False, labelbottom=False, labelleft=False)
        
        ax.plot(start[1], start[0], "go", markersize=6, label="Start")
        ax.plot(goal[1], goal[0], "bo", markersize=6, label="Goal")
        
        explored, parent = steps[frame]
        for (x, y) in explored:
            ax.plot(y, x, "ro", markersize=3)
        if frame == len(steps) - 1:
            # Plot the final path in green
            if goal in parent:
                node = goal
                while node != start:
                    p = parent[node]
                    ax.plot([p[1], node[1]], [p[0], node[0]], "green", linewidth=1)
                    node = p
        else:
            # Plot the explored nodes in orange
            for node, p in parent.items():
                if p is not None:
                    ax.plot([p[1], node[1]], [p[0], node[0]], "orange", linewidth=1)
        
        ax.legend()
    
    ani = animation.FuncAnimation(fig, update, frames=len(steps), repeat=False, interval=200)
    plt.show()

if __name__ == "__main__":
    dim_x = 13
    dim_y = 13
    grid = np.zeros((dim_x, dim_y), dtype=int)

    # Add obstacles randomly
    gridx, gridy = grid.shape
    np.random.seed(42)
    for _ in range(50):
        x, y = np.random.randint(0, gridx), np.random.randint(0, gridy)
        grid[x, y] = 1

    start = (0, 0)
    goal = (dim_x - 1, dim_y - 1)

    steps_theta = []
    path_theta = theta_star(grid, start, goal, steps_theta)
    
    animate_pathfinding(grid, steps_theta, start, goal)

    steps_a_star = []
    path_a_star = a_star(grid, start, goal, steps_a_star)
    animate_pathfinding(grid, steps_a_star, start, goal)