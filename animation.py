import numpy as np

def animate_pathfinding(grid, step, start, goal, ax, is_last_step=False, path=None):
    """Animates the pathfinding process with start and goal markers."""
    grid = np.array(grid)
    ax.imshow(grid, cmap="Greys", origin="upper")
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which="major", color="black", linestyle='-', linewidth=0.5)  
    ax.tick_params(which="both", bottom=False, left=False, labelbottom=False, labelleft=False)
    
    ax.plot(start[1], start[0], "go", markersize=6, label="Start")
    ax.plot(goal[1], goal[0], "bo", markersize=6, label="Goal")
    
    explored, parent = step
    for (x, y) in explored:
        ax.plot(y, x, "ro", markersize=3)
    for node, p in parent.items():
        if p is not None:
            ax.plot([p[1], node[1]], [p[0], node[0]], "orange", linewidth=1)
    
    if is_last_step and path:
        # Handle paths with orientation (Hybrid A*)
        if len(path[0]) == 3:  # Check if path includes orientation (x, y, theta)
            for i in range(len(path) - 1):
                ax.plot([path[i][1], path[i + 1][1]], [path[i][0], path[i + 1][0]], "green", linewidth=1)
        else:  # Handle paths without orientation
            for i in range(len(path) - 1):
                ax.plot([path[i][1], path[i + 1][1]], [path[i][0], path[i + 1][0]], "green", linewidth=1)
    elif is_last_step:
        ax.text(0.5, 0.5, "No path found", ha="center", va="center", transform=ax.transAxes, fontsize=20, color='red', bbox=dict(facecolor='white', alpha=0.9, edgecolor='red'))
    
    ax.legend()