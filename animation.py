import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import time

def animate_pathfinding(grid, steps, start, goal):
    """Animates the pathfinding process with start and goal markers."""
    fig, ax = plt.subplots()
    grid = np.array(grid)
    ax.imshow(grid, cmap="Greys", origin="upper")
    ax.set_xticks(np.arange(0, grid.shape[1], 1))
    ax.set_yticks(np.arange(0, grid.shape[0], 1))
    ax.grid(which="major", color="black", linestyle='-', linewidth=0.5)  
    ax.tick_params(which="both", bottom=False, left=False, labelbottom=False, labelleft=False)
    
    start_time = time.time()
    clock_text = ax.text(0.5, -0.1, '', transform=ax.transAxes, ha='center', fontsize=12, color='blue', bbox=dict(facecolor='white', alpha=0.8, edgecolor='blue'))

    def update(frame):
        ax.clear()
        is_last_step = (frame == len(steps) - 1)
        animate_pathfinding(grid, steps[frame], start, goal, ax, is_last_step)
        
        elapsed_time = time.time() - start_time
        clock_text.set_text(f'Time elapsed: {elapsed_time:.2f} seconds')
        ax.add_artist(clock_text)  # Ensure the clock text is added to the plot
    
    anim = animation.FuncAnimation(fig, update, frames=len(steps), repeat=False, interval=200)
    return anim

def animate_pathfinding(grid, step, start, goal, ax, is_last_step=False):
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
    if goal in parent:
        node = goal
        while node != start:
            p = parent[node]
            ax.plot([p[1], node[1]], [p[0], node[0]], "green", linewidth=1)
            node = p
    elif is_last_step:
        ax.text(0.5, 0.5, "No path found", ha="center", va="center", transform=ax.transAxes, fontsize=20, color='red', bbox=dict(facecolor='white', alpha=0.9, edgecolor='red'))
    
    ax.legend()