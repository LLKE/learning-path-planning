import numpy as np
import time
import math
import matplotlib.patches as patches
import matplotlib.transforms as transforms

def prepare_ax_for_animation(ax, grid, start, goal):
    ax.set_xticks(np.arange(0, grid.shape[1], 1))  # X = columns
    ax.set_yticks(np.arange(0, grid.shape[0], 1))  # Y = rows
    ax.grid(which="major", color="black", linestyle='-', linewidth=0.5)
    ax.tick_params(which="both", bottom=False, left=False, labelbottom=False, labelleft=False)

    # Plot start and goal (convert from row,col to x,y)
    ax.plot(start[1], start[0], "go", markersize=6, label="Start")
    ax.plot(goal[1], goal[0], "bo", markersize=6, label="Goal")

    ax.legend()
    ax.set_aspect('equal')  # Make cells square
    ax.set_xlim(-0.5, grid.shape[1] - 0.5)
    ax.set_ylim(grid.shape[0] - 0.5, -0.5)  # Flip Y-axis to match image coords

    # Plot obstacles
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if grid[row, col] == 1:
                ax.plot(col, row, "ko", markersize=6)
    
    return ax

def animate_pathfinding(fig, ax, steps, grid, path=None, animation_speed=0.1):

    explored = []
    parent = {}
    for i, (current, parent) in enumerate(steps):
        is_last_step = (i == len(steps) - 1)
        current_x, current_y = current[0], current[1]

        ax.plot(current_y, current_x, 'ro', markersize=3)

        current, parent = steps[i]
        explored.append(current)
        if parent is not None:
            parent_x, parent_y = parent[0], parent[1]
            ax.plot([parent_y, current_y], [parent_x, current_x], color='orange', linewidth=1)
        
        if is_last_step:
            if path is not None:
                for a, b in zip(path[:-1], path[1:]):
                    ax.plot([a[1], b[1]], [a[0], b[0]], color='green', linewidth=2)
            else:
                ax.text((grid.shape[1] - 1)/2, (grid.shape[0] - 1)/2, "No path found",
                    ha="center", va="center",
                    fontsize=14, color="red",
                    bbox=dict(
                    boxstyle="round,pad=0.3",
                    facecolor="white",
                    edgecolor="black",
                    alpha=0.5
                ))

        yield  fig, i
        time.sleep(animation_speed)  # Use the selected animation speed

def animate_pathfinding_with_orientation(fig, ax, steps, grid, path=None, animation_speed=0.1):

    explored = []
    parent = {}
    vehicle_width, vehicle_height = .2, .3
    prev_theta = 0
    for i, (current, parent) in enumerate(steps):
        is_last_step = (i == len(steps) - 1)
        current_x, current_y, current_theta = current[0], current[1], current[2]

        rect = patches.Rectangle((current_y - vehicle_width/2, current_x - vehicle_height/2), width=.2, height=.3, linewidth=1, edgecolor='r', facecolor='r')
        t = transforms.Affine2D().rotate_deg_around(current_y, current_x, math.degrees(-current_theta)) + ax.transData
        rect.set_transform(t)
        ax.add_patch(rect)
        current, parent = steps[i]
        explored.append(current)
        if parent is not None:
            parent_x, parent_y = parent[0], parent[1]
            ax.plot([parent_y, current_y], [parent_x, current_x], color='orange', linewidth=1)
        
        if is_last_step:
            if path is not None:
                for a, b in zip(path[:-1], path[1:]):
                    ax.plot([a[1], b[1]], [a[0], b[0]], color='green', linewidth=2)
            else:
                ax.text((grid.shape[1] - 1)/2, (grid.shape[0] - 1)/2, "No path found",
                    ha="center", va="center",
                    fontsize=14, color="red",
                    bbox=dict(
                    boxstyle="round,pad=0.3",
                    facecolor="white",
                    edgecolor="black",
                    alpha=0.5
                ))
        yield  fig, i
        time.sleep(animation_speed)  # Use the selected animation speed



