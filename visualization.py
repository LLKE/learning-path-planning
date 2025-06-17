import numpy as np

def get_step_state(steps, step_index):
    """Reconstruct explored list and parent dict up to step_index"""
    explored = []
    parent = {}
    for i in range(step_index + 1):
        node, p = steps[i]
        explored.append(node)
        if p is not None:
            parent[node] = p 
    return explored, parent

def display_path_finding_step(grid, steps, step_index, start, goal, ax, is_last_step=False, path=None):
    """Animates the pathfinding process."""
    grid = np.array(grid)

    explored, parent = get_step_state(steps, step_index)

    # Remove theta if present
    if len(explored) > 0 and len(explored[0]) == 3:
        explored_2d = [(row, col) for row, col, _ in explored]
    else:
        explored_2d = explored

    for row, col in explored_2d:
        ax.plot(col, row, "ro", markersize=3)

    for node, p in parent.items():
        if p is not None:
            ax.plot([p[1], node[1]], [p[0], node[0]], color="orange", linewidth=1)

    # Draw final path
    if is_last_step and path:
        print(path)
        for i in range(len(path) - 1):
            ax.plot([path[i][1], path[i + 1][1]], [path[i][0], path[i + 1][0]], color="green", linewidth=2)
    elif is_last_step:
        ax.text(0.5, 0.5, "No path found", ha="center", va="center", transform=ax.transAxes,
                fontsize=20, color='red', bbox=dict(facecolor='white', alpha=0.9, edgecolor='red'))

    ax.legend()
    ax.set_aspect('equal')  # Make cells square
    ax.set_xlim(-0.5, grid.shape[1] - 0.5)
    ax.set_ylim(grid.shape[0] - 0.5, -0.5)  # Flip Y-axis to match image coords

