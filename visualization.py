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

def display_current_search_state(grid, steps, step_index, start, goal, ax, is_last_step=False, path=None, drawn_nodes=None):
    """Animates the pathfinding process."""
    if drawn_nodes is None:
        drawn_nodes = set()

    # Plot obstacles
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if grid[row, col] == 1:
                ax.plot(col, row, "ko", markersize=6)

    explored, parent = get_step_state(steps, step_index)

    # Remove theta if present
    if len(explored) > 0 and len(explored[0]) == 3:
        explored_2d = [(row, col) for row, col, _ in explored]
    else:
        explored_2d = explored

    for node in explored_2d:
        if node not in drawn_nodes:
            ax.plot(node[1], node[0], "ro", markersize=3)
            drawn_nodes.add(node)

    for node, p in parent.items():
        if p is not None:
            ax.plot([p[1], node[1]], [p[0], node[0]], color="orange", linewidth=1)

    # Draw final path
    if is_last_step and path:
        for i in range(len(path) - 1):
            ax.plot([path[i][1], path[i + 1][1]], [path[i][0], path[i + 1][0]], color="green", linewidth=2)
    elif is_last_step:
        ax.text(0.5, 0.5, "No path found", ha="center", va="center", transform=ax.transAxes,
                fontsize=20, color='red', bbox=dict(facecolor='white', alpha=0.9, edgecolor='red'))

    ax.legend()
    ax.set_aspect('equal')  # Make cells square
    ax.set_xlim(-0.5, grid.shape[1] - 0.5)
    ax.set_ylim(grid.shape[0] - 0.5, -0.5)  # Flip Y-axis to match image coords

    return drawn_nodes

