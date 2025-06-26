import time
import numpy as np
import streamlit as st
import matplotlib.pyplot as plt
import tracemalloc
from graph_based.a_star import a_star
from graph_based.theta_star import theta_star
from graph_based.hybrid_a_star import hybrid_a_star
from visualization import animate_pathfinding, animate_pathfinding_with_orientation, prepare_ax_for_animation
from algorithm_descriptions import descriptions

def main():

    st.title("Path Planning")

    # Sidebar: Grid-specific settings
    st.sidebar.header("Grid Settings")
    dim_x = st.sidebar.number_input("Grid Size X", min_value=1, max_value=25, value=10)
    dim_y = st.sidebar.number_input("Grid Size Y", min_value=1, max_value=25, value=10)
    num_obstacles = st.sidebar.number_input("Number of Obstacles", min_value=0, value=20)
    start_x = st.sidebar.number_input("Start X", min_value=0, max_value=dim_x-1, value=0)
    start_y = st.sidebar.number_input("Start Y", min_value=0, max_value=dim_y-1, value=0)
    goal_x = st.sidebar.number_input("Goal X", min_value=0, max_value=dim_x-1, value=dim_x-1)
    goal_y = st.sidebar.number_input("Goal Y", min_value=0, max_value=dim_y-1, value=dim_y-1)

    # Sidebar: Animation-specific settings
    st.sidebar.header("Animation Settings")
    animation_speed = st.sidebar.slider("Animation Speed (seconds per frame)", min_value=0.1, max_value=1.0, value=0.2, step=0.1)

    # Sidebar: Vehicle-specific settings
    st.sidebar.header("Vehicle Settings")
    algorithm = st.sidebar.selectbox("Select Algorithm", ["A*", "Theta*", "Hybrid A*"])
    turning_radius = None
    if algorithm == "Hybrid A*":
        turning_radius = st.sidebar.slider("Max Turning Radius (meters)", min_value=1.0, max_value=4.0, value=1.0, step=0.5)
        if not st.sidebar.button("Confirm Turning Radius"):
            st.warning("Please confirm the turning radius before starting the animation.")
            return

    start = (start_x, start_y)
    goal = (goal_x, goal_y)

    # Check if the map size values are feasible
    if dim_x <= 0 or dim_y <= 0:
        st.error("Grid size must be greater than 0.")
        return
    if num_obstacles >= dim_x * dim_y:
        st.error("Number of obstacles must be less than the total number of grid cells.")
        return
    if not (0 <= start[0] < dim_x and 0 <= start[1] < dim_y):
        st.error("Start coordinates must be within the grid.")
        return
    if not (0 <= goal[0] < dim_x and 0 <= goal[1] < dim_y):
        st.error("Goal coordinates must be within the grid.")
        return

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
    animate_with_orientation = False
    if algorithm == "Theta*":
        path = theta_star(grid, start, goal, steps)
    elif algorithm == "A*":
        path = a_star(grid, start, goal, steps)
    elif algorithm == "Hybrid A*":
        start = (start[0], start[1], 0)  # Append orientation to start
        goal = (goal[0], goal[1], 0)    # Append orientation to goal
        path = hybrid_a_star(grid, start, goal, turning_radius, steps)
        animate_with_orientation = True
    else:
        st.error("Unknown algorithm specified.")
        return

    fig, ax = plt.subplots()
    placeholder = st.empty()
    step_counter = st.empty()
    with st.expander("Algorithm Description"):
        st.markdown(descriptions[algorithm])
    ax = prepare_ax_for_animation(ax, grid, start, goal)
    if animate_with_orientation:
        for fig, i in animate_pathfinding_with_orientation(fig, ax, steps, path, animation_speed=animation_speed):
            placeholder.pyplot(fig)
            step_counter.markdown(f"**Step {i+1}*")
    else: 
        for fig, i in animate_pathfinding(fig, ax, steps, path, animation_speed=animation_speed):
            placeholder.pyplot(fig)
            step_counter.markdown(f"**Step {i+1}*")
    
    st.stop()

    
if __name__ == "__main__":
    main()