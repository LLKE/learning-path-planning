import time
import numpy as np
import streamlit as st
import matplotlib.pyplot as plt
from graph_based.a_star import a_star
from graph_based.theta_star import theta_star
from graph_based.hybrid_a_star import hybrid_a_star
from animation import animate_pathfinding
from algorithm_descriptions import descriptions

def main():
    st.title("Path Planning")

    # Sidebar inputs
    algorithm = st.sidebar.selectbox("Select Algorithm", ["A*", "Theta*", "Hybrid A*"])
    dim_x = st.sidebar.number_input("Grid Size X", min_value=1, max_value=25, value=10)
    dim_y = st.sidebar.number_input("Grid Size Y", min_value=1, max_value=25, value=10)
    num_obstacles = st.sidebar.number_input("Number of Obstacles", min_value=0, value=20)
    start_x = st.sidebar.number_input("Start X", min_value=0, max_value=dim_x-1, value=0)
    start_y = st.sidebar.number_input("Start Y", min_value=0, max_value=dim_y-1, value=0)
    goal_x = st.sidebar.number_input("Goal X", min_value=0, max_value=dim_x-1, value=dim_x-1)
    goal_y = st.sidebar.number_input("Goal Y", min_value=0, max_value=dim_y-1, value=dim_y-1)
    animation_speed = st.sidebar.slider("Animation Speed (seconds per frame)", min_value=0.1, max_value=1.0, value=0.2, step=0.1)

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
    if algorithm == "Theta*":
        path = theta_star(grid, start, goal, steps)
    elif algorithm == "A*":
        path = a_star(grid, start, goal, steps)
    elif algorithm == "Hybrid A*":
        path = hybrid_a_star(grid, start, goal, steps)
    else:
        st.error("Unknown algorithm specified.")
        return

    if st.sidebar.button("Start Animation"):
        placeholder = st.empty()
        step_counter = st.empty()
        with st.expander("Algorithm Description"):
            st.markdown(descriptions[algorithm])
        for i, step in enumerate(steps):
            fig, ax = plt.subplots()
            is_last_step = (i == len(steps) - 1)
            animate_pathfinding(grid, step, start, goal, ax, is_last_step, path)
            placeholder.pyplot(fig)
            step_counter.markdown(f"**Step: {i + 1}**")
            plt.close(fig)
            time.sleep(animation_speed)  # Use the selected animation speed

if __name__ == "__main__":
    main()