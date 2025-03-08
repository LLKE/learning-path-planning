import numpy as np
import yaml
import streamlit as st
import matplotlib.pyplot as plt
from graph_based.a_star import a_star
from graph_based.theta_star import theta_star
from graph_based.hybrid_a_star import hybrid_a_star
from animation import animate_pathfinding
import time

def main():
    st.title("Path Planning Visualization")

    # Load configuration from YAML file
    with open("config.yaml", "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    # Sidebar inputs
    algorithm = st.sidebar.selectbox("Select Algorithm", ["a_star", "theta_star", "hybrid_a_star"])
    dim_x = st.sidebar.number_input("Grid Size X", min_value=1, value=config["grid_size"]["x"])
    dim_y = st.sidebar.number_input("Grid Size Y", min_value=1, value=config["grid_size"]["y"])
    num_obstacles = st.sidebar.number_input("Number of Obstacles", min_value=0, value=config["obstacles"])
    start_x = st.sidebar.number_input("Start X", min_value=0, max_value=dim_x-1, value=config["start"][0])
    start_y = st.sidebar.number_input("Start Y", min_value=0, max_value=dim_y-1, value=config["start"][1])
    goal_x = st.sidebar.number_input("Goal X", min_value=0, max_value=dim_x-1, value=config["goal"][0])
    goal_y = st.sidebar.number_input("Goal Y", min_value=0, max_value=dim_y-1, value=config["goal"][1])

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
    if algorithm == "theta_star":
        path = theta_star(grid, start, goal, steps)
    elif algorithm == "a_star":
        path = a_star(grid, start, goal, steps)
    elif algorithm == "hybrid_a_star":
        path = hybrid_a_star(grid, start, goal, steps)
    else:
        st.error("Unknown algorithm specified.")
        return

    path_found = path is not None

    if st.sidebar.button("Start Animation"):
        placeholder = st.empty()
        for step in steps:
            fig, ax = plt.subplots()
            animate_pathfinding(grid, step, start, goal, ax)
            placeholder.pyplot(fig)
            plt.close(fig)
            time.sleep(0.2)  # Adjust the sleep time to control the animation speed

if __name__ == "__main__":
    main()