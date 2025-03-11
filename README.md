# Path Planning Visualization

This repository contains implementations of various pathfinding algorithms, so far including A*, Theta*, and Hybrid A*. These algorithms are used to find the shortest path between a start and a goal point, considering obstacles. The repository also includes an animation module to visualize the pathfinding process.

I created this repo to learn about path planning, maybe you can also learn a little something from it 😊

If there are any algorithms you would like to see me implement, let me know! I have a roadmap of what algorithms I will do next, but I don't have the best overview of the algorithms out there, so I might miss some important ones!

The tool uses Streamlit to create an interactive web application that allows users to visualize the execution of different path planning algorithms on a grid with obstacles.

## Features

- **Interactive Inputs**: Users can configure the grid size, number of obstacles, start and goal positions directly in the Streamlit app.
- **Visualization**: Step-by-step visualization of the pathfinding process, including explored nodes and the final path.

## Algorithms:
- **Grid Based** 
  - A*
  - Theta*
  - Hybrid A*

## Requirements

- Python 3.7 or higher
- Streamlit
- NumPy
- Matplotlib

## Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/path-planning-visualization.git
    cd path-planning-visualization
    ```

2. Install the required packages:
    ```sh
    pip install -r requirements.txt
    ```

## Usage

1. Run the Streamlit app:
    ```sh
    streamlit run path_planning.py
    ```

2. Open your web browser and navigate to the URL provided by Streamlit (usually `http://localhost:8501`).

3. Configure the grid size, number of obstacles, start and goal positions, and select the path planning algorithm from the sidebar.

4. Click the "Start Animation" button to visualize the pathfinding process.

## Files

- `path_planning.py`: The main script that sets up the Streamlit app and handles user inputs.
- `animation.py`: Contains the function to animate the pathfinding process.
- `graph_based/a_star.py`: Implementation of the A* algorithm.
- `graph_based/theta_star.py`: Implementation of the Theta* algorithm.
- `graph_based/hybrid_a_star.py`: Implementation of the Hybrid A* algorithm.

## Contributing

Contributions are welcome! If you have any suggestions, bug reports, or feature requests, please open an issue or submit a pull request. Thanks!

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.