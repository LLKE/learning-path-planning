import unittest
import numpy as np
from graph_based.hybrid_a_star import hybrid_a_star, get_neighbors
import pdb

class TestHybridAStar(unittest.TestCase):
    def setUp(self):
        """Set up common test variables."""
        self.grid_empty = np.zeros((10, 10), dtype=int)  # 10x10 grid with no obstacles
        self.grid_with_obstacles = np.zeros((10, 10), dtype=int)
        self.grid_with_obstacles[4, 4] = 1  # Add an obstacle at (4, 4)
        self.turning_radius = 1.0
        self.steps = []

    def test_get_neighbors_no_obstacles(self):
        """Test the get_neighbors function with no obstacles."""
        node = (5, 5, 0)  # Starting node at the center of the grid with orientation 0
        max_turning_angle = 45  # Maximum turning angle in degrees
        v = 1.0  # Constant forward velocity
        delta_t = 1.0  # Time step

        # Call the get_neighbors function
        neighbors = get_neighbors(node, max_turning_angle, v, delta_t)

        # Check that neighbors are generated
        self.assertGreater(len(neighbors), 0, "No neighbors were generated")

        # Check that all neighbors are within the grid bounds
        for neighbor in neighbors:
            x, y, _ = neighbor
            self.assertGreaterEqual(x, 0, "Neighbor x-coordinate is out of bounds")
            self.assertGreaterEqual(y, 0, "Neighbor y-coordinate is out of bounds")
            self.assertLess(x, self.grid_empty.shape[0], "Neighbor x-coordinate is out of bounds")
            self.assertLess(y, self.grid_empty.shape[1], "Neighbor y-coordinate is out of bounds")

        # Check that the neighbors are not the same as the current node
        self.assertNotIn(node, neighbors, "Current node is included as a neighbor")
        

    def test_simple_path(self):
        """Test a simple path with no obstacles."""
        start = (0, 0, 0)
        goal = (9, 9, 0)
        path = hybrid_a_star(self.grid_empty, start, goal, self.turning_radius, self.steps)
        self.assertIsNotNone(path, "Path should not be None")
        self.assertEqual(path[0], start, "Path should start at the start node")
        self.assertEqual(path[-1][:2], goal[:2], "Path should end at the goal node")

    def test_path_with_obstacles(self):
        """Test a path with obstacles."""
        start = (0, 0, 0)
        goal = (9, 9, 0)
        path = hybrid_a_star(self.grid_with_obstacles, start, goal, self.turning_radius, self.steps)
        self.assertIsNotNone(path, "Path should not be None")
        self.assertNotIn((4, 4, 0), path, "Path should not pass through obstacles")

    def test_unreachable_goal(self):
        """Test a case where the goal is unreachable."""
        grid_blocked = np.ones((10, 10), dtype=int)  # Fully blocked grid
        start = (0, 0, 0)
        goal = (9, 9, 0)
        path = hybrid_a_star(grid_blocked, start, goal, self.turning_radius, self.steps)
        self.assertIsNone(path, "Path should be None when the goal is unreachable")

    def test_same_start_and_goal(self):
        """Test a case where the start and goal are the same."""
        start = (5, 5, 0)
        goal = (5, 5, 0)
        path = hybrid_a_star(self.grid_empty, start, goal, self.turning_radius, self.steps)
        self.assertEqual(path, [start], "Path should contain only the start/goal node")

    def test_turning_radius(self):
        """Test that the turning radius is respected."""
        start = (0, 0, 0)
        goal = (9, 9, 0)
        path = hybrid_a_star(self.grid_empty, start, goal, self.turning_radius, self.steps)
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            distance = np.sqrt(dx**2 + dy**2)
            self.assertGreaterEqual(distance, self.turning_radius / 2, "Turning radius constraint violated")

if __name__ == "__main__":
    unittest.main()