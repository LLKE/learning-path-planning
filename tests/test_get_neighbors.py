import unittest
import numpy as np
from graph_based.hybrid_a_star import get_neighbors

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
        self.assertEqual(len(neighbors), 3, "No neighbors were generated")

        # Check that all neighbors are within the grid bounds
        for neighbor in neighbors:
            x, y, _ = neighbor
            self.assertGreaterEqual(x, 0, "Neighbor x-coordinate is out of bounds")
            self.assertGreaterEqual(y, 0, "Neighbor y-coordinate is out of bounds")
            self.assertLess(x, self.grid_empty.shape[0], "Neighbor x-coordinate is out of bounds")
            self.assertLess(y, self.grid_empty.shape[1], "Neighbor y-coordinate is out of bounds")

        # Check that the neighbors are not the same as the current node
        self.assertNotIn(node, neighbors, "Current node is included as a neighbor")

if __name__ == "__main__":
    unittest.main()