import unittest
import numpy as np
import math
from graph_based.hybrid_a_star import hybrid_a_star, get_neighbors
import pdb

class TestHybridAStar(unittest.TestCase):
    def setUp(self):
        """Set up common test variables."""
        self.grid_empty = np.zeros((10, 10), dtype=int)  # 10x10 grid with no obstacles
        self.grid_with_obstacles = np.zeros((10, 10), dtype=int)
        self.grid_with_obstacles[4, 4] = 1  # Add an obstacle at (4, 4)
        self.turning_radius = 4
        self.steps = []

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
        """Test that the minimum turning radius is respected."""
        start = (0, 0, 0)  # Start position with orientation
        goal = (9, 9, 0)  # Goal position with orientation
        turning_radius = 1.0  # Minimum turning radius (corresponds to 45 degrees max turning angle if wheelbase=1)
        path = hybrid_a_star(self.grid_empty, start, goal, turning_radius, self.steps)
        
        # The maximum allowed steering angle per step
        max_steering_angle = np.arctan2(1, turning_radius)
        
        for i in range(1, len(path)):
            theta_prev = path[i - 1][2]
            theta_curr = path[i][2]
            delta_theta = abs(theta_curr - theta_prev)
            delta_theta = min(delta_theta, 2 * np.pi - delta_theta)

        # The snapping to the grid at a relatively rough quantization rate can cause this to be exceeded
        self.assertLessEqual(delta_theta, max_steering_angle + (math.pi/8) + 1e-6, "Turning radius constraint violated")

if __name__ == "__main__":
    unittest.main()