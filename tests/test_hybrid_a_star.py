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
        self.turning_angle = 15
        self.steps = []

    def test_simple_path(self):
        """Test a simple path with no obstacles."""
        start = (0, 0, 0)
        goal = (9, 9, 0)
        path = hybrid_a_star(self.grid_empty, start, goal, self.turning_angle, self.steps)
        self.assertIsNotNone(path, "Path should not be None")
        self.assertEqual(path[0], start, "Path should start at the start node")
        self.assertEqual(path[-1][:2], goal[:2], "Path should end at the goal node")

    def test_path_with_obstacles(self):
        """Test a path with obstacles."""
        start = (0, 0, 0)
        goal = (9, 9, 0)
        path = hybrid_a_star(self.grid_with_obstacles, start, goal, self.turning_angle, self.steps)
        self.assertIsNotNone(path, "Path should not be None")
        self.assertNotIn((4, 4, 0), path, "Path should not pass through obstacles")

    def test_unreachable_goal(self):
        """Test a case where the goal is unreachable."""
        grid_blocked = np.ones((10, 10), dtype=int)  # Fully blocked grid
        start = (0, 0, 0)
        goal = (9, 9, 0)
        path = hybrid_a_star(grid_blocked, start, goal, self.turning_angle, self.steps)
        self.assertIsNone(path, "Path should be None when the goal is unreachable")

    def test_same_start_and_goal(self):
        """Test a case where the start and goal are the same."""
        start = (5, 5, 0)
        goal = (5, 5, 0)
        path = hybrid_a_star(self.grid_empty, start, goal, self.turning_angle, self.steps)
        self.assertEqual(path, [start], "Path should contain only the start/goal node")

    def test_max_turning_angle(self):
        """Test that the maximum turning angle is respected."""
        start = (0, 0, 0)  # Start position with orientation
        goal = (9, 9, 0)  # Goal position with orientation
        max_turning_angle = 45  # Maximum turning angle in degrees
        path = hybrid_a_star(self.grid_empty, start, goal, max_turning_angle, self.steps)
        
        for i in range(1, len(path)):
            theta_prev = path[i - 1][2]  # Orientation of the previous node
            theta_curr = path[i][2]  # Orientation of the current node
            delta_theta = abs(theta_curr - theta_prev)  # Change in orientation
            
            # Normalize delta_theta to the range [0, π]
            delta_theta = min(delta_theta, 2 * np.pi - delta_theta)
            
            # Convert max_turning_angle to radians
            max_turning_angle_rad = np.radians(max_turning_angle)
            
            # Assert that the change in orientation does not exceed the maximum turning angle
        self.assertLessEqual(delta_theta, max_turning_angle_rad, "Maximum turning angle constraint violated")

if __name__ == "__main__":
    unittest.main()