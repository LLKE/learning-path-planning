import unittest
import math
from graph_based.hybrid_a_star import is_on_path

class TestIsOnPath(unittest.TestCase):
    def test_goal_on_path(self):
        """Test when the goal lies on the line segment between current and neighbor."""
        current = (2, 2)
        neighbor = (4, 4)
        goal = (3, 3)  # Lies on the line segment
        self.assertTrue(is_on_path(goal, neighbor, current), "Goal should be on the path")

    def test_goal_not_on_path(self):
        """Test when the goal does not lie on the line segment between current and neighbor."""
        current = (2, 2)
        neighbor = (4, 4)
        goal = (3, 4)  # Does not lie on the line segment
        self.assertFalse(is_on_path(goal, neighbor, current), "Goal should not be on the path")

    def test_goal_outside_segment_bounds(self):
        """Test when the goal lies on the infinite line but outside the segment bounds."""
        current = (2, 2)
        neighbor = (4, 4)
        goal = (5, 5)  # Lies on the line but outside the segment bounds
        self.assertFalse(is_on_path(goal, neighbor, current), "Goal should not be on the path")

    def test_vertical_line(self):
        """Test when the line segment is vertical."""
        current = (3, 2)
        neighbor = (3, 4)
        goal = (3, 3)  # Lies on the vertical line segment
        self.assertTrue(is_on_path(goal, neighbor, current), "Goal should be on the vertical path")

    def test_horizontal_line(self):
        """Test when the line segment is horizontal."""
        current = (2, 3)
        neighbor = (4, 3)
        goal = (3, 3)  # Lies on the horizontal line segment
        self.assertTrue(is_on_path(goal, neighbor, current), "Goal should be on the horizontal path")

    def test_diagonal_line(self):
        """Test when the line segment is diagonal."""
        current = (1, 1)
        neighbor = (3, 3)
        goal = (2, 2)  # Lies on the diagonal line segment
        self.assertTrue(is_on_path(goal, neighbor, current), "Goal should be on the diagonal path")

    def test_floating_point_precision(self):
        """Test with floating-point precision issues."""
        current = (0.1, 0.1)
        neighbor = (0.3, 0.3)
        goal = (0.2, 0.2)  # Lies on the line segment
        self.assertTrue(is_on_path(goal, neighbor, current), "Goal should be on the path despite floating-point precision issues")

if __name__ == "__main__":
    unittest.main()