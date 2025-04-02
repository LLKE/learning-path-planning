import numpy as np

class DubinsPath:
    def __init__(self, start, end, turning_radius):
        self.start = np.array(start)
        self.end = np.array(end)
        self.turning_radius = turning_radius

    def distance(self, point_1, point_2):
        return np.linalg.norm(np.array(point_2) - np.array(point_1))
    
    def rotate_point(self, point, angle):
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle),   np.cos(angle)]
                                    ])
        return np.dot(rotation_matrix, point)
    
    def lsl_path(self): 
        x0, y0, theta0 = self.start
        x1, y1, theta1 = self.end
        
        # Compute the circle centers
        x_center0 = x0 - self.turning_radius * np.sin(theta0)
        y_center0 = y0 + self.turning_radius * np.cos(theta0)
        x_center1 = x1 - self.turning_radius * np.sin(theta1)
        y_center1 = y1 + self.turning_radius * np.cos(theta1)
        
        # Compute the distance between the circle centers
        d = self.distance([x_center0, y_center0], [x_center1, y_center1])
        
        # Compute the tangent angle
        alpha = np.arcsin((2 * self.turning_radius) / d)
        
        # Compute the angle of the line connecting the two circle centers
        theta_line = np.arctan2(y_center1 - y_center0, x_center1 - x_center0)
        
        # Compute the tangent points
        tangent_angle1 = theta_line + alpha
        tangent_angle2 = theta_line - alpha
        
        # Compute tangent points on each circle
        x_tangent0 = x_center0 + self.turning_radius * np.cos(tangent_angle1)
        y_tangent0 = y_center0 + self.turning_radius * np.sin(tangent_angle1)
        x_tangent1 = x_center1 + self.turning_radius * np.cos(tangent_angle2)
        y_tangent1 = y_center1 + self.turning_radius * np.sin(tangent_angle2)
        
        # Create the path
        # Arc from start to tangent point 1
        arc1_points = self._generate_arc_points(x_center0, y_center0, tangent_angle1, theta0)
        
        # Straight segment from tangent point 1 to tangent point 2
        straight_segment = np.array([ [x_tangent0, y_tangent0], [x_tangent1, y_tangent1] ])
        
        # Arc from tangent point 2 to goal
        arc2_points = self._generate_arc_points(x_center1, y_center1, theta1, tangent_angle2)
        
        # Combine all segments
        path = np.vstack([arc1_points, straight_segment, arc2_points])
        
        # Return path and its length
        total_length = self._calculate_path_length(arc1_points, straight_segment, arc2_points)
        return {"segments": path, "length": total_length}
    

    def lsr_path(self):
        x0, y0, theta0 = self.start
        x1, y1, theta1 = self.end
        
        # Step 1: Compute the circle centers for left turn (start) and right turn (goal)
        x_center0 = x0 - self.turning_radius * np.sin(theta0)
        y_center0 = y0 + self.turning_radius * np.cos(theta0)
        x_center1 = x1 + self.turning_radius * np.sin(theta1)
        y_center1 = y1 - self.turning_radius * np.cos(theta1)
        
        # Step 2: Compute the distance between the circle centers
        d = self.distance([x_center0, y_center0], [x_center1, y_center1])
        
        # Step 3: Check if a solution exists (the distance must be greater than 2 * radius)
        if d < 2 * self.turning_radius:
            return None  # No valid path if the circles are too close
        
        # Step 4: Compute the tangent angle
        alpha = np.arcsin((2 * self.turning_radius) / d)
        
        # Step 5: Compute the angle of the line connecting the two circle centers
        theta_line = np.arctan2(y_center1 - y_center0, x_center1 - x_center0)
        
        # Step 6: Compute the tangent points
        tangent_angle1 = theta_line + alpha
        tangent_angle2 = theta_line - alpha
        
        # Compute tangent points on each circle
        x_tangent0 = x_center0 + self.turning_radius * np.cos(tangent_angle1)
        y_tangent0 = y_center0 + self.turning_radius * np.sin(tangent_angle1)
        x_tangent1 = x_center1 + self.turning_radius * np.cos(tangent_angle2)
        y_tangent1 = y_center1 + self.turning_radius * np.sin(tangent_angle2)
        
        # Step 7: Create the path
        # Arc from start to tangent point 1 (left turn)
        arc1_points = self._generate_arc_points(x_center0, y_center0, tangent_angle1, theta0)
        
        # Straight segment from tangent point 1 to tangent point 2
        straight_segment = np.array([ [x_tangent0, y_tangent0], [x_tangent1, y_tangent1] ])
        
        # Arc from tangent point 2 to goal (right turn)
        arc2_points = self._generate_arc_points(x_center1, y_center1, theta1, tangent_angle2)
        
        # Combine all segments
        path = np.vstack([arc1_points, straight_segment, arc2_points])
        
        # Return path and its length
        total_length = self._calculate_path_length(arc1_points, straight_segment, arc2_points)
        return {"segments": path, "length": total_length}


    def rsl_path(self):
        x0, y0, theta0 = self.start
        x1, y1, theta1 = self.end
        
        # Step 1: Compute the circle centers for right turn (start) and left turn (goal)
        x_center0 = x0 + self.turning_radius * np.sin(theta0)
        y_center0 = y0 - self.turning_radius * np.cos(theta0)
        x_center1 = x1 - self.turning_radius * np.sin(theta1)
        y_center1 = y1 + self.turning_radius * np.cos(theta1)
        
        # Step 2: Compute the distance between the circle centers
        d = self.distance([x_center0, y_center0], [x_center1, y_center1])
        
        # Step 3: Check if a solution exists (the distance must be greater than 2 * radius)
        if d < 2 * self.turning_radius:
            return None  # No valid path if the circles are too close
        
        # Step 4: Compute the tangent angle
        alpha = np.arcsin((2 * self.turning_radius) / d)
        
        # Step 5: Compute the angle of the line connecting the two circle centers
        theta_line = np.arctan2(y_center1 - y_center0, x_center1 - x_center0)
        
        # Step 6: Compute the tangent points
        tangent_angle1 = theta_line + alpha
        tangent_angle2 = theta_line - alpha
        
        # Compute tangent points on each circle
        x_tangent0 = x_center0 + self.turning_radius * np.cos(tangent_angle1)
        y_tangent0 = y_center0 + self.turning_radius * np.sin(tangent_angle1)
        x_tangent1 = x_center1 + self.turning_radius * np.cos(tangent_angle2)
        y_tangent1 = y_center1 + self.turning_radius * np.sin(tangent_angle2)
        
        # Step 7: Create the path
        # Arc from start to tangent point 1 (right turn)
        arc1_points = self._generate_arc_points(x_center0, y_center0, tangent_angle1, theta0)
        
        # Straight segment from tangent point 1 to tangent point 2
        straight_segment = np.array([ [x_tangent0, y_tangent0], [x_tangent1, y_tangent1] ])
        
        # Arc from tangent point 2 to goal (left turn)
        arc2_points = self._generate_arc_points(x_center1, y_center1, theta1, tangent_angle2)
        
        # Combine all segments
        path = np.vstack([arc1_points, straight_segment, arc2_points])
        
        # Return path and its length
        total_length = self._calculate_path_length(arc1_points, straight_segment, arc2_points)
        return {"segments": path, "length": total_length}


    def rsr_path(self):
        x0, y0, theta0 = self.start
        x1, y1, theta1 = self.end
        
        # Step 1: Compute the circle centers for right turn (start) and right turn (goal)
        x_center0 = x0 + self.turning_radius * np.sin(theta0)
        y_center0 = y0 - self.turning_radius * np.cos(theta0)
        x_center1 = x1 + self.turning_radius * np.sin(theta1)
        y_center1 = y1 - self.turning_radius * np.cos(theta1)
        
        # Step 2: Compute the distance between the circle centers
        d = self.distance([x_center0, y_center0], [x_center1, y_center1])
        
        # Step 3: Check if a solution exists (the distance must be greater than 2 * radius)
        if d < 2 * self.turning_radius:
            return None  # No valid path if the circles are too close
        
        # Step 4: Compute the tangent angle
        alpha = np.arcsin((2 * self.turning_radius) / d)
        
        # Step 5: Compute the angle of the line connecting the two circle centers
        theta_line = np.arctan2(y_center1 - y_center0, x_center1 - x_center0)
        
        # Step 6: Compute the tangent points
        tangent_angle1 = theta_line + alpha
        tangent_angle2 = theta_line - alpha
        
        # Compute tangent points on each circle
        x_tangent0 = x_center0 + self.turning_radius * np.cos(tangent_angle1)
        y_tangent0 = y_center0 + self.turning_radius * np.sin(tangent_angle1)
        x_tangent1 = x_center1 + self.turning_radius * np.cos(tangent_angle2)
        y_tangent1 = y_center1 + self.turning_radius * np.sin(tangent_angle2)
        
        # Step 7: Create the path
        # Arc from start to tangent point 1 (right turn)
        arc1_points = self._generate_arc_points(x_center0, y_center0, tangent_angle1, theta0)
        
        # Straight segment from tangent point 1 to tangent point 2
        straight_segment = np.array([ [x_tangent0, y_tangent0], [x_tangent1, y_tangent1] ])
        
        # Arc from tangent point 2 to goal (right turn)
        arc2_points = self._generate_arc_points(x_center1, y_center1, theta1, tangent_angle2)
        
        # Combine all segments
        path = np.vstack([arc1_points, straight_segment, arc2_points])
        
        # Return path and its length
        total_length = self._calculate_path_length(arc1_points, straight_segment, arc2_points)
        return {"segments": path, "length": total_length}


    def rlr_path(self):
        x0, y0, theta0 = self.start
        x1, y1, theta1 = self.end

        # Step 1: Compute the circle centers for right turn (start) and right turn (goal)
        x_center0 = x0 - self.turning_radius * np.sin(theta0)
        y_center0 = y0 + self.turning_radius * np.cos(theta0)
        x_center1 = x1 - self.turning_radius * np.sin(theta1)
        y_center1 = y1 + self.turning_radius * np.cos(theta1)

        # Step 2: Compute the distance between the circle centers
        d = self.distance([x_center0, y_center0], [x_center1, y_center1])
        
        # Step 3: Check if a solution exists (the distance must be greater than 2 * radius)
        if d < 2 * self.turning_radius:
            return None  # No valid path if the circles are too close
        
        # Step 4: Compute the tangent angle
        alpha = np.arcsin((2 * self.turning_radius) / d)
        
        # Step 5: Compute the angle of the line connecting the two circle centers
        theta_line = np.arctan2(y_center1 - y_center0, x_center1 - x_center0)
        
        # Step 6: Compute the tangent points
        tangent_angle1 = theta_line + alpha
        tangent_angle2 = theta_line - alpha
        
        # Compute tangent points on each circle
        x_tangent0 = x_center0 + self.turning_radius * np.cos(tangent_angle1)
        y_tangent0 = y_center0 + self.turning_radius * np.sin(tangent_angle1)
        x_tangent1 = x_center1 + self.turning_radius * np.cos(tangent_angle2)
        y_tangent1 = y_center1 + self.turning_radius * np.sin(tangent_angle2)
        
        # Step 7: Create the path
        # Arc from start to tangent point 1 (right turn)
        arc1_points = self._generate_arc_points(x_center0, y_center0, tangent_angle1, theta0)
        
        # Straight segment from tangent point 1 to tangent point 2
        straight_segment = np.array([[x_tangent0, y_tangent0], [x_tangent1, y_tangent1]])
        
        # Arc from tangent point 2 to goal (right turn)
        arc2_points = self._generate_arc_points(x_center1, y_center1, theta1, tangent_angle2)
        
        # Combine all segments
        path = np.vstack([arc1_points, straight_segment, arc2_points])
        
        # Return path and its length
        total_length = self._calculate_path_length(arc1_points, straight_segment, arc2_points)
        return {"segments": path, "length": total_length}


    def lrl_path(self):
        x0, y0, theta0 = self.start
        x1, y1, theta1 = self.end
        
        # Step 1: Compute the circle centers for left turn (start) and left turn (goal)
        x_center0 = x0 + self.turning_radius * np.sin(theta0)
        y_center0 = y0 - self.turning_radius * np.cos(theta0)
        x_center1 = x1 + self.turning_radius * np.sin(theta1)
        y_center1 = y1 - self.turning_radius * np.cos(theta1)
        
        # Step 2: Compute the distance between the circle centers
        d = self.distance([x_center0, y_center0], [x_center1, y_center1])
        
        # Step 3: Check if a solution exists (the distance must be greater than 2 * radius)
        if d < 2 * self.turning_radius:
            return None  # No valid path if the circles are too close
        
        # Step 4: Compute the tangent angle
        alpha = np.arcsin((2 * self.turning_radius) / d)
        
        # Step 5: Compute the angle of the line connecting the two circle centers
        theta_line = np.arctan2(y_center1 - y_center0, x_center1 - x_center0)
        
        # Step 6: Compute the tangent points
        tangent_angle1 = theta_line + alpha
        tangent_angle2 = theta_line - alpha
        
        # Compute tangent points on each circle
        x_tangent0 = x_center0 + self.turning_radius * np.cos(tangent_angle1)
        y_tangent0 = y_center0 + self.turning_radius * np.sin(tangent_angle1)
        x_tangent1 = x_center1 + self.turning_radius * np.cos(tangent_angle2)
        y_tangent1 = y_center1 + self.turning_radius * np.sin(tangent_angle2)
        
        # Step 7: Create the path
        # Arc from start to tangent point 1 (left turn)
        arc1_points = self._generate_arc_points(x_center0, y_center0, tangent_angle1, theta0)
        
        # Straight segment from tangent point 1 to tangent point 2
        straight_segment = np.array([[x_tangent0, y_tangent0], [x_tangent1, y_tangent1]])
        
        # Arc from tangent point 2 to goal (left turn)
        arc2_points = self._generate_arc_points(x_center1, y_center1, theta1, tangent_angle2)
        
        # Combine all segments
        path = np.vstack([arc1_points, straight_segment, arc2_points])
        
        # Return path and its length
        total_length = self._calculate_path_length(arc1_points, straight_segment, arc2_points)
        return {"segments": path, "length": total_length}
    
    
    def compute_shortest_path(self):
        paths = {
            "LSL": self.lsl_path(),
            "LSR": self.lsr_path(),
            "RSL": self.rsl_path(),
            "RSR": self.rsr_path(),
            "LRL": self.lrl_path(),
            "RLR": self.rlr_path()
        }

        paths = {key: path for key, path in paths.items() if path is not None}
        shortest_path = min(paths.items(), key=lambda x: x[1]['length']) if paths else None
        return shortest_path
    

    def _generate_arc_points(self, x_center, y_center, start_angle, end_angle, num_points=100):
        """Generate points along the circular arc."""
        angles = np.linspace(start_angle, end_angle, num_points)
        x_points = x_center + self.turning_radius * np.cos(angles)
        y_points = y_center + self.turning_radius * np.sin(angles)
        return np.column_stack((x_points, y_points))
    

    def _calculate_path_length(self, arc1, straight, arc2):
        """Calculate the total length of the path."""
        arc1_length = self.turning_radius * np.abs(np.arctan2(arc1[-1][1] - arc1[0][1], arc1[-1][0] - arc1[0][0]))
        arc2_length = self.turning_radius * np.abs(np.arctan2(arc2[-1][1] - arc2[0][1], arc2[-1][0] - arc2[0][0]))
        straight_length = np.linalg.norm(straight[1] - straight[0])
        return arc1_length + arc2_length + straight_length
