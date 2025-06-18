import numpy as np

class DubinsPath:
    def __init__(self, start, end, turning_radius, step_size=0.1):
        """
        start: (x0, y0, theta0)
        end:   (x1, y1, theta1)
        turning_radius: minimum turning radius R
        step_size: sampling resolution along arcs
        """
        self.start = start
        self.end = end
        self.R = turning_radius
        self.step = step_size

    @staticmethod
    def _mod2pi(angle):
        return angle - 2 * np.pi * np.floor(angle / (2 * np.pi))

    def _center(self, x, y, theta, turn):
        # turn = +1 for left, -1 for right
        return np.array([x - turn * self.R * np.sin(theta),
                         y + turn * self.R * np.cos(theta)])

    def _generate_arc(self, center, start_angle, end_angle, turn):
        # ensure shortest positive rotation for left, negative for right
        sa = self._mod2pi(start_angle)
        ea = self._mod2pi(end_angle)
        if turn == 1:  # left turn
            if ea <= sa:
                ea += 2 * np.pi
        else:  # right turn
            if ea >= sa:
                ea -= 2 * np.pi
        angles = np.arange(sa, ea, (self.step / self.R) * turn)
        # include end angle
        angles = np.append(angles, ea)
        pts = np.vstack([center[0] + self.R * np.cos(angles),
                         center[1] + self.R * np.sin(angles)]).T
        return pts

    def _arc_length(self, delta_angle):
        return abs(delta_angle) * self.R

    def _compute_LSL(self):
        x0, y0, th0 = self.start
        x1, y1, th1 = self.end
        R = self.R
        # circle centers
        c0 = self._center(x0, y0, th0, +1)
        c1 = self._center(x1, y1, th1, +1)
        # vector and distance
        d_vec = c1 - c0
        d = np.linalg.norm(d_vec)
        if d < 2 * R:
            return None
        theta = np.arctan2(d_vec[1], d_vec[0])
        alpha = np.arccos(2 * R / d)
        psi = theta + alpha
        # tangent points
        t0 = c0 + R * np.array([np.cos(psi), np.sin(psi)])
        t1 = c1 + R * np.array([np.cos(psi), np.sin(psi)])
        # arc segments
        arc1 = self._generate_arc(c0, th0 + np.pi/2, psi, +1)
        arc2 = self._generate_arc(c1, psi, th1 + np.pi/2, +1)
        # straight segment
        straight = np.vstack([t0, t1])
        # lengths
        len1 = self._arc_length(self._mod2pi(psi - (th0 + np.pi/2)))
        len2 = np.linalg.norm(t1 - t0)
        len3 = self._arc_length(self._mod2pi((th1 + np.pi/2) - psi))
        total_length = len1 + len2 + len3
        path = np.vstack([arc1, straight, arc2])
        return {'segments': path, 'length': total_length}

    def _compute_RSR(self):
        x0, y0, th0 = self.start
        x1, y1, th1 = self.end
        R = self.R
        c0 = self._center(x0, y0, th0, -1)
        c1 = self._center(x1, y1, th1, -1)
        d_vec = c1 - c0
        d = np.linalg.norm(d_vec)
        if d < 2 * R:
            return None
        theta = np.arctan2(d_vec[1], d_vec[0])
        alpha = np.arccos(2 * R / d)
        psi = theta - alpha
        t0 = c0 + R * np.array([np.cos(psi), np.sin(psi)])
        t1 = c1 + R * np.array([np.cos(psi), np.sin(psi)])
        arc1 = self._generate_arc(c0, th0 - np.pi/2, psi, -1)
        arc2 = self._generate_arc(c1, psi, th1 - np.pi/2, -1)
        straight = np.vstack([t0, t1])
        len1 = self._arc_length(self._mod2pi((th0 - np.pi/2) - psi))
        len2 = np.linalg.norm(t1 - t0)
        len3 = self._arc_length(self._mod2pi(psi - (th1 - np.pi/2)))
        total_length = len1 + len2 + len3
        path = np.vstack([arc1, straight, arc2])
        return {'segments': path, 'length': total_length}

    def _compute_LSR(self):
        x0, y0, th0 = self.start
        x1, y1, th1 = self.end
        R = self.R
        c0 = self._center(x0, y0, th0, +1)
        c1 = self._center(x1, y1, th1, -1)
        d_vec = c1 - c0
        d = np.linalg.norm(d_vec)
        if d <= 2 * R:
            return None
        theta = np.arctan2(d_vec[1], d_vec[0])
        alpha = np.arccos(2 * R / d)
        psi = theta + alpha
        t0 = c0 + R * np.array([np.cos(psi), np.sin(psi)])
        t1 = c1 + R * np.array([np.cos(psi + np.pi), np.sin(psi + np.pi)])
        arc1 = self._generate_arc(c0, th0 + np.pi/2, psi, +1)
        arc2 = self._generate_arc(c1, psi + np.pi, th1 - np.pi/2, -1)
        straight = np.vstack([t0, t1])
        len1 = self._arc_length(self._mod2pi(psi - (th0 + np.pi/2)))
        len2 = np.linalg.norm(t1 - t0)
        len3 = self._arc_length(self._mod2pi((th1 - np.pi/2) - (psi + np.pi)))
        total_length = len1 + len2 + len3
        path = np.vstack([arc1, straight, arc2])
        return {'segments': path, 'length': total_length}

    def _compute_RSL(self):
        x0, y0, th0 = self.start
        x1, y1, th1 = self.end
        R = self.R
        c0 = self._center(x0, y0, th0, -1)
        c1 = self._center(x1, y1, th1, +1)
        d_vec = c1 - c0
        d = np.linalg.norm(d_vec)
        if d <= 2 * R:
            return None
        theta = np.arctan2(d_vec[1], d_vec[0])
        alpha = np.arccos(2 * R / d)
        psi = theta - alpha
        t0 = c0 + R * np.array([np.cos(psi), np.sin(psi)])
        t1 = c1 + R * np.array([np.cos(psi + np.pi), np.sin(psi + np.pi)])
        arc1 = self._generate_arc(c0, th0 - np.pi/2, psi, -1)
        arc2 = self._generate_arc(c1, psi + np.pi, th1 + np.pi/2, +1)
        straight = np.vstack([t0, t1])
        len1 = self._arc_length(self._mod2pi((th0 - np.pi/2) - psi))
        len2 = np.linalg.norm(t1 - t0)
        len3 = self._arc_length(self._mod2pi((th1 + np.pi/2) - (psi + np.pi)))
        total_length = len1 + len2 + len3
        path = np.vstack([arc1, straight, arc2])
        return {'segments': path, 'length': total_length}

    def _compute_LRL(self):
        x0, y0, th0 = self.start
        x1, y1, th1 = self.end
        R = self.R
        c0 = self._center(x0, y0, th0, +1)
        c1 = self._center(x1, y1, th1, +1)
        d = np.linalg.norm(c1 - c0)
        # middle circle separation
        if d > 4 * R:
            return None
        # compute angle between centers
        theta = np.arctan2(c1[1] - c0[1], c1[0] - c0[0])
        # compute intermediate angle
        alpha = np.arccos(d / (4 * R))
        psi1 = theta + alpha + np.pi/2
        psi2 = theta - alpha + np.pi/2
        # tangent points
        t0 = c0 + R * np.array([np.cos(psi1), np.sin(psi1)])
        t1 = c1 + R * np.array([np.cos(psi2), np.sin(psi2)])
        # arcs
        arc1 = self._generate_arc(c0, th0 + np.pi/2, psi1, +1)
        arc2 = self._generate_arc(c1, psi2, psi1 + np.pi, -1)
        arc3 = self._generate_arc(c1, psi1 + np.pi, th1 + np.pi/2, +1)
        straight = np.vstack([t0, t1])  # unused in LRL, placeholder
        len1 = self._arc_length(self._mod2pi(psi1 - (th0 + np.pi/2)))
        len2 = self._arc_length(self._mod2pi((psi1 + np.pi) - psi2))
        len3 = self._arc_length(self._mod2pi((th1 + np.pi/2) - (psi1 + np.pi)))
        total_length = len1 + len2 + len3
        path = np.vstack([arc1, arc2, arc3])
        return {'segments': path, 'length': total_length}

    def _compute_RLR(self):
        x0, y0, th0 = self.start
        x1, y1, th1 = self.end
        R = self.R
        c0 = self._center(x0, y0, th0, -1)
        c1 = self._center(x1, y1, th1, -1)
        d = np.linalg.norm(c1 - c0)
        if d > 4 * R:
            return None
        theta = np.arctan2(c1[1] - c0[1], c1[0] - c0[0])
        alpha = np.arccos(d / (4 * R))
        psi1 = theta - alpha - np.pi/2
        psi2 = theta + alpha - np.pi/2
        t0 = c0 + R * np.array([np.cos(psi1), np.sin(psi1)])
        t1 = c1 + R * np.array([np.cos(psi2), np.sin(psi2)])
        arc1 = self._generate_arc(c0, th0 - np.pi/2, psi1, -1)
        arc2 = self._generate_arc(c1, psi1, psi2 + np.pi, +1)
        arc3 = self._generate_arc(c1, psi2 + np.pi, th1 - np.pi/2, -1)
        len1 = self._arc_length(self._mod2pi((th0 - np.pi/2) - psi1))
        len2 = self._arc_length(self._mod2pi((psi2 + np.pi) - psi1))
        len3 = self._arc_length(self._mod2pi((psi2 + np.pi) - (th1 - np.pi/2)))
        total_length = len1 + len2 + len3
        path = np.vstack([arc1, arc2, arc3])
        return {'segments': path, 'length': total_length}

    def compute_shortest_path(self):
        # gather all path types
        methods = [self._compute_LSL, self._compute_RSR,
                   self._compute_LSR, self._compute_RSL,
                   self._compute_LRL, self._compute_RLR]
        results = {}
        for fn in methods:
            res = fn()
            if res is not None:
                results[fn.__name__[9:].upper()] = res
        if not results:
            return None
        # select minimum length
        best = min(results.values(), key=lambda p: p['length'])
        return best
