import math
from itertools import combinations


class Perception:
    def __init__(self):
        # Obstacle detection thresholds
        self.obstacle_threshold = 0.35

        # Charging station geometry (4 pillars of ~5cm forming a 40x40cm square)
        self.pillar_max_width = 0.15       # max cluster width to be a pillar
        self.station_side_min = 0.30       # min side length tolerance
        self.station_side_max = 0.50       # max side length tolerance
        self.station_diag_min = 0.48       # min diagonal tolerance
        self.station_diag_max = 0.65       # max diagonal tolerance

        # Accumulative pillar memory across scans
        self.known_pillars = []            # list of (x, y) pillar positions
        self.pillar_merge_dist = 0.12      # distance to merge duplicate pillars

        # Detected obstacles in the Passadis area
        self.detected_obstacles = []       # list of (x, y) obstacle centroids

        # Station result
        self.station_found = False
        self.station_center = None         # (x, y)
        self.station_pillars = []          # list of 4 (x, y) positions

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ------------------------------------------------------------------
    # Front obstacle check (simple boolean)
    # ------------------------------------------------------------------

    def check_front_obstacle(self, ranges, angle_min, angle_inc):
        """Returns (blocked, min_distance) for a ±30° front cone."""
        min_distance = float('inf')
        for i, r in enumerate(ranges):
            if r <= 0.05 or not math.isfinite(r):
                continue
            angle = self._normalize_angle(angle_min + i * angle_inc)
            if abs(angle) < math.radians(30):
                min_distance = min(min_distance, r)

        return (min_distance < self.obstacle_threshold), min_distance

    # ------------------------------------------------------------------
    # Scan → Cartesian points
    # ------------------------------------------------------------------

    def _scan_to_cartesian(self, ranges, angle_min, angle_inc,
                           robot_x, robot_y, robot_yaw, max_range=3.5):
        """Convert LaserScan ranges to list of (x, y) in map frame."""
        points = []
        for i, r in enumerate(ranges):
            if r <= 0.05 or not math.isfinite(r) or r > max_range:
                continue
            angle = angle_min + i * angle_inc + robot_yaw
            px = robot_x + r * math.cos(angle)
            py = robot_y + r * math.sin(angle)
            points.append((px, py))
        return points

    # ------------------------------------------------------------------
    # Clustering
    # ------------------------------------------------------------------

    def cluster_points(self, cartesian_points, distance_threshold=0.15):
        """Group nearby points into clusters using centroid-linkage."""
        clusters = []
        for x, y in cartesian_points:
            added = False
            for cluster in clusters:
                cx, cy = cluster['centroid']
                if math.hypot(x - cx, y - cy) < distance_threshold:
                    cluster['points'].append((x, y))
                    pts = cluster['points']
                    cluster['centroid'] = (
                        sum(p[0] for p in pts) / len(pts),
                        sum(p[1] for p in pts) / len(pts),
                    )
                    added = True
                    break
            if not added:
                clusters.append({'points': [(x, y)], 'centroid': (x, y)})
        return clusters

    @staticmethod
    def _cluster_width(cluster):
        """Bounding-box diagonal of a cluster."""
        xs = [p[0] for p in cluster['points']]
        ys = [p[1] for p in cluster['points']]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    # ------------------------------------------------------------------
    # Obstacle detection (for Passadis logging)
    # ------------------------------------------------------------------

    def detect_obstacles(self, ranges, angle_min, angle_inc,
                         robot_x, robot_y, robot_yaw):
        """Identify obstacles in the current scan and accumulate."""
        points = self._scan_to_cartesian(
            ranges, angle_min, angle_inc, robot_x, robot_y, robot_yaw,
            max_range=2.5
        )
        clusters = self.cluster_points(points, distance_threshold=0.20)

        new_obstacles = []
        for c in clusters:
            if len(c['points']) < 3:
                continue
            w = self._cluster_width(c)
            # Filter: obstacles are bigger than pillars but smaller than walls
            if 0.05 < w < 1.0:
                cx, cy = c['centroid']
                # Check if already known
                already = any(
                    math.hypot(cx - ox, cy - oy) < 0.30
                    for ox, oy in self.detected_obstacles
                )
                if not already:
                    self.detected_obstacles.append((cx, cy))
                    new_obstacles.append((cx, cy))

        return new_obstacles

    # ------------------------------------------------------------------
    # Pillar memory (accumulative across scans)
    # ------------------------------------------------------------------

    def _merge_pillar(self, px, py):
        """Add pillar to memory, merging if near an existing one."""
        for i, (kx, ky) in enumerate(self.known_pillars):
            if math.hypot(px - kx, py - ky) < self.pillar_merge_dist:
                # Average with existing
                self.known_pillars[i] = ((kx + px) / 2.0, (ky + py) / 2.0)
                return
        self.known_pillars.append((px, py))

    # ------------------------------------------------------------------
    # Charging station detection (accumulative)
    # ------------------------------------------------------------------

    def find_charging_station(self, ranges, angle_min, angle_inc,
                              robot_x, robot_y, robot_yaw):
        """
        Detect small pillar-like clusters in the current scan,
        merge them into accumulated pillar memory, then check if
        any 4 pillars form the 40x40cm square station.

        Returns:
            (center, pillars) if found, else (None, [])
        """
        if self.station_found:
            return self.station_center, self.station_pillars

        # Convert scan to map-frame points
        points = self._scan_to_cartesian(
            ranges, angle_min, angle_inc, robot_x, robot_y, robot_yaw,
            max_range=3.0
        )
        clusters = self.cluster_points(points, distance_threshold=0.10)

        # Filter for pillar-sized clusters (small, few points)
        for c in clusters:
            if len(c['points']) < 2:
                continue
            w = self._cluster_width(c)
            if w <= self.pillar_max_width:
                self._merge_pillar(*c['centroid'])

        # Try to find 4 pillars forming a square
        if len(self.known_pillars) >= 4:
            result = self._check_square_combination(self.known_pillars)
            if result is not None:
                self.station_found = True
                self.station_center = result[0]
                self.station_pillars = result[1]
                return self.station_center, self.station_pillars

        return None, []

    def _check_square_combination(self, pillars):
        """
        Check all combinations of 4 pillars to find a 40x40cm square.
        Returns (center, [4 pillar coords]) or None.
        """
        for comb in combinations(pillars, 4):
            dists = []
            for i in range(4):
                for j in range(i + 1, 4):
                    d = math.hypot(
                        comb[i][0] - comb[j][0],
                        comb[i][1] - comb[j][1]
                    )
                    dists.append(d)
            dists.sort()

            sides = dists[:4]
            diags = dists[4:]

            sides_ok = all(
                self.station_side_min <= s <= self.station_side_max
                for s in sides
            )
            diags_ok = all(
                self.station_diag_min <= d <= self.station_diag_max
                for d in diags
            )

            if sides_ok and diags_ok:
                center_x = sum(p[0] for p in comb) / 4.0
                center_y = sum(p[1] for p in comb) / 4.0
                return (center_x, center_y), list(comb)

        return None

    def reset(self):
        """Reset all accumulated state."""
        self.known_pillars.clear()
        self.detected_obstacles.clear()
        self.station_found = False
        self.station_center = None
        self.station_pillars = []
