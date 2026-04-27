import math
from itertools import combinations

class Perception:
    def __init__(self):
        self.obstacle_threshold = 0.35

        self.pillar_max_width = 0.15
        self.station_side_min = 0.30
        self.station_side_max = 0.50
        self.station_diag_min = 0.48
        self.station_diag_max = 0.65

        self.known_pillars = []
        self.pillar_merge_dist = 0.12

        self.detected_obstacles = []

        self.station_found = False
        self.station_center = None
        self.station_pillars = []

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def check_front_obstacle(self, ranges, angle_min, angle_inc):
        min_distance = float('inf')
        for i, r in enumerate(ranges):
            if r <= 0.05 or not math.isfinite(r):
                continue
            angle = self._normalize_angle(angle_min + i * angle_inc)
            if abs(angle) < math.radians(30):
                min_distance = min(min_distance, r)

        return (min_distance < self.obstacle_threshold), min_distance

    def _scan_to_cartesian(self, ranges, angle_min, angle_inc,
                           robot_x, robot_y, robot_yaw, max_range=3.5):
        points = []
        for i, r in enumerate(ranges):
            if r <= 0.05 or not math.isfinite(r) or r > max_range:
                continue
            angle = angle_min + i * angle_inc + robot_yaw
            px = robot_x + r * math.cos(angle)
            py = robot_y + r * math.sin(angle)
            points.append((px, py))
        return points

    def cluster_points(self, cartesian_points, distance_threshold=0.15):
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
        xs = [p[0] for p in cluster['points']]
        ys = [p[1] for p in cluster['points']]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys))

    def detect_obstacles(self, ranges, angle_min, angle_inc,
                         robot_x, robot_y, robot_yaw):
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
            if 0.05 < w < 1.0:
                cx, cy = c['centroid']
                already = any(
                    math.hypot(cx - ox, cy - oy) < 0.30
                    for ox, oy in self.detected_obstacles
                )
                if not already:
                    self.detected_obstacles.append((cx, cy))
                    new_obstacles.append((cx, cy))

        return new_obstacles

    def _merge_pillar(self, px, py):
        for i, (kx, ky) in enumerate(self.known_pillars):
            if math.hypot(px - kx, py - ky) < self.pillar_merge_dist:
                self.known_pillars[i] = ((kx + px) / 2.0, (ky + py) / 2.0)
                return
        self.known_pillars.append((px, py))

    def find_charging_station(self, ranges, angle_min, angle_inc,
                               robot_x, robot_y, robot_yaw):
        if self.station_found:
            return self.station_center, self.station_pillars

        points = self._scan_to_cartesian(
            ranges, angle_min, angle_inc, robot_x, robot_y, robot_yaw,
            max_range=3.0
        )
        clusters = self.cluster_points(points, distance_threshold=0.10)

        for c in clusters:
            if len(c['points']) < 2:
                continue
            w = self._cluster_width(c)
            if w <= self.pillar_max_width:
                self._merge_pillar(*c['centroid'])

        if len(self.known_pillars) >= 4:
            result = self._check_square_combination(self.known_pillars)
            if result is not None:
                self.station_found = True
                self.station_center = result[0]
                self.station_pillars = result[1]
                return self.station_center, self.station_pillars

        return None, []

    def _check_square_combination(self, pillars):
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
        self.known_pillars.clear()
        self.detected_obstacles.clear()
        self.station_found = False
        self.station_center = None
        self.station_pillars = []

