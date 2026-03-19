import math

class Perception:
    def __init__(self):
        self.obstacle_threshold = 0.35 # Stop distance for obstacles

    def check_front_obstacle(self, ranges, angle_min, angle_inc):
        """ Returns true if there's an obstacle directly in front of the robot. """
        # We check a cone of -30 to +30 degrees in front
        num_angles = len(ranges)
        min_distance = float('inf')
        
        for i, r in enumerate(ranges):
            if r > 0.05 and r != float('inf'): # Ignore zero/inf
                # Calculate angle for this range index
                # LaserScan angles usually go from angle_min up to angle_max
                angle = angle_min + i * angle_inc
                
                # Normalize
                while angle > math.pi: angle -= 2*math.pi
                while angle < -math.pi: angle += 2*math.pi
                
                if abs(angle) < math.radians(30):
                    if r < min_distance:
                        min_distance = r

        if min_distance < self.obstacle_threshold:
            return True, min_distance
        return False, min_distance

    def cluster_points(self, cartesian_points, distance_threshold=0.15):
        """ Group nearby points into clusters. """
        clusters = []
        for x, y in cartesian_points:
            added_to_cluster = False
            for cluster in clusters:
                # check distance to the cluster centroid or any point
                cx, cy = cluster['centroid']
                if math.sqrt((x - cx)**2 + (y - cy)**2) < distance_threshold:
                    cluster['points'].append((x, y))
                    # update centroid
                    cluster['centroid'] = (
                        sum(p[0] for p in cluster['points']) / len(cluster['points']),
                        sum(p[1] for p in cluster['points']) / len(cluster['points'])
                    )
                    added_to_cluster = True
                    break
            
            if not added_to_cluster:
                clusters.append({'points': [(x, y)], 'centroid': (x, y)})
                
        return clusters

    def find_charging_station(self, ranges, angle_min, angle_inc, robot_x, robot_y, robot_yaw):
        """ Identify the 4 pillars of the 40x40cm charging station in a single scan. """
        points = []
        for i, r in enumerate(ranges):
            if r > 0.05 and r < 3.0 and r != float('inf'):
                angle = angle_min + i * angle_inc + robot_yaw
                px = robot_x + r * math.cos(angle)
                py = robot_y + r * math.sin(angle)
                points.append((px, py))
                
        clusters = self.cluster_points(points, distance_threshold=0.15)
        
        # Filter clusters by size. Pillars are small (5cm diameter).
        valid_pillars = []
        for c in clusters:
            # check width of cluster
            if len(c['points']) < 2: continue
            xs = [p[0] for p in c['points']]
            ys = [p[1] for p in c['points']]
            width = math.sqrt((max(xs) - min(xs))**2 + (max(ys) - min(ys))**2)
            if width <= 0.15: # allow some margin for 5cm
                valid_pillars.append(c['centroid'])
                
        # Now search for 4 pillars forming a 40x40 square
        if len(valid_pillars) >= 4:
            from itertools import combinations
            for comb in combinations(valid_pillars, 4):
                # Calculate all pairwise distances (6 pairs for 4 points)
                dists = []
                for i in range(4):
                    for j in range(i + 1, 4):
                        dx = comb[i][0] - comb[j][0]
                        dy = comb[i][1] - comb[j][1]
                        dists.append(math.sqrt(dx**2 + dy**2))
                        
                dists.sort()
                # A 40x40 square has 4 sides of ~0.4m and 2 diagonals of ~0.565m
                sides = dists[:4]
                diags = dists[4:]
                
                # Verify if it fits a 0.4x0.4m square geometry with tolerance
                if all(0.30 <= s <= 0.50 for s in sides) and all(0.48 <= d <= 0.65 for d in diags):
                    # Found the station! Return center
                    center_x = sum(p[0] for p in comb) / 4.0
                    center_y = sum(p[1] for p in comb) / 4.0
                    return (center_x, center_y), list(comb)
                    
        return None, []
