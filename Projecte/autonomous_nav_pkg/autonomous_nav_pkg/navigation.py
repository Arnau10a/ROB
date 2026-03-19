import math

class Navigator:
    def __init__(self):
        self.k_p_linear = 0.5
        self.k_p_angular = 1.5
        self.max_linear = 0.2
        self.max_angular = 0.5
        self.waypoint_tolerance = 0.15 # meters

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def compute_cmd_vel(self, current_x, current_y, current_yaw, goal_x, goal_y):
        """ Computes point-to-point Twist commands. Returns (linear_vel, angular_vel, is_reached) """
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        
        angle_diff = self.normalize_angle(target_angle - current_yaw)
        
        if distance < self.waypoint_tolerance:
            return 0.0, 0.0, True
            
        linear_vel = 0.0
        angular_vel = 0.0
        
        # Turn towards the goal
        if abs(angle_diff) > 0.2:
            angular_vel = self.k_p_angular * angle_diff
            linear_vel = 0.0  # Turn in place if angle is too large
        else:
            angular_vel = self.k_p_angular * angle_diff
            linear_vel = self.k_p_linear * distance
            
        # Clamp bounds
        linear_vel = max(-self.max_linear, min(self.max_linear, linear_vel))
        angular_vel = max(-self.max_angular, min(self.max_angular, angular_vel))
        
        return linear_vel, angular_vel, False

