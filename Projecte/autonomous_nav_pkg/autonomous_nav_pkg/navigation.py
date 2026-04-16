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

    def compute_apf_cmd_vel(self, current_x, current_y, current_yaw, goal_x, goal_y, ranges, angle_min, angle_inc):
        """ Computes Twist commands using Artificial Potential Fields (APF). """
        # Goal state
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < self.waypoint_tolerance:
            return 0.0, 0.0, True

        # Attractive force parameters
        k_att = 1.0
        # Repulsive force parameters
        k_rep = 0.05
        d_safe = 0.45 # Distancia de influencia del obstáculo
        
        # 1. Attractive vector calculation
        v_att_x = k_att * (dx / distance)
        v_att_y = k_att * (dy / distance)

        # 2. Repulsive vector calculation
        v_rep_x = 0.0
        v_rep_y = 0.0
        
        if ranges is not None and len(ranges) > 0:
            for i, r in enumerate(ranges):
                if 0.05 < r < d_safe:
                    # Ignore obstacles behind the robot to prioritize going forward
                    angle = angle_min + i * angle_inc
                    while angle > math.pi: angle -= 2 * math.pi
                    while angle < -math.pi: angle += 2 * math.pi
                    
                    if abs(angle) < math.pi / 2.0: # Only care about front 180 degrees
                        # Force magnitude
                        rep_force = k_rep * ((1.0 / r) - (1.0 / d_safe)) * (1.0 / (r**2))
                        
                        # Convert to global coordinates based on robot yaw
                        abs_angle = current_yaw + angle
                        v_rep_x += rep_force * (-math.cos(abs_angle))
                        v_rep_y += rep_force * (-math.sin(abs_angle))

        # 3. Sum of forces
        v_tot_x = v_att_x + v_rep_x
        v_tot_y = v_att_y + v_rep_y
        
        target_angle = math.atan2(v_tot_y, v_tot_x)
        angle_diff = self.normalize_angle(target_angle - current_yaw)
        
        # Calculate required velocity
        angular_vel = self.k_p_angular * angle_diff
        
        # If the turn is very sharp, slow down the linear velocity
        if abs(angle_diff) > 0.5:
            linear_vel = 0.05
        else:
            linear_vel = self.k_p_linear * min(distance, 1.0) # cap attractive distance effect
            
        # Clamp bounds
        linear_vel = max(-self.max_linear, min(self.max_linear, linear_vel))
        angular_vel = max(-self.max_angular, min(self.max_angular, angular_vel))
        
        return linear_vel, angular_vel, False


