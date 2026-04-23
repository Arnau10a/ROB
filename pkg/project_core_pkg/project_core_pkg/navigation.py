import math


class Navigator:
    def __init__(self):
        # Control gains
        self.k_p_linear = 0.42
        self.k_p_angular = 1.4

        # Velocity limits
        self.max_linear = 0.20
        self.max_angular = 0.60

        # Goal condition
        self.waypoint_tolerance = 0.15  # meters

        # APF params (más suave)
        self.k_att = 1.0
        self.k_rep = 0.04
        self.d_safe = 0.40

        # Obstacle thresholds
        self.emergency_stop_dist = 0.14
        self.caution_dist = 0.24

        # Anti flicker / hysteresis
        self.avoid_enter_count_required = 3
        self.avoid_exit_count_required = 5

        self._danger_count = 0
        self._clear_count = 0
        self._in_avoid_mode = False

        # Cap de repulsión total para evitar giros locos
        self.max_repulsive_mag = 0.80

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _scan_stats(self, ranges, angle_min, angle_inc):
        if not ranges:
            inf = float("inf")
            return inf, inf, inf, inf

        min_front = float("inf")
        min_left = float("inf")
        min_right = float("inf")
        nearest_all = float("inf")

        for i, r in enumerate(ranges):
            if r is None or not math.isfinite(r) or r <= 0.10:
                continue
            if r > 20.0:
                continue

            nearest_all = min(nearest_all, r)

            a = self.normalize_angle(angle_min + i * angle_inc)
            deg = math.degrees(a)

            if -25.0 <= deg <= 25.0:
                min_front = min(min_front, r)
            elif 25.0 < deg <= 100.0:
                min_left = min(min_left, r)
            elif -100.0 <= deg < -25.0:
                min_right = min(min_right, r)

        return min_front, min_left, min_right, nearest_all

    def _update_avoid_hysteresis(self, danger_now: bool):
        if danger_now:
            self._danger_count += 1
            self._clear_count = 0
        else:
            self._clear_count += 1
            self._danger_count = 0

        if not self._in_avoid_mode and self._danger_count >= self.avoid_enter_count_required:
            self._in_avoid_mode = True

        if self._in_avoid_mode and self._clear_count >= self.avoid_exit_count_required:
            self._in_avoid_mode = False

    def _cap_repulsive(self, vx, vy):
        mag = math.hypot(vx, vy)
        if mag <= self.max_repulsive_mag or mag < 1e-9:
            return vx, vy, mag
        scale = self.max_repulsive_mag / mag
        return vx * scale, vy * scale, self.max_repulsive_mag

    def compute_apf_cmd_vel(self, current_x, current_y, current_yaw, goal_x, goal_y, ranges, angle_min, angle_inc):
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.hypot(dx, dy)

        if distance < self.waypoint_tolerance:
            self._danger_count = 0
            self._clear_count = 0
            self._in_avoid_mode = False
            return 0.0, 0.0, True, "GOAL_REACHED", {
                "distance": distance,
                "obstacle_detected": False,
                "danger_now": False,
                "avoid_mode": False,
                "danger_count": 0,
                "clear_count": 0,
                "repulsive_mag": 0.0,
                "angle_diff": 0.0,
            }

        min_front, min_left, min_right, nearest_all = self._scan_stats(ranges, angle_min, angle_inc)

        # Emergencia frontal
        if min_front < self.emergency_stop_dist:
            turn_sign = -1.0 if min_left < min_right else 1.0
            linear_vel = 0.0
            angular_vel = turn_sign * 0.55
            self._in_avoid_mode = True
            self._danger_count = self.avoid_enter_count_required
            self._clear_count = 0

            return linear_vel, angular_vel, False, "EMERGENCY_AVOID", {
                "distance": distance,
                "min_front": min_front,
                "min_left": min_left,
                "min_right": min_right,
                "nearest_all": nearest_all,
                "obstacle_detected": True,
                "repulsive_mag": 0.0,
                "angle_diff": 0.0,
                "danger_now": True,
                "avoid_mode": self._in_avoid_mode,
                "danger_count": self._danger_count,
                "clear_count": self._clear_count,
            }

        # Attractive
        v_att_x = self.k_att * (dx / max(distance, 1e-6))
        v_att_y = self.k_att * (dy / max(distance, 1e-6))

        # Repulsive
        v_rep_x = 0.0
        v_rep_y = 0.0
        obstacle_detected = False

        if ranges:
            for i, r in enumerate(ranges):
                if r is None or not math.isfinite(r) or r <= 0.10:
                    continue
                if r >= self.d_safe:
                    continue

                angle = self.normalize_angle(angle_min + i * angle_inc)

                # usar frente + laterales (240º)
                if abs(angle) > math.radians(120):
                    continue

                obstacle_detected = True
                rep_force = self.k_rep * ((1.0 / r) - (1.0 / self.d_safe)) * (1.0 / (r * r))
                abs_angle = current_yaw + angle

                v_rep_x += rep_force * (-math.cos(abs_angle))
                v_rep_y += rep_force * (-math.sin(abs_angle))

        # cap repulsión
        v_rep_x, v_rep_y, repulsive_mag = self._cap_repulsive(v_rep_x, v_rep_y)

        # Total vector
        v_tot_x = v_att_x + v_rep_x
        v_tot_y = v_att_y + v_rep_y

        target_angle = math.atan2(v_tot_y, v_tot_x)
        angle_diff = self.normalize_angle(target_angle - current_yaw)

        # Condición de peligro instantánea
        danger_now = (min_front < self.caution_dist) or (repulsive_mag > 0.40)
        self._update_avoid_hysteresis(danger_now)

        # Policy
        if self._in_avoid_mode:
            nav_state = "AVOIDING_OBSTACLE"
            if abs(angle_diff) > 0.55:
                linear_vel = 0.0
            else:
                linear_vel = 0.05
            angular_vel = self.k_p_angular * angle_diff
        else:
            if abs(angle_diff) > 0.45:
                nav_state = "TURNING_TO_PATH"
                linear_vel = 0.0
                angular_vel = self.k_p_angular * angle_diff
            elif abs(angle_diff) > 0.15:
                nav_state = "PATH_CORRECTION"
                linear_vel = self.k_p_linear * min(distance, 1.0) * 0.6
                angular_vel = self.k_p_angular * angle_diff
            else:
                nav_state = "GOING_STRAIGHT"
                linear_vel = self.k_p_linear * min(distance, 1.0)
                angular_vel = self.k_p_angular * angle_diff

        linear_vel = max(-self.max_linear, min(self.max_linear, linear_vel))
        angular_vel = max(-self.max_angular, min(self.max_angular, angular_vel))

        return linear_vel, angular_vel, False, nav_state, {
            "distance": distance,
            "min_front": min_front,
            "min_left": min_left,
            "min_right": min_right,
            "nearest_all": nearest_all,
            "obstacle_detected": obstacle_detected,
            "repulsive_mag": repulsive_mag,
            "angle_diff": angle_diff,
            "danger_now": danger_now,
            "avoid_mode": self._in_avoid_mode,
            "danger_count": self._danger_count,
            "clear_count": self._clear_count,
        }
