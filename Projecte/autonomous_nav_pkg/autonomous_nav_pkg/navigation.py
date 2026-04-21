import math


class Navigator:
    def __init__(self):
        # ── Ganancias del controlador ──────────────────────────────────────────
        self.k_p_linear = 0.42       # ganancia proporcional para velocidad lineal
        self.k_p_angular = 1.4       # ganancia proporcional para velocidad angular

        # ── Límites de velocidad ───────────────────────────────────────────────
        self.max_linear = 0.20       # velocidad lineal máxima (m/s)
        self.max_angular = 0.60      # velocidad angular máxima (rad/s)

        # ── Condición de llegada al waypoint ───────────────────────────────────
        self.waypoint_tolerance = 0.15   # distancia mínima para considerar meta alcanzada (m)

        # ── Umbrales de obstáculos ─────────────────────────────────────────────
        self.emergency_stop_dist = 0.14  # parada de emergencia: obstáculo < 14 cm
        self.caution_dist = 0.30         # zona de precaución: obstáculo < 30 cm → girar en sitio

        # ── Política de movimiento (controlador go-to-goal simplificado) ───────
        self.align_angle = 0.80      # umbral de alineación: si |angle_diff| < 0.80 rad → avanzar
        self.min_linear = 0.03       # velocidad lineal mínima para evitar quedarse parado (m/s)

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

    def compute_apf_cmd_vel(self, current_x, current_y, current_yaw, goal_x, goal_y, ranges, angle_min, angle_inc):
        # Distancia y ángulo directo al objetivo (go-to-goal)
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.hypot(dx, dy)

        # Meta alcanzada: parar y señalizar
        if distance < self.waypoint_tolerance:
            return 0.0, 0.0, True, "GOAL_REACHED", {
                "distance": distance,
                "min_front": float("inf"),
                "min_left": float("inf"),
                "min_right": float("inf"),
                "nearest_all": float("inf"),
                "obstacle_detected": False,
                "repulsive_mag": 0.0,
                "angle_diff": 0.0,
                "danger_now": False,
                "avoid_mode": False,
                "danger_count": 0,
                "clear_count": 0,
            }

        # Estadísticas del láser (frente, izquierda, derecha, más cercano)
        min_front, min_left, min_right, nearest_all = self._scan_stats(ranges, angle_min, angle_inc)

        # Ángulo hacia el objetivo y diferencia con la orientación actual
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - current_yaw)

        # ── Parada de emergencia: obstáculo muy cerca por delante ──────────────
        if min_front < self.emergency_stop_dist:
            # Girar hacia el lado más libre para escapar del obstáculo
            turn_sign = 1.0 if min_left >= min_right else -1.0
            return 0.0, turn_sign * self.max_angular, False, "EMERGENCY_AVOID", {
                "distance": distance,
                "min_front": min_front,
                "min_left": min_left,
                "min_right": min_right,
                "nearest_all": nearest_all,
                "obstacle_detected": True,
                "repulsive_mag": 0.0,
                "angle_diff": angle_diff,
                "danger_now": True,
                "avoid_mode": True,
                "danger_count": 1,
                "clear_count": 0,
            }

        # ── Zona de precaución: obstáculo cerca por delante → girar en sitio ───
        if min_front < self.caution_dist:
            # Girar hacia el lado con más espacio libre
            turn_sign = 1.0 if min_left >= min_right else -1.0
            return 0.0, turn_sign * self.max_angular, False, "AVOIDING_OBSTACLE", {
                "distance": distance,
                "min_front": min_front,
                "min_left": min_left,
                "min_right": min_right,
                "nearest_all": nearest_all,
                "obstacle_detected": True,
                "repulsive_mag": 0.0,
                "angle_diff": angle_diff,
                "danger_now": True,
                "avoid_mode": True,
                "danger_count": 1,
                "clear_count": 0,
            }

        # ── Controlador go-to-goal: sin obstáculos bloqueando el frente ────────

        # Velocidad angular proporcional al error de orientación
        angular_vel = self.k_p_angular * angle_diff
        angular_vel = max(-self.max_angular, min(self.max_angular, angular_vel))

        if abs(angle_diff) < self.align_angle:
            # Suficientemente alineado: avanzar (reducir velocidad si el ángulo es grande)
            # La velocidad lineal escala con cos(angle_diff) para frenar al girar
            scale = max(0.0, math.cos(angle_diff))
            linear_vel = self.k_p_linear * min(distance, 1.0) * scale
            # Garantizar velocidad mínima para no quedarse parado
            linear_vel = max(self.min_linear, linear_vel)
            linear_vel = min(self.max_linear, linear_vel)
            nav_state = "GOING_STRAIGHT" if abs(angle_diff) < 0.3 else "TURNING_TO_GOAL"
        else:
            # Ángulo demasiado grande: girar en sitio antes de avanzar
            linear_vel = 0.0
            nav_state = "TURNING_TO_GOAL"

        return linear_vel, angular_vel, False, nav_state, {
            "distance": distance,
            "min_front": min_front,
            "min_left": min_left,
            "min_right": min_right,
            "nearest_all": nearest_all,
            "obstacle_detected": False,
            "repulsive_mag": 0.0,
            "angle_diff": angle_diff,
            "danger_now": False,
            "avoid_mode": False,
            "danger_count": 0,
            "clear_count": 0,
        }
