import math
import csv
import os
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

from autonomous_nav_pkg.navigation import Navigator
from autonomous_nav_pkg.perception import Perception


# ──────────────────────────────────────────────────────────────────────
# Helper
# ──────────────────────────────────────────────────────────────────────

def yaw_from_quaternion(q):
    """Extract yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Wrap angle to [-π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ──────────────────────────────────────────────────────────────────────
# Reference points (from the project document)
# ──────────────────────────────────────────────────────────────────────

PUNT_B = (3.72, 2.55)
PUNT_O = (5.10, 12.61)
PUNT_BASE = (5.00, 11.69)
PUNT_P = (0.30, 11.01)
PUNT_Q = (1.90, 12.21)
PUNT_R = (7.12, 12.61)
PORTA = (5.92, 8.12)

# Passadís exploration area (estimated from reference points)
PASSADIS_X_MIN = 0.5
PASSADIS_X_MAX = 7.0
PASSADIS_Y_MIN = 10.5
PASSADIS_Y_MAX = 13.5

# Lawnmower exploration step
EXPLORATION_STEP_X = 1.5   # metres between vertical sweeps
EXPLORATION_STEP_Y = 2.0   # vertical extent per sweep


# ──────────────────────────────────────────────────────────────────────
# Exploration waypoint generator
# ──────────────────────────────────────────────────────────────────────

def generate_exploration_waypoints():
    """Generate a lawnmower pattern covering the Passadís area."""
    waypoints = []
    x = PASSADIS_X_MIN
    going_up = True

    while x <= PASSADIS_X_MAX:
        if going_up:
            waypoints.append((x, PASSADIS_Y_MIN))
            waypoints.append((x, PASSADIS_Y_MAX))
        else:
            waypoints.append((x, PASSADIS_Y_MAX))
            waypoints.append((x, PASSADIS_Y_MIN))
        going_up = not going_up
        x += EXPLORATION_STEP_X

    return waypoints


# ══════════════════════════════════════════════════════════════════════
# Mission Controller Node
# ══════════════════════════════════════════════════════════════════════

class MissionController(Node):

    # ── Phase constants ──────────────────────────────────────────────
    PHASE_I = "I"
    PHASE_II = "II"
    PHASE_III = "III"
    PHASE_DONE = "DONE"

    def __init__(self):
        super().__init__('mission_controller_node')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('use_sim_time', False)

        # ── QoS ──────────────────────────────────────────────────────
        qos_best_effort = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ── Subscribers ──────────────────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_best_effort
        )

        # ── Publisher ────────────────────────────────────────────────
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── TF for SLAM-based localisation ───────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_ready = False

        # ── Timer (20 Hz) ────────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.timer_callback)

        # ── Sub-modules ──────────────────────────────────────────────
        self.navigator = Navigator()
        self.perception = Perception()

        # ── Odometry state (fallback) ────────────────────────────────
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_ready = False

        # ── Initial pose (D) ────────────────────────────────────────
        self.initial_x = 3.32
        self.initial_y = 0.95
        self.initial_yaw = 0.0

        # ── Current pose in map frame ────────────────────────────────
        self.map_x = self.initial_x
        self.map_y = self.initial_y
        self.map_yaw = self.initial_yaw

        # ── Scan state ───────────────────────────────────────────────
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.scan_ready = False

        # ── Phase I: D → B → O ──────────────────────────────────────
        self.phase = self.PHASE_I
        self.waypoints_phase_1 = [PUNT_B, PUNT_O]
        self.current_wp_idx = 0

        # ── Phase II: Exploration ────────────────────────────────────
        self.exploration_wps = generate_exploration_waypoints()
        self.exploration_wp_idx = 0
        self.exploration_returning = False  # True once heading to Punt Base

        # ── Phase III: Docking ───────────────────────────────────────
        self.docking_approach_dist = 0.30   # switch to precision at 30cm

        # ── CSV log ──────────────────────────────────────────────────
        self.log_file_path = os.path.join(os.getcwd(), 'mission_log.csv')
        self._init_csv_log()

        # ── Console throttling ───────────────────────────────────────
        self._last_state = None
        self._last_state_log_ns = 0
        self._state_log_period_ns = int(0.5e9)
        self._heartbeat_period_ns = int(2.0e9)
        self._last_heartbeat_ns = 0

        # ── Diagnostics ──────────────────────────────────────────────
        self._odom_count = 0
        self._scan_count = 0
        self._last_diag_ns = 0
        self._diag_period_ns = int(2.0e9)

        # ── Startup log ──────────────────────────────────────────────
        self.get_logger().info('Mission Controller initialised')
        self.get_logger().info(
            f'Initial pose (D): x={self.initial_x:.2f}, '
            f'y={self.initial_y:.2f}, yaw={self.initial_yaw:.2f}'
        )
        self.get_logger().info(f'Phase I route: {self.waypoints_phase_1}')
        self.get_logger().info(
            f'Phase II exploration: {len(self.exploration_wps)} waypoints'
        )

    # ==================================================================
    # CSV Logging
    # ==================================================================

    def _init_csv_log(self):
        with open(self.log_file_path, mode='w', newline='') as f:
            csv.writer(f).writerow([
                'timestamp_sec', 'phase', 'wp_index',
                'goal_x', 'goal_y',
                'robot_x', 'robot_y', 'robot_yaw',
                'linear_cmd', 'angular_cmd', 'nav_state',
                'distance_to_goal', 'angle_diff',
                'obstacle_detected',
                'min_front', 'min_left', 'min_right', 'nearest_all',
                'repulsive_mag', 'danger_now', 'avoid_mode',
                'danger_count', 'clear_count',
                'station_found', 'station_x', 'station_y',
                'pillar_positions', 'obstacles_detected',
            ])

    def _append_csv(self, goal_x, goal_y, lin, ang, nav_state, nav_debug):
        station_x = station_y = ''
        pillar_str = ''
        obstacle_str = ''

        if self.perception.station_found:
            station_x = f'{self.perception.station_center[0]:.3f}'
            station_y = f'{self.perception.station_center[1]:.3f}'
            pillar_str = ';'.join(
                f'{p[0]:.3f},{p[1]:.3f}'
                for p in self.perception.station_pillars
            )

        if self.perception.detected_obstacles:
            obstacle_str = ';'.join(
                f'{o[0]:.3f},{o[1]:.3f}'
                for o in self.perception.detected_obstacles
            )

        with open(self.log_file_path, mode='a', newline='') as f:
            csv.writer(f).writerow([
                self.get_clock().now().to_msg().sec,
                self.phase,
                self.current_wp_idx,
                f'{goal_x:.3f}', f'{goal_y:.3f}',
                f'{self.map_x:.3f}', f'{self.map_y:.3f}',
                f'{self.map_yaw:.3f}',
                f'{lin:.3f}', f'{ang:.3f}',
                nav_state,
                self._fmt(nav_debug.get('distance')),
                self._fmt(nav_debug.get('angle_diff')),
                nav_debug.get('obstacle_detected', ''),
                self._fmt(nav_debug.get('min_front')),
                self._fmt(nav_debug.get('min_left')),
                self._fmt(nav_debug.get('min_right')),
                self._fmt(nav_debug.get('nearest_all')),
                self._fmt(nav_debug.get('repulsive_mag')),
                nav_debug.get('danger_now', ''),
                nav_debug.get('avoid_mode', ''),
                nav_debug.get('danger_count', ''),
                nav_debug.get('clear_count', ''),
                self.perception.station_found,
                station_x, station_y,
                pillar_str, obstacle_str,
            ])

    @staticmethod
    def _fmt(val):
        """Format a numeric value to 3 decimal places, or empty string."""
        if val is None:
            return ''
        try:
            return f'{float(val):.3f}'
        except (TypeError, ValueError):
            return ''

    # ==================================================================
    # Localisation
    # ==================================================================

    def update_pose_from_tf(self):
        """
        Try to get the pose from SLAM TF (map → base_link).
        Falls back to dead-reckoning from odometry.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time()
            )
            self.map_x = t.transform.translation.x
            self.map_y = t.transform.translation.y
            self.map_yaw = yaw_from_quaternion(t.transform.rotation)
            self.tf_ready = True
        except TransformException:
            # Fallback: dead reckoning
            if self.odom_ready:
                self.map_x = self.initial_x + self.odom_x
                self.map_y = self.initial_y + self.odom_y
                self.map_yaw = self.initial_yaw + self.odom_yaw

    # ==================================================================
    # Callbacks
    # ==================================================================

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ready = True
        self._odom_count += 1

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.scan_ready = True
        self._scan_count += 1

    # ==================================================================
    # Velocity helpers
    # ==================================================================

    def publish_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        self.publish_vel(0.0, 0.0)

    # ==================================================================
    # Console logging (throttled)
    # ==================================================================

    def _maybe_log_state(self, nav_state, nav_debug,
                         goal_x, goal_y, lin, ang):
        now_ns = self.get_clock().now().nanoseconds
        changed = (nav_state != self._last_state)
        periodic = (now_ns - self._last_state_log_ns) >= self._state_log_period_ns

        if changed or periodic:
            loc_src = 'TF' if self.tf_ready else 'DR'
            self.get_logger().info(
                f'[{self.phase}|{loc_src}] {nav_state} '
                f'wp={self.current_wp_idx + 1} '
                f'goal=({goal_x:.2f},{goal_y:.2f}) '
                f'pos=({self.map_x:.2f},{self.map_y:.2f}) '
                f'd={self._fmt(nav_debug.get("distance"))} '
                f'obs={nav_debug.get("obstacle_detected", False)} '
                f'cmd=({lin:.2f},{ang:.2f})'
            )
            self._last_state = nav_state
            self._last_state_log_ns = now_ns

        # Heartbeat
        if (now_ns - self._last_heartbeat_ns) >= self._heartbeat_period_ns:
            self.get_logger().info(
                f'[HEARTBEAT] phase={self.phase} '
                f'pose=({self.map_x:.2f},{self.map_y:.2f},{self.map_yaw:.2f}) '
                f'tf_ok={self.tf_ready} '
                f'station={self.perception.station_found}'
            )
            self._last_heartbeat_ns = now_ns

    def _diagnostics(self):
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_diag_ns) < self._diag_period_ns:
            return
        self.get_logger().info(
            f'[DIAG] odom_msgs={self._odom_count} '
            f'scan_msgs={self._scan_count} '
            f'pillars_known={len(self.perception.known_pillars)} '
            f'obstacles={len(self.perception.detected_obstacles)}'
        )
        self._odom_count = 0
        self._scan_count = 0
        self._last_diag_ns = now_ns

    # ==================================================================
    # Main timer
    # ==================================================================

    def timer_callback(self):
        self._diagnostics()
        self.update_pose_from_tf()

        if not self.odom_ready or not self.scan_ready:
            return

        if self.phase == self.PHASE_I:
            self.execute_phase_1()
        elif self.phase == self.PHASE_II:
            self.execute_phase_2()
        elif self.phase == self.PHASE_III:
            self.execute_phase_3()
        elif self.phase == self.PHASE_DONE:
            self.stop_robot()

    # ==================================================================
    # Phase I — Global Navigation: D → B → O
    # ==================================================================

    def execute_phase_1(self):
        if self.current_wp_idx >= len(self.waypoints_phase_1):
            self.get_logger().info(
                '[PHASE I] Complete — Transitioning to Phase II'
            )
            self.phase = self.PHASE_II
            self.current_wp_idx = 0
            return

        goal_x, goal_y = self.waypoints_phase_1[self.current_wp_idx]

        lin, ang, reached, nav_state, nav_debug = (
            self.navigator.compute_apf_cmd_vel(
                self.map_x, self.map_y, self.map_yaw,
                goal_x, goal_y,
                self.scan_ranges, self.angle_min, self.angle_inc,
            )
        )

        self._maybe_log_state(nav_state, nav_debug, goal_x, goal_y, lin, ang)
        self._append_csv(goal_x, goal_y, lin, ang, nav_state, nav_debug)

        if reached:
            self.get_logger().info(
                f'[WAYPOINT] Reached wp {self.current_wp_idx + 1}: '
                f'({goal_x:.2f}, {goal_y:.2f})'
            )
            self.current_wp_idx += 1
        else:
            self.publish_vel(lin, ang)

    # ==================================================================
    # Phase II — Exploration & Perception
    # ==================================================================

    def execute_phase_2(self):
        # Always try to detect the charging station
        self.perception.find_charging_station(
            self.scan_ranges, self.angle_min, self.angle_inc,
            self.map_x, self.map_y, self.map_yaw,
        )

        # Detect obstacles in the Passadís for logging
        new_obs = self.perception.detect_obstacles(
            self.scan_ranges, self.angle_min, self.angle_inc,
            self.map_x, self.map_y, self.map_yaw,
        )
        if new_obs:
            for ox, oy in new_obs:
                self.get_logger().info(
                    f'[OBSTACLE] Detected at ({ox:.2f}, {oy:.2f})'
                )

        # If station just found, switch to returning to Punt Base
        if self.perception.station_found and not self.exploration_returning:
            cx, cy = self.perception.station_center
            self.get_logger().info(
                f'[STATION] Charging station found at '
                f'({cx:.2f}, {cy:.2f})!'
            )
            for i, p in enumerate(self.perception.station_pillars):
                self.get_logger().info(
                    f'[STATION] Pillar {i + 1}: ({p[0]:.2f}, {p[1]:.2f})'
                )
            self.exploration_returning = True

        # Determine current goal
        if self.exploration_returning:
            goal_x, goal_y = PUNT_BASE
            label = 'RETURNING_TO_BASE'
        else:
            if self.exploration_wp_idx >= len(self.exploration_wps):
                # Exploration finished without finding station
                self.get_logger().warn(
                    '[PHASE II] Exploration complete — '
                    'station NOT found, returning to base anyway'
                )
                self.exploration_returning = True
                goal_x, goal_y = PUNT_BASE
                label = 'RETURNING_TO_BASE'
            else:
                goal_x, goal_y = self.exploration_wps[
                    self.exploration_wp_idx
                ]
                label = f'EXPLORING_WP_{self.exploration_wp_idx + 1}'

        # Navigate towards goal using APF
        lin, ang, reached, nav_state, nav_debug = (
            self.navigator.compute_apf_cmd_vel(
                self.map_x, self.map_y, self.map_yaw,
                goal_x, goal_y,
                self.scan_ranges, self.angle_min, self.angle_inc,
            )
        )

        full_state = f'{label}|{nav_state}'
        self._maybe_log_state(full_state, nav_debug, goal_x, goal_y, lin, ang)
        self._append_csv(goal_x, goal_y, lin, ang, full_state, nav_debug)

        if reached:
            if self.exploration_returning:
                # Arrived at Punt Base — save map and go to Phase III
                self.get_logger().info(
                    '[PHASE II] Arrived at Punt Base — saving map'
                )
                self.save_map()

                if self.perception.station_found:
                    self.get_logger().info(
                        '[PHASE II] Complete — '
                        'Transitioning to Phase III (Docking)'
                    )
                    self.phase = self.PHASE_III
                else:
                    self.get_logger().warn(
                        '[PHASE II] Complete — Station NOT found. '
                        'Mission cannot continue to docking.'
                    )
                    self.phase = self.PHASE_DONE
            else:
                self.get_logger().info(
                    f'[EXPLORE] Reached exploration wp '
                    f'{self.exploration_wp_idx + 1}'
                )
                self.exploration_wp_idx += 1
        else:
            self.publish_vel(lin, ang)

    # ==================================================================
    # Phase III — Precision Docking
    # ==================================================================

    def execute_phase_3(self):
        if not self.perception.station_found:
            self.get_logger().error(
                '[PHASE III] No station detected — aborting'
            )
            self.phase = self.PHASE_DONE
            return

        goal_x, goal_y = self.perception.station_center
        dx = goal_x - self.map_x
        dy = goal_y - self.map_y
        dist = math.hypot(dx, dy)

        # Docking complete
        if dist < 0.05:
            self.stop_robot()
            self.get_logger().info(
                '[PHASE III] DOCKING COMPLETE — '
                f'Final pose: ({self.map_x:.3f}, {self.map_y:.3f})'
            )
            self.phase = self.PHASE_DONE
            nav_state = 'DOCKED'
            nav_debug = {'distance': dist, 'angle_diff': 0.0,
                         'obstacle_detected': False}
            self._append_csv(goal_x, goal_y, 0.0, 0.0, nav_state, nav_debug)
            return

        target_angle = math.atan2(dy, dx)
        angle_diff = normalize_angle(target_angle - self.map_yaw)

        if dist > self.docking_approach_dist:
            # Approach using APF (still avoids pillars)
            lin, ang, reached, nav_state, nav_debug = (
                self.navigator.compute_apf_cmd_vel(
                    self.map_x, self.map_y, self.map_yaw,
                    goal_x, goal_y,
                    self.scan_ranges, self.angle_min, self.angle_inc,
                )
            )
            nav_state = f'DOCKING_APPROACH|{nav_state}'
        else:
            # Precision phase: slow, no APF, direct control
            nav_state = 'DOCKING_PRECISION'

            # First align, then advance
            if abs(angle_diff) > 0.15:
                lin = 0.0
                ang = 0.6 * angle_diff
                nav_state = 'DOCKING_ALIGNING'
            else:
                lin = min(0.04, 0.12 * dist)   # very slow
                ang = 0.5 * angle_diff

            lin = max(-0.05, min(0.05, lin))
            ang = max(-0.40, min(0.40, ang))

            nav_debug = {
                'distance': dist,
                'angle_diff': angle_diff,
                'obstacle_detected': False,
            }

        self._maybe_log_state(
            nav_state, nav_debug, goal_x, goal_y, lin, ang
        )
        self._append_csv(goal_x, goal_y, lin, ang, nav_state, nav_debug)
        self.publish_vel(lin, ang)

    # ==================================================================
    # Map saving
    # ==================================================================

    def save_map(self):
        """Save the SLAM-generated map using map_saver_cli."""
        map_path = os.path.join(os.getcwd(), 'generated_map')
        try:
            subprocess.Popen([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path,
                '--ros-args', '-p', 'save_map_timeout:=5000',
            ])
            self.get_logger().info(f'[MAP] Saving map to {map_path}')
        except Exception as e:
            self.get_logger().error(f'[MAP] Failed to save map: {e}')

    # ==================================================================
    # Shutdown
    # ==================================================================

    def shutdown(self):
        self.stop_robot()
        self.get_logger().info(
            f'[SHUTDOWN] Final pose: ({self.map_x:.3f}, '
            f'{self.map_y:.3f}, {self.map_yaw:.3f})'
        )
        self.get_logger().info(
            f'[SHUTDOWN] Station found: {self.perception.station_found}'
        )
        if self.perception.station_found:
            self.get_logger().info(
                f'[SHUTDOWN] Station center: '
                f'{self.perception.station_center}'
            )


# ──────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
