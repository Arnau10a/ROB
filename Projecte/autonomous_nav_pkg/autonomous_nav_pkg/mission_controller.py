import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import csv
import os

from autonomous_nav_pkg.navigation import Navigator


def obtener_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller_node')

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriber_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscriber_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)

        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.navigator = Navigator()

        # Odom state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_ready = False

        # Initial pose (D)
        self.initial_x = 3.32
        self.initial_y = 0.95
        self.initial_yaw = 0.0

        self.map_x = self.initial_x
        self.map_y = self.initial_y
        self.map_yaw = self.initial_yaw

        # Scan state
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.scan_ready = False

        # Mission D -> B -> O
        self.phase = "I"
        self.waypoints_phase_1 = [
            (3.72, 2.55),   # B
            (5.10, 12.61),  # O
        ]
        self.current_wp_idx = 0

        # CSV log
        self.log_file_path = os.path.join(os.getcwd(), 'mission_log.csv')
        self.init_logger()

        # Console throttling
        self._last_state = None
        self._last_state_log_time_ns = 0
        self._state_log_period_ns = int(0.5e9)
        self._heartbeat_period_ns = int(2.0e9)
        self._last_heartbeat_ns = 0

        # Diagnostics counters
        self._odom_count = 0
        self._scan_count = 0
        self._last_diag_ns = 0
        self._diag_period_ns = int(2.0e9)

        self.get_logger().info('Mission Controller Started')
        self.get_logger().info(f'Initial pose set to D: x={self.initial_x:.2f}, y={self.initial_y:.2f}, yaw={self.initial_yaw:.2f}')
        self.get_logger().info(f'Route loaded: {self.waypoints_phase_1}')

    def init_logger(self):
        with open(self.log_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'timestamp_sec',
                'phase',
                'wp_index',
                'goal_x',
                'goal_y',
                'robot_x',
                'robot_y',
                'robot_yaw',
                'linear_cmd',
                'angular_cmd',
                'nav_state',
                'distance_to_goal',
                'angle_diff',
                'obstacle_detected',
                'min_front',
                'min_left',
                'min_right',
                'nearest_all',
                'repulsive_mag',
                'danger_now',
                'avoid_mode',
                'danger_count',
                'clear_count'
            ])

    def append_log(self, goal_x, goal_y, lin, ang, nav_state, nav_debug):
        with open(self.log_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.get_clock().now().to_msg().sec,
                self.phase,
                self.current_wp_idx,
                goal_x,
                goal_y,
                self.map_x,
                self.map_y,
                self.map_yaw,
                lin,
                ang,
                nav_state,
                nav_debug.get('distance', None),
                nav_debug.get('angle_diff', None),
                nav_debug.get('obstacle_detected', None),
                nav_debug.get('min_front', None),
                nav_debug.get('min_left', None),
                nav_debug.get('min_right', None),
                nav_debug.get('nearest_all', None),
                nav_debug.get('repulsive_mag', None),
                nav_debug.get('danger_now', None),
                nav_debug.get('avoid_mode', None),
                nav_debug.get('danger_count', None),
                nav_debug.get('clear_count', None),
            ])

    @staticmethod
    def _fmt2(val):
        if val is None:
            return "nan"
        try:
            return f"{float(val):.2f}"
        except Exception:
            return "nan"

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = obtener_yaw(msg.pose.pose.orientation)

        self.map_x = self.initial_x + self.odom_x
        self.map_y = self.initial_y + self.odom_y
        self.map_yaw = self.initial_yaw + self.odom_yaw
        self.odom_ready = True
        self._odom_count += 1

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.scan_ready = True
        self._scan_count += 1

    def publish_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher_cmd.publish(msg)

    def stop_robot(self):
        self.publish_vel(0.0, 0.0)

    def _maybe_log_state(self, nav_state, nav_debug, goal_x, goal_y, lin, ang):
        now_ns = self.get_clock().now().nanoseconds
        state_changed = (nav_state != self._last_state)
        periodic = (now_ns - self._last_state_log_time_ns) >= self._state_log_period_ns

        if state_changed or periodic:
            self.get_logger().info(
                f"[NAV] state={nav_state} "
                f"wp={self.current_wp_idx + 1}/{len(self.waypoints_phase_1)} "
                f"goal=({goal_x:.2f},{goal_y:.2f}) "
                f"pos=({self.map_x:.2f},{self.map_y:.2f}) "
                f"d={self._fmt2(nav_debug.get('distance', None))} "
                f"ang={self._fmt2(nav_debug.get('angle_diff', None))} "
                f"obs={nav_debug.get('obstacle_detected', False)} "
                f"front={self._fmt2(nav_debug.get('min_front', None))} "
                f"rep={self._fmt2(nav_debug.get('repulsive_mag', None))} "
                f"cmd=({lin:.2f},{ang:.2f})"
            )
            self._last_state = nav_state
            self._last_state_log_time_ns = now_ns

        if (now_ns - self._last_heartbeat_ns) >= self._heartbeat_period_ns:
            self.get_logger().info(
                f"[HEARTBEAT] phase={self.phase} wp_idx={self.current_wp_idx} "
                f"pose=({self.map_x:.2f},{self.map_y:.2f},{self.map_yaw:.2f})"
            )
            self._last_heartbeat_ns = now_ns

    def _diagnostics(self):
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_diag_ns) < self._diag_period_ns:
            return

        cmd_subs = self.count_subscribers('/cmd_vel')
        odom_pubs = self.count_publishers('/odom')
        scan_pubs = self.count_publishers('/scan')

        self.get_logger().info(
            f"[DIAG] cmd_vel_subs={cmd_subs} odom_pubs={odom_pubs} scan_pubs={scan_pubs} "
            f"odom_msgs={self._odom_count} scan_msgs={self._scan_count}"
        )

        self._odom_count = 0
        self._scan_count = 0
        self._last_diag_ns = now_ns

    def timer_callback(self):
        self._diagnostics()

        if not self.odom_ready or not self.scan_ready:
            return

        if self.phase == "DONE":
            self.stop_robot()
            return

        self.execute_phase_1()

    def execute_phase_1(self):
        if self.current_wp_idx >= len(self.waypoints_phase_1):
            self.phase = "DONE"
            self.stop_robot()
            self.get_logger().info('[MISSION] Already complete.')
            return

        goal_x, goal_y = self.waypoints_phase_1[self.current_wp_idx]

        lin, ang, reached, nav_state, nav_debug = self.navigator.compute_apf_cmd_vel(
            self.map_x, self.map_y, self.map_yaw,
            goal_x, goal_y,
            self.scan_ranges, self.angle_min, self.angle_inc
        )

        self._maybe_log_state(nav_state, nav_debug, goal_x, goal_y, lin, ang)
        self.append_log(goal_x, goal_y, lin, ang, nav_state, nav_debug)

        if reached:
            self.get_logger().info(f"[WAYPOINT] Reached wp {self.current_wp_idx + 1}: ({goal_x:.2f}, {goal_y:.2f})")
            self.current_wp_idx += 1

            if self.current_wp_idx >= len(self.waypoints_phase_1):
                self.phase = "DONE"
                self.stop_robot()
                self.get_logger().info('[MISSION] Ruta D -> B -> O completada.')
            else:
                nx, ny = self.waypoints_phase_1[self.current_wp_idx]
                self.get_logger().info(f"[WAYPOINT] Next target wp {self.current_wp_idx + 1}: ({nx:.2f}, {ny:.2f})")
        else:
            self.publish_vel(lin, ang)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
