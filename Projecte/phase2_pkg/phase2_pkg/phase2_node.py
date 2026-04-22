import math
import os
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

from project_core_pkg.navigation import Navigator
from project_core_pkg.perception import Perception

# Reference points
PUNT_BASE = (5.00, 11.69)

# Passadís exploration area
PASSADIS_X_MIN = 0.5
PASSADIS_X_MAX = 7.0
PASSADIS_Y_MIN = 10.5
PASSADIS_Y_MAX = 13.5

# Lawnmower exploration step
EXPLORATION_STEP_X = 1.5
EXPLORATION_STEP_Y = 2.0

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

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Phase2Node(Node):
    def __init__(self):
        super().__init__('phase2_node')
        
        self.declare_parameter('use_sim_time', False)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_best_effort)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_ready = False

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.navigator = Navigator()
        self.perception = Perception()

        self.odom_x, self.odom_y, self.odom_yaw = 0.0, 0.0, 0.0
        self.odom_ready = False

        self.map_x, self.map_y, self.map_yaw = 0.0, 0.0, 0.0

        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.scan_ready = False

        self.exploration_wps = generate_exploration_waypoints()
        self.exploration_wp_idx = 0
        self.exploration_returning = False
        self.phase_completed = False

        self.get_logger().info('Phase 2 Node initialized: Exploration & Perception')

    def update_pose_from_tf(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.map_x = t.transform.translation.x
            self.map_y = t.transform.translation.y
            self.map_yaw = yaw_from_quaternion(t.transform.rotation)
            self.tf_ready = True
        except TransformException:
            if self.odom_ready:
                self.map_x = self.odom_x
                self.map_y = self.odom_y
                self.map_yaw = self.odom_yaw

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ready = True

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.scan_ready = True

    def publish_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_cmd.publish(msg)

    def stop_robot(self):
        self.publish_vel(0.0, 0.0)

    def save_map(self):
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

    def timer_callback(self):
        self.update_pose_from_tf()
        if not self.odom_ready or not self.scan_ready or self.phase_completed:
            return

        # Detection
        self.perception.find_charging_station(
            self.scan_ranges, self.angle_min, self.angle_inc,
            self.map_x, self.map_y, self.map_yaw,
        )
        new_obs = self.perception.detect_obstacles(
            self.scan_ranges, self.angle_min, self.angle_inc,
            self.map_x, self.map_y, self.map_yaw,
        )
        if new_obs:
            for ox, oy in new_obs:
                self.get_logger().info(f'[OBSTACLE] Detected at ({ox:.2f}, {oy:.2f})')

        if self.perception.station_found and not self.exploration_returning:
            cx, cy = self.perception.station_center
            self.get_logger().info(f'[STATION] Charging station found at ({cx:.2f}, {cy:.2f})!')
            self.exploration_returning = True

        # Goal selection
        if self.exploration_returning:
            goal_x, goal_y = PUNT_BASE
        else:
            if self.exploration_wp_idx >= len(self.exploration_wps):
                self.get_logger().warn('[PHASE II] Exploration complete - station NOT found, returning to base')
                self.exploration_returning = True
                goal_x, goal_y = PUNT_BASE
            else:
                goal_x, goal_y = self.exploration_wps[self.exploration_wp_idx]

        # Navigation
        lin, ang, reached, _, _ = self.navigator.compute_apf_cmd_vel(
            self.map_x, self.map_y, self.map_yaw,
            goal_x, goal_y,
            self.scan_ranges, self.angle_min, self.angle_inc,
        )

        if reached:
            if self.exploration_returning:
                self.get_logger().info('[PHASE II] Arrived at Punt Base - saving map')
                self.save_map()
                self.phase_completed = True
                self.stop_robot()
            else:
                self.get_logger().info(f'[EXPLORE] Reached waypoint {self.exploration_wp_idx + 1}')
                self.exploration_wp_idx += 1
        else:
            self.publish_vel(lin, ang)

def main(args=None):
    rclpy.init(args=args)
    node = Phase2Node()
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
