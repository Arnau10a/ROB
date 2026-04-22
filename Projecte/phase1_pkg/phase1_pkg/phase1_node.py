import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

from project_core_pkg.navigation import Navigator

# Waypoints para la Fase I
PUNT_B = (3.72, 2.55)
PORTA = (5.92, 8.12)
PUNT_O = (5.10, 12.61)

def yaw_from_quaternion(q):
    """Extract yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Phase1Node(Node):
    def __init__(self):
        super().__init__('phase1_node')
        
        # Parameters
        self.declare_parameter('use_sim_time', False)

        # QoS
        qos_best_effort = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_best_effort)
        
        # Publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF for SLAM-based localisation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_ready = False

        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Sub-modules
        self.navigator = Navigator()

        # Odometry state (fallback)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_ready = False

        # Initial pose (D) - Configurable
        self.initial_x = 3.32
        self.initial_y = 0.95
        self.initial_yaw = 0.0

        # Current pose in map frame
        self.map_x = self.initial_x
        self.map_y = self.initial_y
        self.map_yaw = self.initial_yaw

        # Scan state
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.scan_ready = False

        # Phase I Waypoints: D -> B -> Porta -> O
        self.waypoints = [PUNT_B, PORTA, PUNT_O]
        self.current_wp_idx = 0
        self.mission_completed = False

        self.get_logger().info('Phase 1 Node initialized. Waypoints: D -> B -> O')

    def update_pose_from_tf(self):
        """
        Try to get the pose from SLAM TF (map -> base_link).
        Falls back to dead-reckoning from odometry.
        """
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
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

    def timer_callback(self):
        self.update_pose_from_tf()

        if not self.odom_ready or not self.scan_ready:
            return

        if self.mission_completed:
            self.stop_robot()
            return

        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info('PHASE 1 COMPLETE! Reached destination.')
            self.mission_completed = True
            return

        goal_x, goal_y = self.waypoints[self.current_wp_idx]

        lin, ang, reached, nav_state, nav_debug = self.navigator.compute_apf_cmd_vel(
            self.map_x, self.map_y, self.map_yaw,
            goal_x, goal_y,
            self.scan_ranges, self.angle_min, self.angle_inc,
        )

        if reached:
            self.get_logger().info(f'WAYPOINT REACHED: ({goal_x:.2f}, {goal_y:.2f})')
            self.current_wp_idx += 1
        else:
            self.publish_vel(lin, ang)

def main(args=None):
    rclpy.init(args=args)
    node = Phase1Node()
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
