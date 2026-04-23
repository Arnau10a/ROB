import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

from project_core_pkg.navigation import Navigator

# Referencias de puntos del Mapa
PUNT_A = (4.280, 1.735)
PUNT_B = (5.280, 1.735)
PUNT_C = (4.880, 2.535)
PUNT_D = (5.080, 5.740)
PUNT_E = (5.880, 8.145)
PUNT_F = (5.480, 10.545)
PORTA  = (6.280, 11.685)
P_BASE = (3.475, 15.390)
PUNT_Q = (9.115, 14.190)
PUNT_R = (7.310, 16.190)
PUNT_S = (3.675, 14.190)
PUNT_T = (1.275, 14.990)
PUNT_U = (1.075, 16.190)

# Waypoints seleccionados para la Fase I
PUNT_MIG_VESTIBOL = (7.380, 11.685)

def yaw_from_quaternion(q):
    """Extract yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Phase1Node(Node):
    def __init__(self):
        super().__init__('phase1_node')
        
        # Parameters
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # QoS
        qos_best_effort = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_best_effort)
        
        # Publishers
        self.pub_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/target_path', 10)

        # TF for SLAM-based localisation
        try:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info("TF Listener started.")
        except Exception as e:
            self.get_logger().error(f"Failed to start TF Listener: {e}")
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

        # Initial pose (E) - Configurable
        self.initial_x = PUNT_E[0]
        self.initial_y = PUNT_E[1]
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

        # Phase I Waypoints: E -> F
        self.waypoints = [PUNT_F, PORTA, PUNT_MIG_VESTIBOL, PUNT_Q, P_BASE]
        self.current_wp_idx = 0
        self.mission_completed = False

        self.get_logger().info(f'Phase 1 Node initialized. Waypoints: {len(self.waypoints)} targets loaded.')

        # Pre-create the path message for RViz
        self.target_path_msg = Path()
        self.target_path_msg.header.frame_id = 'map'
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            self.target_path_msg.poses.append(pose)

        # Start mechanism
        self.started = False
        self.start_thread = threading.Thread(target=self.wait_for_enter)
        self.start_thread.daemon = True
        self.start_thread.start()

    def wait_for_enter(self):
        input("\n>>> PRESIONA ENTER PARA EMPEZAR LA MISIÓN <<<\n")
        self.started = True
        self.get_logger().info("MISIÓN INICIADA!")

    def update_pose_from_tf(self):
        """
        Pure odometry dead-reckoning (SLAM disabled).
        """
        if self.odom_ready:
            self.map_x = self.initial_x + self.odom_x * math.cos(self.initial_yaw) - self.odom_y * math.sin(self.initial_yaw)
            self.map_y = self.initial_y + self.odom_x * math.sin(self.initial_yaw) + self.odom_y * math.cos(self.initial_yaw)
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
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.pub_cmd.publish(msg)


    def stop_robot(self):
        if rclpy.ok():
            self.publish_vel(0.0, 0.0)

    def timer_callback(self):
        # Publish target path every cycle for RViz
        self.target_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_path.publish(self.target_path_msg)

        try:
            self.update_pose_from_tf()
        except Exception as e:
            self.get_logger().error(f"Error in update_pose_from_tf: {e}")

        if not self.odom_ready or not self.scan_ready:
            return

        if not self.started:
            # Publish path so it's visible in RViz while waiting
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

        # Log de depuración cada ~2 segundos
        now = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, '_last_nav_log') or (now - self._last_nav_log >= 2.0):
            dist = math.hypot(goal_x - self.map_x, goal_y - self.map_y)
            self.get_logger().info(
                f"Target WP[{self.current_wp_idx+1}/{len(self.waypoints)}]: ({goal_x:.2f}, {goal_y:.2f}) "
                f"| Pose: ({self.map_x:.2f}, {self.map_y:.2f}) "
                f"| Estado: {nav_state} | Dist: {dist:.2f}m"
            )
            self._last_nav_log = now

        if reached:
            self.get_logger().info(f'>>> WAYPOINT REACHED [{self.current_wp_idx + 1}/{len(self.waypoints)}]: ({goal_x:.2f}, {goal_y:.2f})')
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
