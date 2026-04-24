import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from project_core_pkg.mission_logger import MissionLogger

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Phase1ProNode(Node):
    def __init__(self):
        super().__init__('phase1_pro_node')
        
        # ==========================================
        # 1. PARAMETERS (Instead of hardcoded values)
        # ==========================================
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('waypoints_x', [3.72, 5.92, 5.10])
        self.declare_parameter('waypoints_y', [2.55, 8.12, 12.61])
        self.declare_parameter('linear_vel', 0.20)
        self.declare_parameter('angular_vel', 0.60)
        self.declare_parameter('waypoint_tolerance', 0.20)
        self.declare_parameter('obstacle_threshold', 0.35)
        self.declare_parameter('clear_threshold', 0.60)
        self.declare_parameter('recovery_time', 2.0)

        # Read parameters
        wps_x = self.get_parameter('waypoints_x').value
        wps_y = self.get_parameter('waypoints_y').value
        self.waypoints = list(zip(wps_x, wps_y))
        
        self.p_lin_vel = self.get_parameter('linear_vel').value
        self.p_ang_vel = self.get_parameter('angular_vel').value
        self.p_tolerance = self.get_parameter('waypoint_tolerance').value
        self.p_obs_thresh = self.get_parameter('obstacle_threshold').value
        self.p_clr_thresh = self.get_parameter('clear_threshold').value
        self.p_rec_time = self.get_parameter('recovery_time').value

        # QoS
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_best_effort)
        
        # Publishers
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        # 2. DEBUG TOPIC
        self.pub_state = self.create_publisher(String, '/phase1/state', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Pose State
        self.odom_x, self.odom_y, self.odom_yaw = 0.0, 0.0, 0.0
        self.odom_ready = False
        self.map_x, self.map_y, self.map_yaw = 3.32, 0.95, 0.0

        # Scan State
        self.scan_ranges = []
        self.angle_min, self.angle_inc = 0.0, 0.0
        self.scan_ready = False

        # Mission State
        self.current_wp_idx = 0
        self.mission_completed = False
        self.logger = MissionLogger()
        self.log_count = 0

        # State Machine
        self.state = "TURN_TO_GOAL"
        self.avoid_turn_dir = 1.0
        self.recovery_timer = 0.0
        
        self.get_logger().info('Phase 1 Pro Node initialized with Parameters and Recovery State.')

    def update_pose_from_tf(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.map_x = t.transform.translation.x
            self.map_y = t.transform.translation.y
            self.map_yaw = yaw_from_quaternion(t.transform.rotation)
        except TransformException:
            if self.odom_ready:
                self.map_x = 3.32 + self.odom_x
                self.map_y = 0.95 + self.odom_y
                self.map_yaw = 0.0 + self.odom_yaw

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

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def get_clearances(self):
        min_front = float('inf')
        min_left = float('inf')
        min_right = float('inf')
        
        for i, r in enumerate(self.scan_ranges):
            if r is None or not math.isfinite(r) or r <= 0.10: continue
            if r > 10.0: continue
            
            a = self.normalize_angle(self.angle_min + i * self.angle_inc)
            deg = math.degrees(a)
            
            if -30.0 <= deg <= 30.0:
                min_front = min(min_front, r)
            elif 30.0 < deg <= 90.0:
                min_left = min(min_left, r)
            elif -90.0 <= deg < -30.0:
                min_right = min(min_right, r)
                
        return min_front, min_left, min_right

    def timer_callback(self):
        self.update_pose_from_tf()
        if not self.odom_ready or not self.scan_ready: return

        # 3. Publish Telemetry
        self.publish_state()

        if self.mission_completed:
            self.publish_vel(0.0, 0.0)
            return

        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info('PHASE 1 COMPLETE!')
            self.mission_completed = True
            return

        goal_x, goal_y = self.waypoints[self.current_wp_idx]

        self.log_count += 1
        if self.log_count % 20 == 0:
            self.logger.log("I", self.map_x, self.map_y, self.map_yaw)

        dx = goal_x - self.map_x
        dy = goal_y - self.map_y
        dist = math.hypot(dx, dy)

        if dist < self.p_tolerance:
            self.get_logger().info(f'WAYPOINT REACHED: ({goal_x:.2f}, {goal_y:.2f})')
            self.current_wp_idx += 1
            self.state = "TURN_TO_GOAL"
            return

        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - self.map_yaw)

        front_d, left_d, right_d = self.get_clearances()

        # ==========================================
        # 4. RECOVERY BEHAVIOR
        # ==========================================
        if self.state != "RECOVERY_REVERSE":
            # If all sides are blocked...
            if front_d < self.p_obs_thresh and left_d < self.p_obs_thresh and right_d < self.p_obs_thresh:
                self.get_logger().warn("STUCK! Initiating recovery reverse.")
                self.state = "RECOVERY_REVERSE"
                self.recovery_timer = self.p_rec_time

        # STATE MACHINE LOGIC
        if self.state == "RECOVERY_REVERSE":
            self.recovery_timer -= self.timer_period
            if self.recovery_timer <= 0:
                self.state = "TURN_TO_GOAL" # Try again
            else:
                self.publish_vel(-0.15, 0.0) # Back up blindly

        elif self.state == "AVOID_OBSTACLE":
            if front_d > self.p_clr_thresh:
                self.state = "TURN_TO_GOAL"
            else:
                self.publish_vel(0.0, self.p_ang_vel * self.avoid_turn_dir)
                
        elif self.state == "TURN_TO_GOAL":
            if front_d < self.p_obs_thresh:
                self.state = "AVOID_OBSTACLE"
                self.avoid_turn_dir = 1.0 if right_d < left_d else -1.0
            elif abs(angle_diff) > 0.15:
                turn_speed = self.p_ang_vel if angle_diff > 0 else -self.p_ang_vel
                self.publish_vel(0.0, turn_speed)
            else:
                self.state = "MOVE_FORWARD"
                
        elif self.state == "MOVE_FORWARD":
            if front_d < self.p_obs_thresh:
                self.state = "AVOID_OBSTACLE"
                self.avoid_turn_dir = 1.0 if right_d < left_d else -1.0
            elif abs(angle_diff) > 0.35:
                self.state = "TURN_TO_GOAL"
            else:
                self.publish_vel(self.p_lin_vel, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = Phase1ProNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.logger.save_map(node, "phase1_pro")
        node.publish_vel(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
