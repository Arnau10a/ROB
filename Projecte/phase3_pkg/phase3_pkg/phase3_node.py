import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

from project_core_pkg.navigation import Navigator
from project_core_pkg.perception import Perception

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

class Phase3Node(Node):
    def __init__(self):
        super().__init__('phase3_node')
        
        self.declare_parameter('use_sim_time', False)
        # In a real scenario, these would be parameters passed from Phase II
        self.declare_parameter('station_x', 0.0)
        self.declare_parameter('station_y', 0.0)

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

        self.docking_approach_dist = 0.30
        self.phase_completed = False

        self.get_logger().info('Phase 3 Node initialized: Precision Docking')

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

    def timer_callback(self):
        self.update_pose_from_tf()
        if not self.odom_ready or not self.scan_ready or self.phase_completed:
            return

        # Redetect station to confirm position if not fixed
        self.perception.find_charging_station(
            self.scan_ranges, self.angle_min, self.angle_inc,
            self.map_x, self.map_y, self.map_yaw,
        )

        if not self.perception.station_found:
            # Try using parameters if perception fails
            goal_x = self.get_parameter('station_x').get_parameter_value().double_value
            goal_y = self.get_parameter('station_y').get_parameter_value().double_value
            if goal_x == 0.0 and goal_y == 0.0:
                self.get_logger().throttle_sequential(5.0, '[PHASE III] Waiting for station detection...')
                return
        else:
            goal_x, goal_y = self.perception.station_center

        dx = goal_x - self.map_x
        dy = goal_y - self.map_y
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            self.stop_robot()
            self.get_logger().info(f'[PHASE III] DOCKING COMPLETE - Final pose: ({self.map_x:.3f}, {self.map_y:.3f})')
            self.phase_completed = True
            return

        target_angle = math.atan2(dy, dx)
        angle_diff = normalize_angle(target_angle - self.map_yaw)

        if dist > self.docking_approach_dist:
            # Approach using APF
            lin, ang, reached, _, _ = self.navigator.compute_apf_cmd_vel(
                self.map_x, self.map_y, self.map_yaw,
                goal_x, goal_y,
                self.scan_ranges, self.angle_min, self.angle_inc,
            )
        else:
            # Precision phase
            if abs(angle_diff) > 0.15:
                lin = 0.0
                ang = 0.6 * angle_diff
            else:
                lin = min(0.04, 0.12 * dist)
                ang = 0.5 * angle_diff
            
            lin = max(-0.05, min(0.05, lin))
            ang = max(-0.40, min(0.40, ang))

        self.publish_vel(lin, ang)

def main(args=None):
    rclpy.init(args=args)
    node = Phase3Node()
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
