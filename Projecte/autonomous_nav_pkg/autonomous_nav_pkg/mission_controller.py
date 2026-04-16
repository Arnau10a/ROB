import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import csv
import time
import os
import subprocess

from autonomous_nav_pkg.navigation import Navigator
from autonomous_nav_pkg.perception import Perception

def obtener_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller_node')
        
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriber_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscriber_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile)
        
        # NOTE: Your past code used TwistStamped for cmd_vel. 
        # Uncomment the one that your environment actually uses.
        self.publisher_cmd = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        # self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 Hz
        
        self.navigator = Navigator()
        self.perception = Perception()
        
        # Robot state (Odometry)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_ready = False
        
        # --- CONFIGURACIÓ INICIAL (LABORATORI) ---
        # Si el profe et posa al Punt A, posa: 2.52, 1.35
        # Si et posa al Punt C, posa: 1.32, 0.95
        # ORIENTACIÓ: El robot ha de mirar cap a la dreta del mapa (Eix X+)
        self.initial_x = 2.52 
        self.initial_y = 1.35
        self.initial_yaw = 0.0 # rad
        # -----------------------------------------
        
        # Absolute Map Coordinates (Initial + Odom)
        self.map_x = self.initial_x
        self.map_y = self.initial_y
        self.map_yaw = self.initial_yaw
        
        self.scan_ready = False
        
        # Scan data
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_inc = 0.0
        
        # Mission flow variables
        self.phase = "I" # "I", "II", "III", "DONE"
        
        # Waypoints Phase I
        self.waypoints_phase_1 = [
            (3.72, 2.55), # Waypoint B 
            (5.10, 12.61), # Waypoint O
            (5.00, 11.69) # Destination Zone 2 (P Base)
        ]
        self.current_wp_idx = 0
        
        # Phase 2 variables
        self.station_center = None
        self.station_pillars = []
        # simple sweeping path to scan Passadis area from P Base
        self.exploration_wps = [
            (5.5, 11.0), (4.5, 10.5), (6.0, 12.0), (3.5, 12.5), (5.0, 11.69) # Back to base
        ]
        self.expl_idx = 0
        
        # Logging
        self.log_file_path = os.path.join(os.getcwd(), 'mission_log.csv')
        self.init_logger()

        self.get_logger().info('Mission Controller Started - Initializing...')
        
        # NOTE: Removed basic obstacle avoidance state variables in favor of robust APF.

    def init_logger(self):
        with open(self.log_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'current_phase', 'robot_x', 'robot_y', 'robot_yaw', 'station_x', 'station_y'])

    def log_status(self):
        sx = self.station_center[0] if self.station_center else -1.0
        sy = self.station_center[1] if self.station_center else -1.0
        
        with open(self.log_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.get_clock().now().to_msg().sec, self.phase, self.map_x, self.map_y, self.map_yaw, sx, sy])

    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = obtener_yaw(msg.pose.pose.orientation)
        
        # Transform odometry to absolute map frame
        # (Assuming robot starts aligned with map X axis)
        self.map_x = self.initial_x + self.odom_x
        self.map_y = self.initial_y + self.odom_y
        self.map_yaw = self.initial_yaw + self.odom_yaw
        
        self.odom_ready = True

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment
        self.scan_ready = True

    def publish_vel(self, linear, angular):
        # NOTE: Publishing using TwistStamped. Change to Twist if needed.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.publisher_cmd.publish(msg)
        
    def stop_robot(self):
        self.publish_vel(0.0, 0.0)

    def timer_callback(self):
        if not self.odom_ready or not self.scan_ready:
            return
            
        if self.phase == "DONE":
            self.stop_robot()
            return
            
        # Log periodically
        if (self.get_clock().now().nanoseconds % 1000000000) < 50000000: # ~1 Hz logging
            self.log_status()

        # State Machine
        if self.phase == "I":
            self.execute_phase_1()
        elif self.phase == "II":
            self.execute_phase_2()
        elif self.phase == "III":
            self.execute_phase_3()


    def execute_phase_1(self):
        # Follow path B -> O -> P base
        goal_x, goal_y = self.waypoints_phase_1[self.current_wp_idx]
        lin, ang, reached = self.navigator.compute_apf_cmd_vel(
            self.map_x, self.map_y, self.map_yaw, goal_x, goal_y,
            self.scan_ranges, self.angle_min, self.angle_inc
        )
        
        if reached:
            self.get_logger().info(f'Waypoint {self.current_wp_idx + 1} reached!')
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.waypoints_phase_1):
                self.phase = "II"
                self.get_logger().info('Phase I Complete! Starting Phase II (Exploration).')
        else:
            self.publish_vel(lin, ang)
            
    def execute_phase_2(self):
        # Look for charging station while exploring passadis
        center, pillars = self.perception.find_charging_station(
            self.scan_ranges, self.angle_min, self.angle_inc, self.map_x, self.map_y, self.map_yaw
        )
        
        if center is not None:
            self.station_center = center
            self.station_pillars = pillars
            self.get_logger().info(f'Charging Station Found at: {center}!')
            self.get_logger().info('Returning to P Base...')
            # Overwrite exploration with return to base
            self.exploration_wps = [(5.00, 11.69)]
            self.expl_idx = 0
            
        # Follow exploration wps
        if self.expl_idx < len(self.exploration_wps):
            goal_x, goal_y = self.exploration_wps[self.expl_idx]
            lin, ang, reached = self.navigator.compute_apf_cmd_vel(
                self.map_x, self.map_y, self.map_yaw, goal_x, goal_y,
                self.scan_ranges, self.angle_min, self.angle_inc
            )
            if reached:
                self.expl_idx += 1
                if self.station_center is not None and self.expl_idx >= len(self.exploration_wps):
                    self.phase = "III"
                    self.get_logger().info('Phase II Complete! Starting Phase III (Precision Docking).')
            else:
                self.publish_vel(lin, ang)
        else:
            # Reached end of exploration but no station found
            self.get_logger().warn('Completed exploration but stations not found. Sweeping again...')
            self.publish_vel(0.0, 0.4) # Spin to look around

    def execute_phase_3(self):
        # Precise docking
        goal_x, goal_y = self.station_center
        # Reduce speeds for precision
        self.navigator.max_linear = 0.05
        self.navigator.max_angular = 0.2
        self.navigator.waypoint_tolerance = 0.02 # High precision within 2cm
        
        lin, ang, reached = self.navigator.compute_cmd_vel(self.map_x, self.map_y, self.map_yaw, goal_x, goal_y)
        if reached:
            self.get_logger().info('Precision Docking Complete! MISSION ACCOMPLISHED.')
            self.stop_robot()
            self.phase = "DONE"
            
            # Save the SLAM Map automatically
            self.get_logger().info('Executing command to save the map: map_final.yaml / .pgm')
            try:
                subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', 'map_final'])
                self.get_logger().info('Map saved successfully in the current directory.')
            except Exception as e:
                self.get_logger().error(f'Failed to save map: {e}')
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
