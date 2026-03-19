import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math
from geometry_msgs.msg import TwistStamped

class SquareTrajectory(Node):
    def __init__(self):
        super().__init__('square_node')
        qos_profile_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile_r)
        
        self.timer_period = 0.1  
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 0: Avanzar, 1: Girar
        self.state = 0 
        self.lados_completados = 0
        
        self.linear_speed = 0.2    
        self.angular_speed = 0.5   
        
        self.target_distance = 1.0 
        self.target_angle = math.pi / 2 
        
        self.forward_time = self.target_distance / self.linear_speed
        self.turn_time = self.target_angle / self.angular_speed
        
        self.start_time = self.get_clock().now()
        self.get_logger().info('Iniciando cuadrado...')

    def timer_callback(self):
        if self.lados_completados >= 4:
            self.stop_robot()
            if self.timer is not None:
                self.get_logger().info('¡Hecho!')
                self.timer.cancel()
                self.timer = None
            return

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        move_msg = TwistStamped()
        move_msg.header.stamp = self.get_clock().now().to_msg()
        move_msg.header.frame_id = ''
        
        if self.state == 0: # ir recto
            if elapsed_time < self.forward_time:
                move_msg.twist.linear.x = self.linear_speed
                move_msg.twist.angular.z = 0.0
            else:
                self.state = 1 
                self.start_time = self.get_clock().now()
                
        elif self.state == 1: # girar 90 grados
            if elapsed_time < self.turn_time:
                move_msg.twist.linear.x = 0.0
                move_msg.twist.angular.z = self.angular_speed
            else:
                self.state = 0 
                self.lados_completados += 1
                self.start_time = self.get_clock().now()
                
        self.publisher.publish(move_msg)

    def stop_robot(self):
        self.publisher.publish(TwistStamped())

def main(args=None):
    rclpy.init(args=args)
    node = SquareTrajectory()
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
