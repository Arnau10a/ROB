import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

# calcula el angulo Yaw
def obtener_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class CuadradoSimple(Node):
    def __init__(self):
        super().__init__('cuadrado_simple_node')
        self.publicador = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.suscriptor = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.033, self.timer_callback)

        #ajustes
        self.metros_lado = 0.5
        self.radianes_giro = (math.pi / 2) * 0.95
        self.vel_lineal = 0.2
        self.vel_angular = 0.5

        # 0=Recto, 1=Girar
        self.estado = 0 
        self.lados_hechos = 0
        self.odom_lista = False

        self.x_ini, self.y_ini, self.yaw_ini = 0.0, 0.0, 0.0
        self.x_act, self.y_act, self.yaw_act = 0.0, 0.0, 0.0

    def odom_callback(self, msg):
        self.x_act = msg.pose.pose.position.x
        self.y_act = msg.pose.pose.position.y
        self.yaw_act = obtener_yaw(msg.pose.pose.orientation)

        if not self.odom_lista:
            self.reiniciar_referencias()
            self.odom_lista = True

    def reiniciar_referencias(self):
        self.x_ini = self.x_act
        self.y_ini = self.y_act
        self.yaw_ini = self.yaw_act

    def timer_callback(self):
        if not self.odom_lista: return

        if self.lados_hechos >= 4:
            self.parar_robot()
            self.timer.cancel()
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        if self.estado == 0: # ir recto
            distancia = math.sqrt((self.x_act - self.x_ini)**2 + (self.y_act - self.y_ini)**2)
            if distancia < self.metros_lado:
                cmd.twist.linear.x = self.vel_lineal
            else:
                self.parar_robot()
                self.estado = 1 
                self.reiniciar_referencias()

        elif self.estado == 1: # girar
            diferencia_angulo = self.yaw_act - self.yaw_ini
            while diferencia_angulo > math.pi: diferencia_angulo -= 2*math.pi
            while diferencia_angulo < -math.pi: diferencia_angulo += 2*math.pi

            if abs(diferencia_angulo) < self.radianes_giro:
                cmd.twist.angular.z = self.vel_angular
            else:
                self.parar_robot()
                self.estado = 0
                self.lados_hechos += 1
                self.reiniciar_referencias()

        self.publicador.publish(cmd)

    def parar_robot(self):
        self.publicador.publish(TwistStamped())

def main(args=None):
    rclpy.init(args=args)
    node = CuadradoSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.parar_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
