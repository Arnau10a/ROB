import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SeguidorReflectante(Node):
    def __init__(self):
        super().__init__('seguidor_reflectante_node')
        self.publicador = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Creamos el perfil QoS para que coincida con el LiDAR del TurtleBot
        perfil_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Añadimos el perfil_qos a la suscripción
        self.suscriptor = self.create_subscription(LaserScan, '/scan', self.scan_callback, perfil_qos)
        
        self.distancia_seguridad = 0.5
        self.umbral_intensidad = 10000.0
        self.get_logger().info('Nodo seguidor de reflectante iniciado. Buscando objetivo...')

    def scan_callback(self, msg):
        if not msg.intensities:
            self.get_logger().warning('El sensor no reporta datos de intensidad.')
            return
            
        mejor_intensidad = 0.0
        indice_max = -1
        
        # Buscamos la máxima intensidad, pero SOLO en distancias que sean válidas
        for i in range(len(msg.intensities)):
            dist = msg.ranges[i]
            # Filtramos errores (0.0), infinitos y fuera de rango útil
            if not math.isinf(dist) and not math.isnan(dist) and dist > 0.12 and dist < 3.5:
                if msg.intensities[i] > mejor_intensidad:
                    mejor_intensidad = msg.intensities[i]
                    indice_max = i
        
        # Si no encontramos nada o la intensidad es menor al umbral, paramos
        if indice_max == -1 or mejor_intensidad < self.umbral_intensidad:
            self.parar_robot()
            return
            
        distancia_objetivo = msg.ranges[indice_max]
        
        # LOG MEJORADO: Nos chivará la intensidad real para calibrar el umbral
        self.get_logger().info(f'Target! Int: {mejor_intensidad:.0f} | Indice: {indice_max} | Dist: {distancia_objetivo:.2f}m')

        # Calculamos hacia dónde girar
        angulo_objetivo = msg.angle_min + (indice_max * msg.angle_increment)
        
        while angulo_objetivo > math.pi:
            angulo_objetivo -= 2 * math.pi
        while angulo_objetivo < -math.pi:
            angulo_objetivo += 2 * math.pi

        # Movimiento usando TwistStamped
        cmd = TwistStamped()
        
        cmd.twist.angular.z = 1.0 * angulo_objetivo
        
        error_distancia = distancia_objetivo - self.distancia_seguridad
        if error_distancia > 0.05:
            cmd.twist.linear.x = -0.5 * error_distancia
        elif error_distancia < -0.05:
            cmd.twist.linear.x = 0.5 * abs(error_distancia)
        else:
            cmd.twist.linear.x = 0.0

        # Límites de velocidad para que no sea brusco
        cmd.twist.linear.x = max(min(cmd.twist.linear.x, 0.2), -0.2)
        cmd.twist.angular.z = max(min(cmd.twist.angular.z, 0.5), -0.5)

        self.publicador.publish(cmd)

    def parar_robot(self):
        self.publicador.publish(TwistStamped())

def main(args=None):
    rclpy.init(args=args)
    node = SeguidorReflectante()
    rclpy.spin(node)
    node.parar_robot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
