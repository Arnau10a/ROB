import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan

class AnalisisLaser(Node):
    def __init__(self):
        super().__init__('analisis_laser_node')
        
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile)
        
        #guardaremos 10 scans para la media
        self.n_scans_objetivo = 10
        self.historial_scans = []
        self.analisis_terminado = False

        self.get_logger().info('Esperando 10 scans...')

    def laser_callback(self, msg):
        if self.analisis_terminado:
            return

        # guardar array de distancias
        self.historial_scans.append(msg.ranges)
        self.get_logger().info(f'Scan {len(self.historial_scans)} guardado')

        if len(self.historial_scans) == self.n_scans_objetivo:
            self.calcular_estadisticas()
            self.analisis_terminado = True

    def calcular_estadisticas(self):
        num_angulos = len(self.historial_scans[0])
        
        for angulo in range(num_angulos):
            # coger la misma posicion en los 10 scans
            mediciones_angulo = [scan[angulo] for scan in self.historial_scans]
            
            # quitar infinitos
            validas = [m for m in mediciones_angulo if m != float('inf') and m != float('-inf')]
            
            if len(validas) > 0:
                val_min = min(validas)
                val_max = max(validas)
                val_media = sum(validas) / len(validas)
                print(f"Angulo {angulo:03d}: Min={val_min:.3f}, Max={val_max:.3f}, Media={val_media:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = AnalisisLaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

"""

[INFO] [1772734736.045832715] [analisis_laser_node]: Capturado scan 1 de 10

[INFO] [1772734736.047034403] [analisis_laser_node]: Capturado scan 2 de 10

[INFO] [1772734736.047983815] [analisis_laser_node]: Capturado scan 3 de 10

[INFO] [1772734736.246648553] [analisis_laser_node]: Capturado scan 4 de 10

[INFO] [1772734736.457252831] [analisis_laser_node]: Capturado scan 5 de 10

[INFO] [1772734736.666377738] [analisis_laser_node]: Capturado scan 6 de 10

[INFO] [1772734736.870578982] [analisis_laser_node]: Capturado scan 7 de 10

[INFO] [1772734737.065933832] [analisis_laser_node]: Capturado scan 8 de 10

[INFO] [1772734737.260309932] [analisis_laser_node]: Capturado scan 9 de 10

[INFO] [1772734737.463229566] [analisis_laser_node]: Capturado scan 10 de 10

[INFO] [1772734737.463684129] [analisis_laser_node]:

--- RESULTADOS DEL ANÁLISIS ---

Ángulo 000° -> Min: 2.103m | Max: 2.133m | Media: 2.124m

Ángulo 001° -> Min: 2.161m | Max: 2.196m | Media: 2.172m

Ángulo 002° -> Min: 2.215m | Max: 2.250m | Media: 2.234m

Ángulo 003° -> Min: 2.276m | Max: 2.319m | Media: 2.302m

Ángulo 004° -> Min: 2.365m | Max: 2.401m | Media: 2.379m

Ángulo 005° -> Min: 2.445m | Max: 2.496m | Media: 2.476m

Ángulo 006° -> Min: 2.518m | Max: 2.631m | Media: 2.556m

Ángulo 007° -> Min: 2.710m | Max: 2.769m | Media: 2.738m

Ángulo 008° -> Min: 2.790m | Max: 2.862m | Media: 2.821m

Ángulo 009° -> Min: 2.921m | Max: 2.967m | Media: 2.948m

Ángulo 010° -> Min: 3.014m | Max: 3.099m | Media: 3.044m

Ángulo 011° -> Min: 3.158m | Max: 3.231m | Media: 3.188m

Ángulo 012° -> Min: 3.300m | Max: 3.352m | Media: 3.324m

Ángulo 013° -> Min: 3.418m | Max: 3.540m | Media: 3.474m

Ángulo 014° -> Min: 3.492m | Max: 3.527m | Media: 3.510m

Ángulo 015° -> Min: 3.581m | Max: 3.638m | Media: 3.614m

Ángulo 016° -> Min: 3.565m | Max: 3.594m | Media: 3.585m

Ángulo 017° -> Min: 3.603m | Max: 3.779m | Media: 3.670m

Ángulo 018° -> Min: 3.822m | Max: 3.974m | Media: 3.921m

Ángulo 019° -> Min: 3.852m | Max: 3.919m | Media: 3.897m

Ángulo 020° -> Min: 3.867m | Max: 3.942m | Media: 3.903m

Ángulo 021° -> Min: 3.866m | Max: 3.901m | Media: 3.881m

Ángulo 022° -> Min: 3.832m | Max: 3.885m | Media: 3.862m

Ángulo 023° -> Min: 3.839m | Max: 3.880m | Media: 3.869m

Ángulo 024° -> Min: 3.832m | Max: 3.873m | Media: 3.852m

Ángulo 025° -> Min: 3.807m | Max: 3.856m | Media: 3.830m

Ángulo 026° -> Min: 3.803m | Max: 3.884m | Media: 3.840m

Ángulo 027° -> Min: 3.815m | Max: 3.857m | Media: 3.831m

Ángulo 028° -> Min: 3.796m | Max: 3.848m | Media: 3.829m

Ángulo 029° -> Min: 3.797m | Max: 3.857m | Media: 3.827m

Ángulo 030° -> Min: 3.815m | Max: 3.856m | Media: 3.839m

Ángulo 031° -> Min: 3.797m | Max: 3.833m | Media: 3.816m

Ángulo 032° -> Min: 3.808m | Max: 3.846m | Media: 3.825m

Ángulo 033° -> Min: 3.797m | Max: 3.836m | Media: 3.822m

Ángulo 034° -> Min: 3.793m | Max: 3.841m | Media: 3.821m

Ángulo 035° -> Min: 3.443m | Max: 3.743m | Media: 3.585m

Ángulo 036° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 037° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 038° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 039° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 040° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 041° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 042° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 043° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 044° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 045° -> Min: 0.854m | Max: 1.075m | Media: 0.975m

Ángulo 046° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 047° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 048° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 049° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 050° -> Min: 0.216m | Max: 0.220m | Media: 0.217m

Ángulo 051° -> Min: 0.211m | Max: 0.214m | Media: 0.212m

Ángulo 052° -> Min: 0.204m | Max: 0.206m | Media: 0.205m

Ángulo 053° -> Min: 0.186m | Max: 0.196m | Media: 0.189m

Ángulo 054° -> Min: 0.188m | Max: 0.190m | Media: 0.189m

Ángulo 055° -> Min: 0.190m | Max: 0.191m | Media: 0.190m

Ángulo 056° -> Min: 0.182m | Max: 0.183m | Media: 0.183m

Ángulo 057° -> Min: 0.179m | Max: 0.182m | Media: 0.180m

Ángulo 058° -> Min: 0.175m | Max: 0.176m | Media: 0.175m

Ángulo 059° -> Min: 0.172m | Max: 0.173m | Media: 0.172m

Ángulo 060° -> Min: 0.170m | Max: 0.170m | Media: 0.170m

Ángulo 061° -> Min: 0.163m | Max: 0.165m | Media: 0.164m

Ángulo 062° -> Min: 0.161m | Max: 0.164m | Media: 0.162m

Ángulo 063° -> Min: 0.158m | Max: 0.159m | Media: 0.158m

Ángulo 064° -> Min: 0.155m | Max: 0.155m | Media: 0.155m

Ángulo 065° -> Min: 0.153m | Max: 0.153m | Media: 0.153m

Ángulo 066° -> Min: 0.151m | Max: 0.151m | Media: 0.151m

Ángulo 067° -> Min: 0.146m | Max: 0.147m | Media: 0.147m

Ángulo 068° -> Min: 0.145m | Max: 0.146m | Media: 0.146m

Ángulo 069° -> Min: 0.143m | Max: 0.144m | Media: 0.144m

Ángulo 070° -> Min: 0.140m | Max: 0.142m | Media: 0.141m

Ángulo 071° -> Min: 0.138m | Max: 0.139m | Media: 0.138m

Ángulo 072° -> Min: 0.137m | Max: 0.138m | Media: 0.137m

Ángulo 073° -> Min: 0.136m | Max: 0.137m | Media: 0.136m

Ángulo 074° -> Min: 0.132m | Max: 0.133m | Media: 0.132m

Ángulo 075° -> Min: 0.130m | Max: 0.131m | Media: 0.130m

Ángulo 076° -> Min: 0.130m | Max: 0.131m | Media: 0.130m

Ángulo 077° -> Min: 0.130m | Max: 0.131m | Media: 0.130m

Ángulo 078° -> Min: 0.125m | Max: 0.129m | Media: 0.128m

Ángulo 079° -> Min: 0.124m | Max: 0.125m | Media: 0.125m

Ángulo 080° -> Min: 0.124m | Max: 0.124m | Media: 0.124m

Ángulo 081° -> Min: 0.124m | Max: 0.125m | Media: 0.124m

Ángulo 082° -> Min: 0.123m | Max: 0.125m | Media: 0.124m

Ángulo 083° -> Min: 0.118m | Max: 0.122m | Media: 0.120m

Ángulo 084° -> Min: 0.118m | Max: 0.119m | Media: 0.118m

Ángulo 085° -> Min: 0.118m | Max: 0.119m | Media: 0.119m

Ángulo 086° -> Min: 0.119m | Max: 0.119m | Media: 0.119m

Ángulo 087° -> Min: 0.119m | Max: 0.120m | Media: 0.120m

Ángulo 088° -> Min: 0.113m | Max: 0.119m | Media: 0.114m

Ángulo 089° -> Min: 0.117m | Max: 0.121m | Media: 0.120m

Ángulo 090° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 091° -> Min: 0.000m | Max: 0.128m | Media: 0.089m

Ángulo 092° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 093° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 094° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 095° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 096° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 097° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 098° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 099° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 100° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 101° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 102° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 103° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 104° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 105° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 106° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 107° -> Min: 0.804m | Max: 0.876m | Media: 0.837m

Ángulo 108° -> Min: 0.738m | Max: 0.814m | Media: 0.765m

Ángulo 109° -> Min: 0.738m | Max: 0.841m | Media: 0.792m

Ángulo 110° -> Min: 0.777m | Max: 0.905m | Media: 0.858m

Ángulo 111° -> Min: 1.602m | Max: 1.636m | Media: 1.621m

Ángulo 112° -> Min: 1.608m | Max: 1.636m | Media: 1.626m

Ángulo 113° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 114° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 115° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 116° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 117° -> Min: 0.719m | Max: 1.105m | Media: 0.953m

Ángulo 118° -> Min: 0.642m | Max: 1.136m | Media: 0.940m

Ángulo 119° -> Min: 0.878m | Max: 1.116m | Media: 0.982m

Ángulo 120° -> Min: 0.917m | Max: 1.087m | Media: 0.985m

Ángulo 121° -> Min: 0.836m | Max: 1.105m | Media: 0.982m

Ángulo 122° -> Min: 0.806m | Max: 1.055m | Media: 0.978m

Ángulo 123° -> Min: 0.831m | Max: 1.147m | Media: 1.004m

Ángulo 124° -> Min: 0.898m | Max: 1.168m | Media: 0.999m

Ángulo 125° -> Min: 0.844m | Max: 1.121m | Media: 0.972m

Ángulo 126° -> Min: 0.932m | Max: 1.149m | Media: 0.999m

Ángulo 127° -> Min: 0.000m | Max: 1.091m | Media: 0.373m

Ángulo 128° -> Min: 0.791m | Max: 1.004m | Media: 0.902m

Ángulo 129° -> Min: 0.795m | Max: 1.041m | Media: 0.925m

Ángulo 130° -> Min: 0.000m | Max: 1.164m | Media: 0.116m

Ángulo 131° -> Min: 0.772m | Max: 1.041m | Media: 0.955m

Ángulo 132° -> Min: 0.754m | Max: 1.145m | Media: 0.962m

Ángulo 133° -> Min: 0.000m | Max: 1.218m | Media: 0.953m

Ángulo 134° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 135° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 136° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 137° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 138° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 139° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 140° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 141° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 142° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 143° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 144° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 145° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 146° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 147° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 148° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 149° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 150° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 151° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 152° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 153° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 154° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 155° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 156° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 157° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 158° -> Min: 0.461m | Max: 0.473m | Media: 0.466m

Ángulo 159° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 160° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 161° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 162° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 163° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 164° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 165° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 166° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 167° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 168° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 169° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 170° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 171° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 172° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 173° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 174° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 175° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 176° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 177° -> Min: 0.660m | Max: 0.692m | Media: 0.675m

Ángulo 178° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 179° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 180° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 181° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 182° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 183° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 184° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 185° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 186° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 187° -> Min: 0.720m | Max: 0.941m | Media: 0.825m

Ángulo 188° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 189° -> Min: 0.000m | Max: 2.915m | Media: 2.300m

Ángulo 190° -> Min: 2.760m | Max: 2.834m | Media: 2.790m

Ángulo 191° -> Min: 2.763m | Max: 2.817m | Media: 2.790m

Ángulo 192° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 193° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 194° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 195° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 196° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 197° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 198° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 199° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 200° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 201° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 202° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 203° -> Min: 2.604m | Max: 2.631m | Media: 2.622m

Ángulo 204° -> Min: 2.589m | Max: 2.620m | Media: 2.610m

Ángulo 205° -> Min: 2.599m | Max: 2.617m | Media: 2.609m

Ángulo 206° -> Min: 2.561m | Max: 2.576m | Media: 2.570m

Ángulo 207° -> Min: 2.565m | Max: 2.581m | Media: 2.573m

Ángulo 208° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 209° -> Min: 2.459m | Max: 2.475m | Media: 2.469m

Ángulo 210° -> Min: 2.459m | Max: 2.475m | Media: 2.467m

Ángulo 211° -> Min: 2.461m | Max: 2.485m | Media: 2.473m

Ángulo 212° -> Min: 2.463m | Max: 2.496m | Media: 2.482m

Ángulo 213° -> Min: 2.470m | Max: 2.501m | Media: 2.483m

Ángulo 214° -> Min: 2.478m | Max: 2.510m | Media: 2.498m

Ángulo 215° -> Min: 2.475m | Max: 2.502m | Media: 2.488m

Ángulo 216° -> Min: 2.496m | Max: 2.528m | Media: 2.515m

Ángulo 217° -> Min: 2.506m | Max: 2.524m | Media: 2.516m

Ángulo 218° -> Min: 2.513m | Max: 2.532m | Media: 2.523m

Ángulo 219° -> Min: 2.525m | Max: 2.546m | Media: 2.536m

Ángulo 220° -> Min: 2.527m | Max: 2.559m | Media: 2.541m

Ángulo 221° -> Min: 2.545m | Max: 2.585m | Media: 2.568m

Ángulo 222° -> Min: 2.552m | Max: 2.583m | Media: 2.567m

Ángulo 223° -> Min: 2.587m | Max: 2.600m | Media: 2.594m

Ángulo 224° -> Min: 2.592m | Max: 2.633m | Media: 2.610m

Ángulo 225° -> Min: 2.626m | Max: 2.647m | Media: 2.637m

Ángulo 226° -> Min: 2.641m | Max: 2.655m | Media: 2.648m

Ángulo 227° -> Min: 2.627m | Max: 2.644m | Media: 2.636m

Ángulo 228° -> Min: 2.635m | Max: 2.682m | Media: 2.658m

Ángulo 229° -> Min: 2.655m | Max: 2.681m | Media: 2.669m

Ángulo 230° -> Min: 0.734m | Max: 0.783m | Media: 0.760m

Ángulo 231° -> Min: 0.742m | Max: 0.783m | Media: 0.764m

Ángulo 232° -> Min: 0.704m | Max: 0.759m | Media: 0.741m

Ángulo 233° -> Min: 0.692m | Max: 0.728m | Media: 0.713m

Ángulo 234° -> Min: 0.689m | Max: 0.721m | Media: 0.705m

Ángulo 235° -> Min: 0.687m | Max: 0.707m | Media: 0.698m

Ángulo 236° -> Min: 0.678m | Max: 0.711m | Media: 0.695m

Ángulo 237° -> Min: 0.682m | Max: 0.705m | Media: 0.695m

Ángulo 238° -> Min: 0.679m | Max: 0.704m | Media: 0.695m

Ángulo 239° -> Min: 0.664m | Max: 0.694m | Media: 0.679m

Ángulo 240° -> Min: 0.650m | Max: 0.680m | Media: 0.665m

Ángulo 241° -> Min: 0.627m | Max: 0.648m | Media: 0.642m

Ángulo 242° -> Min: 0.617m | Max: 0.634m | Media: 0.625m

Ángulo 243° -> Min: 0.606m | Max: 0.615m | Media: 0.612m

Ángulo 244° -> Min: 0.604m | Max: 0.613m | Media: 0.609m

Ángulo 245° -> Min: 0.597m | Max: 0.612m | Media: 0.604m

Ángulo 246° -> Min: 0.579m | Max: 0.602m | Media: 0.591m

Ángulo 247° -> Min: 0.523m | Max: 0.547m | Media: 0.535m

Ángulo 248° -> Min: 0.496m | Max: 0.498m | Media: 0.497m

Ángulo 249° -> Min: 0.493m | Max: 0.495m | Media: 0.494m

Ángulo 250° -> Min: 0.460m | Max: 0.463m | Media: 0.462m

Ángulo 251° -> Min: 0.449m | Max: 0.455m | Media: 0.453m

Ángulo 252° -> Min: 0.447m | Max: 0.449m | Media: 0.448m

Ángulo 253° -> Min: 0.446m | Max: 0.448m | Media: 0.447m

Ángulo 254° -> Min: 0.443m | Max: 0.446m | Media: 0.444m

Ángulo 255° -> Min: 0.439m | Max: 0.443m | Media: 0.441m

Ángulo 256° -> Min: 0.439m | Max: 0.443m | Media: 0.441m

Ángulo 257° -> Min: 0.439m | Max: 0.444m | Media: 0.441m

Ángulo 258° -> Min: 0.435m | Max: 0.443m | Media: 0.439m

Ángulo 259° -> Min: 0.439m | Max: 0.441m | Media: 0.440m

Ángulo 260° -> Min: 0.448m | Max: 0.449m | Media: 0.449m

Ángulo 261° -> Min: 0.458m | Max: 0.460m | Media: 0.459m

Ángulo 262° -> Min: 0.468m | Max: 0.470m | Media: 0.469m

Ángulo 263° -> Min: 0.485m | Max: 0.490m | Media: 0.487m

Ángulo 264° -> Min: 0.501m | Max: 0.504m | Media: 0.502m

Ángulo 265° -> Min: 0.617m | Max: 0.636m | Media: 0.628m

Ángulo 266° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 267° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 268° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 269° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 270° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 271° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 272° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 273° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 274° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 275° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 276° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 277° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 278° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 279° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 280° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 281° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 282° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 283° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 284° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 285° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 286° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 287° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 288° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 289° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 290° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 291° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 292° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 293° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 294° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 295° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 296° -> Min: 0.000m | Max: 0.000m | Media: 0.000m

Ángulo 297° -> Min: 1.245m | Max: 1.396m | Media: 1.313m

Ángulo 298° -> Min: 1.225m | Max: 1.431m | Media: 1.291m

Ángulo 299° -> Min: 1.298m | Max: 1.399m | Media: 1.342m

Ángulo 300° -> Min: 1.339m | Max: 1.359m | Media: 1.349m

Ángulo 301° -> Min: 1.291m | Max: 1.342m | Media: 1.313m

Ángulo 302° -> Min: 0.000m | Max: 1.319m | Media: 0.912m

Ángulo 303° -> Min: 1.127m | Max: 1.152m | Media: 1.144m

Ángulo 304° -> Min: 1.140m | Max: 1.166m | Media: 1.149m

Ángulo 305° -> Min: 1.142m | Max: 1.173m | Media: 1.158m

Ángulo 306° -> Min: 1.144m | Max: 1.166m | Media: 1.156m

Ángulo 307° -> Min: 1.145m | Max: 1.181m | Media: 1.157m

Ángulo 308° -> Min: 1.145m | Max: 1.178m | Media: 1.160m

Ángulo 309° -> Min: 1.139m | Max: 1.177m | Media: 1.160m

Ángulo 310° -> Min: 1.151m | Max: 1.183m | Media: 1.168m

Ángulo 311° -> Min: 1.148m | Max: 1.180m | Media: 1.163m

Ángulo 312° -> Min: 1.155m | Max: 1.180m | Media: 1.171m

Ángulo 313° -> Min: 1.145m | Max: 1.178m | Media: 1.163m

Ángulo 314° -> Min: 1.163m | Max: 1.187m | Media: 1.178m

Ángulo 315° -> Min: 1.153m | Max: 1.186m | Media: 1.174m

Ángulo 316° -> Min: 1.173m | Max: 1.202m | Media: 1.185m

Ángulo 317° -> Min: 1.192m | Max: 1.217m | Media: 1.203m

Ángulo 318° -> Min: 1.180m | Max: 1.210m | Media: 1.197m

Ángulo 319° -> Min: 1.198m | Max: 1.223m | Media: 1.209m

Ángulo 320° -> Min: 1.188m | Max: 1.210m | Media: 1.199m

Ángulo 321° -> Min: 1.188m | Max: 1.224m | Media: 1.211m

Ángulo 322° -> Min: 1.196m | Max: 1.230m | Media: 1.215m

Ángulo 323° -> Min: 1.198m | Max: 1.238m | Media: 1.223m

Ángulo 324° -> Min: 1.219m | Max: 1.240m | Media: 1.234m

Ángulo 325° -> Min: 1.224m | Max: 1.257m | Media: 1.239m

Ángulo 326° -> Min: 1.245m | Max: 1.279m | Media: 1.259m

Ángulo 327° -> Min: 1.247m | Max: 1.270m | Media: 1.264m

Ángulo 328° -> Min: 1.262m | Max: 1.299m | Media: 1.274m

Ángulo 329° -> Min: 1.280m | Max: 1.295m | Media: 1.287m

Ángulo 330° -> Min: 1.276m | Max: 1.323m | Media: 1.306m

Ángulo 331° -> Min: 1.281m | Max: 1.304m | Media: 1.292m

Ángulo 332° -> Min: 1.299m | Max: 1.328m | Media: 1.313m

Ángulo 333° -> Min: 1.322m | Max: 1.345m | Media: 1.332m

Ángulo 334° -> Min: 1.318m | Max: 1.368m | Media: 1.342m

Ángulo 335° -> Min: 1.344m | Max: 1.364m | Media: 1.357m

Ángulo 336° -> Min: 1.349m | Max: 1.383m | Media: 1.364m

Ángulo 337° -> Min: 1.362m | Max: 1.383m | Media: 1.374m

Ángulo 338° -> Min: 1.378m | Max: 1.402m | Media: 1.392m

Ángulo 339° -> Min: 1.402m | Max: 1.431m | Media: 1.414m

Ángulo 340° -> Min: 1.413m | Max: 1.440m | Media: 1.426m

Ángulo 341° -> Min: 1.438m | Max: 1.453m | Media: 1.446m

Ángulo 342° -> Min: 1.462m | Max: 1.485m | Media: 1.474m

Ángulo 343° -> Min: 1.485m | Max: 1.506m | Media: 1.497m

Ángulo 344° -> Min: 1.504m | Max: 1.524m | Media: 1.512m

Ángulo 345° -> Min: 1.514m | Max: 1.532m | Media: 1.519m

Ángulo 346° -> Min: 1.537m | Max: 1.566m | Media: 1.552m

Ángulo 347° -> Min: 1.578m | Max: 1.597m | Media: 1.589m

Ángulo 348° -> Min: 1.597m | Max: 1.617m | Media: 1.605m

Ángulo 349° -> Min: 1.632m | Max: 1.646m | Media: 1.638m

Ángulo 350° -> Min: 1.656m | Max: 1.681m | Media: 1.670m

Ángulo 351° -> Min: 1.688m | Max: 1.712m | Media: 1.700m

Ángulo 352° -> Min: 1.715m | Max: 1.738m | Media: 1.726m

Ángulo 353° -> Min: 1.751m | Max: 1.774m | Media: 1.760m

Ángulo 354° -> Min: 1.779m | Max: 1.807m | Media: 1.790m

Ángulo 355° -> Min: 1.801m | Max: 1.828m | Media: 1.818m

Ángulo 356° -> Min: 1.863m | Max: 1.881m | Media: 1.871m

Ángulo 357° -> Min: 1.897m | Max: 1.915m | Media: 1.909m

Ángulo 358° -> Min: 1.909m | Max: 1.944m | Media: 1.933m

Ángulo 359° -> Min: 2.059m | Max: 2.089m | Media: 2.072m

"""