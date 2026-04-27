# 🚀 Guía Maestra: Proyecto Navegación Autònoma (3 Fases)

Esta guía contiene todos los pasos necesarios para configurar y ejecutar el proyecto en los ordenadores del laboratorio (Ubuntu 24 / ROS 2 Jazzy).

---

## OPCIÓN A: Configuración Automática (Recomendado)
Si tienes el script `setup_lab.sh` y tus archivos `.py` en la misma carpeta:

1. Abre un terminal en la carpeta de tu USB.
2. Ejecuta:
   ```bash
   chmod +x setup_lab.sh
   ./setup_lab.sh
   ```
3. Introduce tu **ROS_DOMAIN_ID** cuando te lo pida.

---

## OPCIÓN B: Configuración Manual (Paso a paso)
Si prefieres hacerlo todo a mano, sigue estos comandos en orden:

### 1. Inicializar Sistema y Workspace
```bash
# Instalar entorno del lab
chmod +x install_ros2_turtlebot.sh
./install_ros2_turtlebot.sh
source ~/.bashrc

# Configurar variables
export ROS_DOMAIN_ID=N   # Cambia N por tu número
source /opt/ros/jazzy/setup.bash

# Crear carpetas
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Crear Paquetes y Estructura
```bash
# Crear Core
ros2 pkg create --build-type ament_python project_core_pkg

# Crear Fases (con dependencias y nodos)
ros2 pkg create --build-type ament_python --node-name phase1_node phase1_pkg --dependencies project_core_pkg rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros
ros2 pkg create --build-type ament_python --node-name phase1_alt_node phase1_alt_pkg --dependencies project_core_pkg rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros
ros2 pkg create --build-type ament_python --node-name phase1_pro_node phase1_pro_pkg --dependencies project_core_pkg rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros std_msgs
ros2 pkg create --build-type ament_python --node-name phase2_node phase2_pkg --dependencies project_core_pkg rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros
ros2 pkg create --build-type ament_python --node-name phase3_node phase3_pkg --dependencies project_core_pkg rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros
```

### 3. Copiar Archivos Manualmente
Copia tus archivos `.py` desde el USB a sus carpetas correspondientes:

| Archivo Fuente | Carpeta Destino |
| :--- | :--- |
| `navigation.py`, `perception.py` | `~/ros2_ws/src/project_core_pkg/project_core_pkg/` |
| `phase1_node.py` | `~/ros2_ws/src/phase1_pkg/phase1_pkg/` |
| `phase1_alt_node.py` | `~/ros2_ws/src/phase1_alt_pkg/phase1_alt_pkg/` |
| `phase1_pro_node.py` | `~/ros2_ws/src/phase1_pro_pkg/phase1_pro_pkg/` |
| `params.yaml` | `~/ros2_ws/src/phase1_pro_pkg/config/` |
| `phase1_pro.launch.py` | `~/ros2_ws/src/phase1_pro_pkg/launch/` |
| `phase2_node.py` | `~/ros2_ws/src/phase2_pkg/phase2_pkg/` |
| `phase3_node.py` | `~/ros2_ws/src/phase3_pkg/phase3_pkg/` |

*Asegúrate de que existan archivos `__init__.py` vacíos en cada una de esas carpetas de destino. Recuerda crear las carpetas `config/` y `launch/` dentro de `phase1_pro_pkg` y modificar su `setup.py` para incluir los data_files.*

### 4. Compilar
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🛠️ Ejecución de la Misión

### 1. Puesta en marcha del Robot (SSH)
```bash
ssh ubuntu@10.10.73.2xx   # Sustituye xx por el ID de tu robot
# Robot SSH:
ros2 launch turtlebot3_bringup robot.launch.py
```

### 2. Lanzar SLAM (PC Lab)
```bash
export ROS_DOMAIN_ID=N
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

### 3. Ejecutar Fase deseada (PC Lab)
```bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=N

# Fase I (Navegación Global - Original APF):
ros2 run phase1_pkg phase1_node

# Fase I (Alternativa - Máquina de Estados):
ros2 run phase1_alt_pkg phase1_alt_node

# Fase I (Pro - Parámetros, Telemetría y Launch):
ros2 launch phase1_pro_pkg phase1_pro.launch.py

# Fase II (Exploración):
ros2 run phase2_pkg phase2_node

# Fase III (Docking):
ros2 run phase3_pkg phase3_node
```

---

## 🛑 Finalización
Apaga el robot desde su terminal SSH:
```bash
sudo shutdown now
```
