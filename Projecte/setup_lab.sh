#!/bin/bash

# =================================================================
# SCRIPT DE AUTOMATIZACIÓN PARA EL LABORATORIO (UBUNTU 24 / JAZZY)
# =================================================================

echo "--- Iniciando configuración del entorno ROS 2 ---"

# 1. Ejecutar el script de instalación del lab
if [ -f "./install_ros2_turtlebot.sh" ]; then
    echo "[1/5] Ejecutando install_ros2_turtlebot.sh..."
    chmod +x install_ros2_turtlebot.sh
    ./install_ros2_turtlebot.sh
else
    echo "ERROR: No se encuentra install_ros2_turtlebot.sh en esta carpeta."
    exit 1
fi

# 2. Configurar ROS_DOMAIN_ID
echo "-------------------------------------------------"
read -p "Introduce tu ROS_DOMAIN_ID (N): " domain_id
export ROS_DOMAIN_ID=$domain_id
echo "ROS_DOMAIN_ID configurado a: $ROS_DOMAIN_ID"

# Asegurar que las variables de ROS están cargadas
source /opt/ros/jazzy/setup.bash

# 3. Crear Workspace y Paquetes
echo "[2/5] Creando workspace y paquetes..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Core
ros2 pkg create --build-type ament_python project_core_pkg
# Fases
ros2 pkg create --build-type ament_python --node-name phase1_node phase1_pkg --dependencies project_core_pkg rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros
ros2 pkg create --build-type ament_python --node-name phase2_node phase2_pkg --dependencies project_core_pkg rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros
ros2 pkg create --build-type ament_python --node-name phase3_node phase3_pkg --dependencies project_core_pkg rclpy geometry_msgs sensor_msgs nav_msgs tf2_ros

# 4. Copiar los archivos .py (Asume que están en la carpeta donde lanzaste el script)
# Volvemos a la carpeta original donde está el script y los archivos .py
ORIGIN_DIR=$OLDPWD 
echo "[3/5] Copiando archivos fuente desde $ORIGIN_DIR..."

# Copiar Core
cp $ORIGIN_DIR/navigation.py ~/ros2_ws/src/project_core_pkg/project_core_pkg/
cp $ORIGIN_DIR/perception.py ~/ros2_ws/src/project_core_pkg/project_core_pkg/
cp $ORIGIN_DIR/__init__.py ~/ros2_ws/src/project_core_pkg/project_core_pkg/ 2>/dev/null || touch ~/ros2_ws/src/project_core_pkg/project_core_pkg/__init__.py

# Copiar Fases
cp $ORIGIN_DIR/phase1_node.py ~/ros2_ws/src/phase1_pkg/phase1_pkg/
cp $ORIGIN_DIR/phase2_node.py ~/ros2_ws/src/phase2_pkg/phase2_pkg/
cp $ORIGIN_DIR/phase3_node.py ~/ros2_ws/src/phase3_pkg/phase3_pkg/

# Crear __init__.py faltantes
touch ~/ros2_ws/src/phase1_pkg/phase1_pkg/__init__.py
touch ~/ros2_ws/src/phase2_pkg/phase2_pkg/__init__.py
touch ~/ros2_ws/src/phase3_pkg/phase3_pkg/__init__.py

# 5. Compilar
echo "[4/5] Compilando workspace..."
cd ~/ros2_ws
colcon build

# Finalización
echo "[5/5] Proceso completado."
echo "-------------------------------------------------"
echo "RECUERDA ejecutar: source ~/ros2_ws/install/setup.bash"
echo "Para lanzar la fase 1: ros2 run phase1_pkg phase1_node"
echo "-------------------------------------------------"

# Mantener el DOMAIN_ID en la sesión actual
exec bash
