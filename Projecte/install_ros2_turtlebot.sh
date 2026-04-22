#!/bin/bash
# =================================================================
# INSTAL·LACIÓ COMPLETA: ROS 2 JAZZY + TURTLEBOT3 (UBUNTU 24.04)
# =================================================================
set -e

echo "🌟 Iniciant configuració total (esquivant fake_node)..."

# 1. ACTUALITZAR I REPOSITORIS (Claus GPG)
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update

# 2. INSTAL·LAR PAQUETS (Incloent el pont ros_gz i simulació)
echo "🛠 Instal·lant binaris de ROS 2, Gazebo Harmonic i dependències..."
sudo apt install -y \
  ros-jazzy-desktop \
  gz-harmonic \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-cartographer \
  ros-jazzy-cartographer-ros \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

# Inicialitzar rosdep
sudo rosdep init || true
rosdep update

# 3. WORKSPACE I CODI FONT
echo "📥 Baixant codi de TurtleBot3 (branca Jazzy)..."
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

repos=(
    "https://github.com/ROBOTIS-GIT/DynamixelSDK.git"
    "https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git"
    "https://github.com/ROBOTIS-GIT/turtlebot3.git"
    "https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git"
)

for r in "${repos[@]}"; do
    d=$(basename "$r" .git)
    [ ! -d "$d" ] && git clone -b jazzy "$r"
done

# 4. COMPILACIÓ SELECTIVA (Sense el metapaquet que demana el fake_node)
echo "🏗 Compilant paquets útils (ignorant fake_node)..."
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y -r

colcon build --symlink-install --packages-select \
    turtlebot3_gazebo \
    turtlebot3_description \
    turtlebot3_teleop \
    turtlebot3_node \
    turtlebot3_bringup \
    turtlebot3_msgs \
    dynamixel_sdk

# 5. CONFIGURACIÓ BASHRC
echo "📝 Escrivint variables al .bashrc..."
f_bash() { grep -qF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc; }

f_bash "source /opt/ros/jazzy/setup.bash"
f_bash "source ~/turtlebot3_ws/install/setup.bash"
f_bash "export TURTLEBOT3_MODEL=burger"
f_bash "export ROS_DOMAIN_ID=30"
f_bash "export GZ_VERSION=harmonic"
f_bash "export GZ_SIM_RESOURCE_PATH=~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models"
f_bash "alias eb='nano ~/.bashrc'"
f_bash "alias sb='source ~/.bashrc'"

echo "✅ INSTAL·LACIÓ COMPLETADA!"
echo "Executa: source ~/.bashrc"
