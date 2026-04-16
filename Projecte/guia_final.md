# 🚀 Guia Final d'Execució: Projecte Navegació Autònoma

Aquesta guia conté tots els passos i comandes necessàries per executar el projecte al laboratori amb el **TurtleBot3 Burger** real.

## 1. Preparació al PC del Laboratori (Ubuntu)
Obre un terminal al PC i descarrega el teu projecte:

```bash
# 1. Ves a la carpeta on vulguis guardar el projecte
cd ~/Documents/UPC

# 2. Descarrega el projecte (Clone públic, no et demanarà usuari)
git clone https://github.com/Arnau10a/ROB.git
cd ROB

# 3. Activa l'entorn de ROS 2 Jazzy (necessari per a cada terminal)
source /opt/ros/jazzy/setup.bash

# 4. Instal·la els paquets de TurtleBot3 i Gazebo (si el PC s'ha reiniciat)
# Assegura't de tenir el fitxer .sh a /home/rob o on indiqui el profe
chmod +x install_ros2_turtlebot.sh
./install_ros2_turtlebot.sh
source ~/.bashrc

# 5. Configura el Domain ID del teu robot (substitueix X pel número del teu Burger)
export ROS_DOMAIN_ID=X
export TURTLEBOT3_MODEL=burger

# 6. Compila i carrega el TEU projecte
colcon build --packages-select autonomous_nav_pkg
source install/setup.bash
```

---

## 2. Connexió i "Bringup" del Robot
Has de connectar-te al robot via SSH. El robot ha d'estar encès.

1. **Terminal 1 (PC):** Connecta't al robot:
   ```bash
   ssh ubuntu@10.10.73.2XX  # 2XX és la IP del teu robot (ex: 241)
   # Password: turtlebot
   ```

2. **Terminal 1 (Dins SSH):** Llança els motors i sensors:
   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
   *Deixa aquesta finestra oberta fins al final.*

---

## 3. Llançament de la Missió
Col·loca el robot al punt de sortida i orienta'l correctament segons el mapa.

1. **Terminal 2 (PC):** Llança tot el projecte (SLAM + Controlador):
   ```bash
   # Recorda fer source si és un terminal nou
   source /opt/ros/jazzy/setup.bash
   source ~/Documents/UPC/ROB/install/setup.bash
   
   # Llança la missió
   ros2 launch autonomous_nav_pkg mission.launch.py
   ```

---

## 4. Monitorització (RVIZ)
Si vols veure el mapa que s'està creant:

1. **Terminal 3 (PC):** Obre la visualització:
   ```bash
   rviz2
   ```

---

## 5. Aturada Segura (MOLT IMPORTANT)
Quan acabis la missió (el robot guardarà el mapa automàticament), apaga-ho tot així:

1. **Atura els programes:** Fes `Ctrl+C` a tous els terminals del PC.
2. **Apaga el Robot (SSH):** Al terminal on estàs connectat al robot (T1), executa:
   ```bash
   sudo shutdown now
   ```
3. Espera 15 segons i tanca l'interruptor físic del Burger.

> [!TIP]
> **Recordatori de Coordenades**: Si el robot comença en un punt diferent al **Punt A**, recorda obrir `mission_controller.py` i canviar els valors de `self.initial_x` i `self.initial_y` abans del `colcon build`.
