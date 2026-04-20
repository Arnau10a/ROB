# 🚀 Guia Final d'Execució: Projecte Navegació Autònoma

Aquesta guia conté els passos per executar la missió completa de 3 fases amb el **TurtleBot3 Burger** real.

## 1. Preparació al PC del Laboratori
Obre un terminal al PC per compilar el projecte:

```bash
# 1. Ves al teu workspace de ROS 2
cd ~/ros2_ws 

# 2. Compila el paquet de navegació
colcon build --packages-select autonomous_nav_pkg
source install/setup.bash

# 3. Configura el Domain ID (substitueix X pel número del teu Burger)
export ROS_DOMAIN_ID=X
export TURTLEBOT3_MODEL=burger
```

---

## 2. Connexió i "Bringup" del Robot
Has de connectar-te al robot via SSH. El robot ha d'estar encès i a la xarxa del lab.

1. **Terminal 1 (PC):** Connecta't al robot:
   ```bash
   ssh burger@10.10.73.2XX  # 2XX és la IP del teu robot
   # Password: turtlebot
   ```

2. **Terminal 1 (Dins SSH):** Llança els motors i sensors:
   ```bash
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
   *Deixa aquesta finestra oberta.*

---

## 3. Llançament de la Missió (3 Fases)
El robot completarà: **Fase I** (Ruta D→B→O), **Fase II** (Exploració + Trobar Estació + Tornar a Base) i **Fase III** (Docking de Precisió).

1. **Terminal 2 (PC):** Llança tot el projecte (SLAM + Controlador):
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   
   # Llança la missió (usa use_sim_time:=true si estàs en Gazebo)
   ros2 launch autonomous_nav_pkg mission.launch.py
   ```

---

## 4. Monitorització i Verificació
- **RViz2**: Obre `rviz2` al PC. Afegeix `Map` per veure el SLAM i `TF` per veure com el robot es localitza respecte al mapa (`map → base_link`).
- **Terminal**: El controlador treguerà missatges indicant en quina fase es troba (`PHASE I`, `PHASE II`, `PHASE III`).
- **Detecció**: Quan el robot entri al Passadís, veuràs logs de `[STATION]` si detecta els pilars de 40x40cm.

---

## 5. Resultats i Aturada
Quan el robot arribi al final (Docking completat):

1. **Atura els programes:** `Ctrl+C` als terminals.
2. **Recupera els fitxers:** Al directori on has executat la missió trobaràs:
   - `mission_log.csv`: El log requerit pel projecte amb posicions d'obstacles i pilars.
   - `generated_map.yaml` i `generated_map.pgm`: El mapa generat pel robot.
3. **Apaga el Robot (SSH):** 
   ```bash
   sudo shutdown now
   ```

> [!IMPORTANT]
> **Coordenades de Sortida**: El robot està configurat per sortir des del **Punt D** (3.32, 0.95). Si surts de Punt A o C, modifica `self.initial_x` i `self.initial_y` a `mission_controller.py` i torna a fer `colcon build`.

> [!TIP]
> **Seguretat**: Si el robot s'apropa massa a un obstacle no detectat pel SLAM, l'algorisme **APF** (forces repulsives) hauria d'actuar, però estigues preparat per aturar-lo manualment si cal.
