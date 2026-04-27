# Bibliografía y Referencias Técnicas

Este documento detalla las fuentes teóricas y técnicas consultadas para el desarrollo del sistema de navegación autónoma y percepción del TurtleBot3 Burger.

## 1. Fundamentos de Navegación Reactiva
*   **Khatib, O. (1986).** "Real-time obstacle avoidance for manipulators and mobile robots". *The International Journal of Robotics Research*. 
    *   *Nota:* Referencia principal para la implementación de los **Campos Potenciales Artificiales (APF)** utilizados en el nodo de navegación.
*   **Stanford University - Robotics Lab.** "Handout: Artificial Potential Fields".
    *   *Enlace:* [Stanford CS Robotics](https://see.stanford.edu/materials/lmanaphotos/handout24_PotentialFields.pdf)
    *   *Uso:* Consulta de las fórmulas de gradiente para fuerzas repulsivas basadas en sensores de rango (LiDAR).

## 2. Planificación y Control de Robots Móviles
*   **Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011).** *Introduction to Autonomous Mobile Robots*. MIT Press (ETH Zurich).
    *   *Nota:* Guía fundamental para la integración de la odometría, transformaciones de coordenadas (TF) y evasión de obstáculos.
*   **Linden713 (GitHub Repository).** "Artificial Potential Fields Implementation".
    *   *Enlace:* [linden713/artificial_potential_fields](https://github.com/linden713/artificial_potential_fields)
    *   *Uso:* Referencia técnica para la estructuración de vectores de fuerza en Python y visualización del comportamiento del algoritmo.

## 3. Percepción y Detección Geométrica
*   **Point Cloud Clustering Algorithms.** 
    *   Investigación sobre algoritmos de agrupamiento basados en distancia Euclídea para la segmentación de lecturas LiDAR 2D en "clusters" de objetos.
*   **Geometría Multivista y Reconocimiento de Patrones.**
    *   Implementación de lógica ad-hoc para la validación de cuadriláteros basada en restricciones de distancia entre vértices (detección de los 4 pilares de la base de carga).

## 4. Documentación de Plataforma y Framework
*   **Open Robotics.** "ROS 2 Jazzy Jalisco Documentation".
    *   *Enlace:* [docs.ros.org](https://docs.ros.org/en/jazzy/)
*   **ROBOTIS.** "TurtleBot3 e-Manual".
    *   *Enlace:* [emanual.robotis.com](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
    *   *Uso:* Configuración de hardware, calibración de sensores y límites cinemáticos del robot.
