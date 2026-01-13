# lab_robotica_g15
Este paquete (`autonomous_navigation`) implementa un sistema de navegación autónoma reactiva para el robot TurtleBot3. El nodo principal permite al robot alcanzar objetivos de navegación (*goals*) en un entorno global, implementando un control proporcional y una estrategia de evasión de obstáculos basada en datos de LIDAR.

## 1. Estructura del Código

El script principal, `robot_controller.py`, define la clase `TurtlebotController`, la cual orquesta la lógica de control mediante una pseudo-máquina de estados.

* **Clase `TurtlebotController`:**
    * **Gestión de Objetivos:** Recibe coordenadas globales (`PoseStamped`), las transforma al marco de referencia del robot (`base_link`) usando `tf`, y calcula el error angular y la distancia.
    * **Control Cinemático:** Implementa un controlador proporcional ($P$) para la velocidad angular y lineal. El robot prioriza la alineación angular antes del desplazamiento lineal.
    * **Lógica de Navegación:**
        1.  **Alineación:** Giro en el sitio hasta reducir el error angular.
        2.  **Avance:** Desplazamiento hacia el objetivo ajustando la orientación.
        3.  **Recuperación (Recovery):** Si el robot detecta inactividad (posible atasco), ejecuta una maniobra de giro de 180º.
    * **Evasión de Obstáculos:** Utiliza el método `find_closest_obstacle` para procesar el array del láser. Si se detecta un objeto dentro de la `dist_peligrosa` (0.75m), se activa un vector de evasión que modifica la trayectoria y reduce la velocidad lineal por seguridad.

* **Clase `Obstacle`:** Estructura de datos auxiliar que almacena la distancia y el ángulo del obstáculo más cercano filtrado por el LIDAR.

## 2. Interfaces del Nodo

El nodo interactúa con el ecosistema ROS a través de los siguientes tópicos:

### Suscripciones (Subscribers)

| Tópico | Tipo de Mensaje | Descripción |
| :--- | :--- | :--- |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Objetivo de destino (X, Y) enviado desde rviz. |
| `/scan` | `sensor_msgs/LaserScan` | Lecturas del sensor LIDAR para la detección de obstáculos. |
| `/tf` | `tf/tfMessage` | Transformadas necesarias para referenciar el objetivo respecto al robot. |

### Publicaciones (Publishers)

| Tópico | Tipo de Mensaje | Descripción |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Comandos de velocidad lineal ($v$) y angular ($\omega$) enviados al robot. |

---

## 3. Instrucciones de Lanzamiento

El sistema puede ejecutarse tanto en el entorno de simulación (Gazebo) como en el robot físico.

En el caso de la simulación será necesario lanzar los nodos de rviz y del propio controlador. Además de |
incluir el archivo `turtlebot3_sim.launch` que lanza la simulación en sí con diferentes parámetros así como |
el escenario y la posición en los 3 ejes del robot.

Para el caso de la simulación real, es necesario lanzar el comando `roslaunch turtlebot3_bringup turtlebot3_robot.launch`|
en la conexión ssh con el robot físico para poner en marcha sus drivers. Posteriormente lanzamos un archivo .launch que contiene |
los nodos de rviz y el controlador al igual que antes y además incluye el archivo `turtlebot3_remote.launch` que tiene como argumento |
el modelo del turtlebot y sirve para recibir los mensajes de los drivers que se mandan desde el |
propio sistema físico.