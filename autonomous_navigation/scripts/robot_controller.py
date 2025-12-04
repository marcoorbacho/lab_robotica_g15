#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import math
import rospy
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import numpy as np

angulo_seg_delantero = 45 
dist_peligrosa = 0.5 
zona_muerta = 5

# Constantes de Control Proporcional
# Kp para el giro hacia el objetivo 
obj_kp = 1.5 
#kp para el frenado del objetivo
dist_kp=1.0
# Velocidad lineal máxima de avance 
max_linear_speed = 0.5 
# Velocidad angular máxima para giro 
max_angular_speed = 1.2
# Umbral de ángulo para considerar que estamos alineados (en radianes, ~3 grados)
tolerancia_alineada = np.radians(3)

class Obstacle:
    def __init__(self):
        self.angle = 0.0
        self.distance = 0.0 

class TurtlebotController():
    
    def __init__(self, rate):
        
        # Read parameters
        self.goal_tol = 0.15
        self.inicio=0

        #parametros cooldown evasion
        self.cooldown_activo=False
        self.duracion_cooldown=1.0
        self.cooldown_empezar=rospy.Time.now()
        self.evading_direction=0.0
        
        self.rate = rate # Hz  (1/Hz = secs)
        
        # Initialize internal data 
        self.goal = PoseStamped()
        self.goal_received = False

        # Subscribers / publishers
        self.tf_listener = tf.TransformListener()

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)

        # Subscriptor para el LIDAR
        self.scan_data = None
        rospy.Subscriber("scan", LaserScan, self.Datos_lidar)

        rospy.loginfo("TurtlebotController started")
        

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # A default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel_pub.publish(Twist())
        # Sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


    def goalCallback(self,goal):
        rospy.loginfo("Goal received! x: %.2f, y:%.2f", goal.pose.position.x, goal.pose.position.y)
        self.goal = goal  
        self.goal_received = True


    def command(self):

        # Check if we already received data
        if(self.goal_received == False):
            rospy.loginfo("Goal not received. Waiting...")
            return

        if(self.goal_received == True):
            rospy.loginfo("Goal received!! Moving...")
            linear = 0.0
            angular = 0.0
            # Moverse hacia el goal el orientación correcta
                
            try:
                self.goal.header.stamp = rospy.Time()
                base_goal = self.tf_listener.transformPose('base_link', self.goal)

                angle=math.atan2(base_goal.pose.position.y, base_goal.pose.position.x)

                distancia = math.sqrt(base_goal.pose.position.x**2 + base_goal.pose.position.y**2)                
                
                #Extraer datos del lidar
                colision = self.find_closest_obstacle(angulo_seg_delantero, zona_muerta)  
                linear = 0.0
                angular = 0.0   

                if self.cooldown_activo and (rospy.Time.now() - self.cooldown_empezar).to_sec() > self.duracion_cooldown:
                    self.cooldown_activo = False # Termina el periodo de giro forzado
                    rospy.loginfo("Cooldown de evasión terminado.")     

                if self.inicio==0:
                    angular = obj_kp * angle 
                    
                    # Limitamos la velocidad angular
                    angular = max(-max_angular_speed, min(max_angular_speed, angular))
                    self.inicio=1

                else:
                    if colision.distance>0.0 and colision.distance<dist_peligrosa and colision.distance < distancia:
                        if not self.cooldown_activo:
                            # Guardar la dirección de giro que se ha decidido (derecha o izquierda)
                            self.evading_direction = -1.0 if colision.angle >= 0.0 else 1.0
                            self.cooldown_activo = True
                            self.cooldown_empezar = rospy.Time.now()

                            # Factor Proporcional basado en la distancia (más cerca = factor mayor)
                            # Usamos una función inversa suave para evitar divisiones por cero y límites bruscos
                            safety_factor = max(0.0, (dist_peligrosa - colision.distance) / dist_peligrosa)
                            turn_direction = -1.0 if colision.angle >= 0.0 else 1.0
                        
                            # Velocidad angular de evasión, limitada por la velocidad máxima
                            angular = turn_direction * safety_factor * max_angular_speed
                            angular = max(-max_angular_speed, min(max_angular_speed, angular))

                            # Reducción de la velocidad lineal para maniobras seguras
                            # Si está muy cerca (menor que dist_peligrosa), la velocidad es muy baja
                            linear = min(max_linear_speed, 3*(colision.distance)/4 * max_linear_speed)

                            if linear < 0.1:
                                linear = 0.0  # Detenerse si está demasiado cerca

                        elif self.cooldown_activo:
                            rospy.loginfo("Cooldown de evasión activo: Giro sostenido.")
                            # Fuerza al robot a continuar el giro en la última dirección determinada
                            
                            # Velocidad angular constante para un giro rápido y sostenido
                            angular = self.evading_direction * max_angular_speed 
                            
                            # Permite un avance lento mientras gira
                            linear = 0.2
                    else:
                        # Girar si el ángulo al objetivo es grande
                        # Usamos control P para que el giro sea más suave cerca del objetivo (angle -> 0)
                        angular = obj_kp * angle 

                        
                        
                        # Limitamos la velocidad angular
                        angular = max(-max_angular_speed, min(max_angular_speed, angular))

                        # Si el robot está razonablemente alineado, avanzamos
                        if abs(angle) < 0.1: # umbral de 0.1 radianes
                                # Kp * distancia para frenado proporcional
                            linear = dist_kp * distancia 
                            linear = min(linear , max_linear_speed)
                        # Saturación
                        else:
                            # Reducimos la velocidad lineal para dar prioridad al giro de realineación
                            linear = max_linear_speed * (1.0 - abs(angular) / max_angular_speed)
                            linear = max(0.0, min(max_linear_speed, linear))


                #Mandar velocidades
                self.publish(linear,angular)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Problem TF")
                return False

        # Check if the final goal has been reached
        if(self.goalReached()==True):
            rospy.loginfo("GOAL REACHED!!! Stopping!")
            self.publish(0.0, 0.0)
            self.goal_received = False
            self.inicio=0
            return
        return 
        
        #######################################################################################################
        # Check for collisions                                                                                #
        # Implement control law                                                                               #
        # Note: You could transform next goal to local robot coordinates to compute control law more easily   #
        # Note: You should saturate the maximum angular and linear robot velocities                           #
        #######################################################################################################
            
        # Publish velocity command
        self.publish(linear,angular)
        return False
    

    # Recoger datos del LIDAR
    def Datos_lidar(self, dato):
        self.scan_data = dato  

    def find_closest_obstacle(self, fov_degrees: float, dead_zone_degrees: float) -> Obstacle:

        obstaculo = Obstacle()

        if self.scan_data is None:
            return obstaculo

        lidar_ranges = np.array(self.scan_data.ranges)
        num_points = len(lidar_ranges)
        angle_increment = self.scan_data.angle_increment
        
        points_per_side = int(fov_degrees / np.degrees(angle_increment))
        dead_points = int(dead_zone_degrees / np.degrees(angle_increment))
        
        # Indices frontales: [0 a points_per_side] y [num_points - points_per_side a num_points - 1]
        front_indices = np.concatenate([
            np.arange(points_per_side), 
            np.arange(num_points - points_per_side, num_points)
        ])
        
        # Indices de la zona muerta a excluir
        dead_zone_indices = np.concatenate([
            np.arange(dead_points), 
            np.arange(num_points - dead_points, num_points)
        ])
        
        # Indices de búsqueda: FOV frontal, excluyendo la zona muerta
        search_indices = np.setdiff1d(front_indices, dead_zone_indices)         #Excluye los indices repetidos
        
        if search_indices.size == 0:
            return obstaculo

        # 2. Encontrar la distancia mínima
        # Reemplazar np.inf y valores fuera de rango por un valor grande para la comparación
        lidar_search_values = lidar_ranges[search_indices].copy()
        
        # Asegurarse de que los valores inválidos (inf, nan) no causen problemas
        lidar_search_values[~np.isfinite(lidar_search_values)] = 100.0 

        min_distance = np.min(lidar_search_values)

        # 3. Calcular el ángulo del obstáculo más cercano
        if min_distance > 0.0 and min_distance < dist_peligrosa + 0.5: # Considerar distancias razonables
            
            min_index_relative = np.argmin(lidar_search_values)
            min_index_absolute = search_indices[min_index_relative]
            
            # Mapeo de índice a ángulo en radianes
            # El ángulo 0 es al frente. Indices crecientes (0->180) son Izquierda (positivo).
            if min_index_absolute < num_points / 2:
                # Ángulo positivo (izquierda)
                angle_degrees = min_index_absolute
            else:
                # Ángulo negativo (derecha)
                angle_degrees = min_index_absolute - num_points
                
            obstaculo.angle = np.radians(angle_degrees)
            obstaculo.distance = min_distance
            
        return obstaculo
           
        
    def goalReached(self):
        # Return True if the FINAL goal was reached, False otherwise

        if self.goal_received:
            pose_transformed = PoseStamped()

            # Update the goal timestamp to avoid issues with TF transform
            self.goal.header.stamp = rospy.Time()

            try:
                pose_transformed = self.tf_listener.transformPose('base_footprint', self.goal)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Problem TF")
                return False

            goal_distance = math.sqrt(pose_transformed.pose.position.x ** 2 + pose_transformed.pose.position.y ** 2)
            if(goal_distance < self.goal_tol):
                return True

        return False

    
    def publish(self, lin_vel, ang_vel):
        # Twist is a datatype for velocity
        move_cmd = Twist()
        # Copy the forward velocity
        move_cmd.linear.x = lin_vel
        # Copy the angular velocity
        move_cmd.angular.z = ang_vel
        rospy.loginfo("Commanding lv: %.2f, av: %.2f", lin_vel, ang_vel)
        self.cmd_vel_pub.publish(move_cmd)


if __name__ == '__main__':
    
    # Initiliaze
    rospy.init_node('TurtlebotController', anonymous=False)

    # Tell user how to stop TurtleBot
    rospy.loginfo("To stop TurtleBot CTRL + C")

    rate = 10 # Frecuency (Hz) for commanding the robot
    robot = TurtlebotController(rate)
        
    # What function to call when you CTRL + C    
    rospy.on_shutdown(robot.shutdown)
        
    # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
    r = rospy.Rate(rate)
        
    # As long as you haven't CTRL + C keeping doing...
    while not (rospy.is_shutdown()):
        
	    # Publish the velocity
        robot.command()

        # Wait for 0.1 seconds (10 HZ) and publish again
        r.sleep()