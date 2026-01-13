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
dist_peligrosa = 0.75 
zona_muerta = 5

# Constantes de Control Proporcional
obj_kp = 3.0 
dist_kp=0.8
max_linear_speed = 0.7 
max_angular_speed = 1.2
tolerancia_alineada = np.radians(10)

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
        self.duracion_cooldown=0.25
        self.cooldown_empezar=rospy.Time.now()
        self.evading_direction=0.0
        
        self.last_move_time = rospy.Time.now() 
        self.recovering_180 = False            
        self.recovery_end_time = rospy.Time.now()

        self.rate = rate # Hz  (1/Hz = secs)
        
        # Initialize internal data 
        self.goal = PoseStamped()
        self.goal_received = False

        # Subscribers / publishers
        self.tf_listener = tf.TransformListener()

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)

        self.scan_data = None
        rospy.Subscriber("scan", LaserScan, self.Datos_lidar)

        rospy.loginfo("TurtlebotController started")
        

    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


    def goalCallback(self,goal):
        rospy.loginfo("Goal received! x: %.2f, y:%.2f", goal.pose.position.x, goal.pose.position.y)
        self.goal = goal  
        self.goal_received = True
        self.inicio=0
        # Resetear temporizador al recibir nuevo goal
        self.last_move_time = rospy.Time.now()


    def command(self):

        if(self.goal_received == False):
            rospy.loginfo_throttle(5, "Goal not received. Waiting...")
            return

        if(self.goal_received == True):
            linear = 0.0
            angular = 0.0
                
            try:
                self.goal.header.stamp = rospy.Time()
                base_goal = self.tf_listener.transformPose('base_link', self.goal)

                angle=math.atan2(base_goal.pose.position.y, base_goal.pose.position.x)
                distancia = math.sqrt(base_goal.pose.position.x**2 + base_goal.pose.position.y**2)                
                
                colision = self.find_closest_obstacle(angulo_seg_delantero, zona_muerta)  

                linear = 0.0
                angular = 0.0   
                current_time = rospy.Time.now()

                # Lógica normal de movimiento
                if self.cooldown_activo and (rospy.Time.now() - self.cooldown_empezar).to_sec() > self.duracion_cooldown:
                    self.cooldown_activo = False 
                    rospy.loginfo("Cooldown de evasión terminado.")    

                if self.inicio==0:
                    angular = obj_kp**2 * angle
                    linear= 0.0
                    if abs(angle)<tolerancia_alineada:
                        self.inicio=1
                        self.last_move_time = current_time 
                        
                    angular = max(-max_angular_speed, min(max_angular_speed, angular))

                else:
                    # Bloque de evasión de obstáculos
                    if colision.distance>0.0 and colision.distance<dist_peligrosa and colision.distance < distancia:
                        if not self.cooldown_activo:
                            self.evading_direction = -1.0 if colision.angle >= 0.0 else 1.0
                            self.cooldown_activo = True
                            self.cooldown_empezar = rospy.Time.now()

                            safety_factor = max(0.0, (dist_peligrosa - colision.distance) / dist_peligrosa)
                            turn_direction = -1.0 if colision.angle >= 0.0 else 1.0
                        
                            angular = turn_direction * safety_factor * max_angular_speed
                            angular = max(-max_angular_speed, min(max_angular_speed, angular))

                            linear = min(max_linear_speed, (colision.distance)*3/4 * max_linear_speed)

                            if linear < 0.15:
                                linear = 0.0 

                        elif self.cooldown_activo:
                            rospy.loginfo_throttle(1, "Cooldown de evasión activo...")
                            angular = self.evading_direction * max_angular_speed*3/4
                            linear = min(max_linear_speed, (colision.distance)*3/4 * max_linear_speed)

                            if linear < 0.15:
                                linear = 0.0 
                    else:
                        # Navegación normal hacia el objetivo
                        angular = obj_kp * angle 
                        angular = max(-max_angular_speed, min(max_angular_speed, angular))
                        linear=max_linear_speed
                        linear = max(max_linear_speed*3/4, linear)
                        if abs(angle) < 0.1: 
                            angular =0.0
                        else:
                            linear = max_linear_speed/2

                        if colision.distance > distancia:
                            linear = min(linear, (colision.distance)*1.5 * max_linear_speed)


                if self.recovering_180:
                    if current_time < self.recovery_end_time:
                        # Sobrescribimos cualquier comando anterior
                        linear = 0.0
                        angular = max_angular_speed 
                        rospy.logwarn_throttle(1, "Recuperación: Girando 180 grados...")
                    else:
                        # Se acabó el tiempo de giro
                        self.recovering_180 = False
                        self.last_move_time = current_time 
                        self.inicio = 0 
                        rospy.loginfo("Recuperación terminada. Volviendo a control normal.")

                else:
                    is_active = (linear > 0.05) or (self.inicio == 0)

                    if is_active:
                        self.last_move_time = current_time
                    else:
                        # Solo contamos tiempo si NO estamos activos (ni moviéndonos ni alineándonos)
                        time_stopped = (current_time - self.last_move_time).to_sec()
                        
                        if time_stopped > 3.0: 
                            self.recovering_180 = True
                            turn_duration = math.pi / max_angular_speed
                            self.recovery_end_time = current_time + rospy.Duration(turn_duration)
                            rospy.logwarn("¡ATASCO DETECTADO! Iniciando maniobra de 180º.")
                

                # Mandar velocidades
                self.publish(linear,angular)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Problem TF")
                return False

        if(self.goalReached()==True):
            rospy.loginfo("GOAL REACHED!!! Stopping!")
            self.publish(0.0, 0.0)
            self.goal_received = False
            self.inicio=0
            return
        return
        
    def Datos_lidar(self, dato):
        self.scan_data = dato  

    def find_closest_obstacle(self, fov_degrees: float, dead_zone_degrees: float) -> Obstacle:
            obstaculo = Obstacle()
            obstaculo.distance = 0.0 

            if self.scan_data is None:
                return obstaculo

            lidar_ranges = np.array(self.scan_data.ranges)
            num_points = len(lidar_ranges)
            angle_increment = self.scan_data.angle_increment
            
            points_per_side = int(fov_degrees / np.degrees(angle_increment))
            dead_points = int(dead_zone_degrees / np.degrees(angle_increment))
            
            front_indices = np.concatenate([
                np.arange(points_per_side), 
                np.arange(num_points - points_per_side, num_points)
            ])
            
            dead_zone_indices = np.concatenate([
                np.arange(dead_points), 
                np.arange(num_points - dead_points, num_points)
            ])
            
            search_indices = np.setdiff1d(front_indices, dead_zone_indices)
            
            if search_indices.size == 0:
                return obstaculo

            lidar_search_values = lidar_ranges[search_indices].copy()
            lidar_search_values[~np.isfinite(lidar_search_values)] = 100.0
            lidar_search_values[lidar_search_values < 0.12] = 100.0 

            min_distance = np.min(lidar_search_values)

            if min_distance < 100.0:
                min_index_relative = np.argmin(lidar_search_values)
                min_index_absolute = search_indices[min_index_relative]
                
                if min_index_absolute < num_points / 2:
                    angle_rad = min_index_absolute * angle_increment
                else:
                    angle_rad = (min_index_absolute - num_points) * angle_increment
                    
                obstaculo.angle = angle_rad
                obstaculo.distance = min_distance
                
            return obstaculo
           
        
    def goalReached(self):
        if self.goal_received:
            pose_transformed = PoseStamped()
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
        move_cmd = Twist()
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        self.cmd_vel_pub.publish(move_cmd)


if __name__ == '__main__':
    rospy.init_node('TurtlebotController', anonymous=False)
    rospy.loginfo("To stop TurtleBot CTRL + C")
    rate = 10 
    robot = TurtlebotController(rate)
    rospy.on_shutdown(robot.shutdown)
    r = rospy.Rate(rate)
    while not (rospy.is_shutdown()):
        robot.command()
        r.sleep()