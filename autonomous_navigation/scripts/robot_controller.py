#!/usr/bin/python3
# This Python file uses the following encoding: utf-8

import math
import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped, PointStamped

class TurtlebotController():
    
    def __init__(self, rate):
        
        # Read parameters
        self.goal_tol = 0.15
        
        self.rate = rate # Hz
        
        # Initialize internal data 
        self.goal = PoseStamped()
        self.goal_received = False

        # TF listener
        self.tf_listener = tf.TransformListener()

        # Publishers / subscribers
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goalCallback)
      
        rospy.loginfo("TurtlebotController started")
        

    def shutdown(self):
        rospy.loginfo("Stop TurtleBot")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


    def goalCallback(self,goal):
        rospy.loginfo("Goal received! x: %.2f, y:%.2f", goal.pose.position.x, goal.pose.position.y)
        self.goal = goal  
        self.goal_received = True


    def command(self):

        # Check if we have a goal
        if not self.goal_received:
            rospy.loginfo("Goal not received. Waiting...")
            return

        # Check if we already reached the goal
        if self.goalReached():
            rospy.loginfo("GOAL REACHED! Stopping.")
            self.publish(0.0, 0.0)
            self.goal_received = False
            return
        
        rospy.loginfo("Goal received. Moving...")

        try:
            # Transform goal into local frame
            goal_local = self.tf_listener.transformPose('base_link', self.goal)

            x = goal_local.pose.position.x
            y = goal_local.pose.position.y

            angle = math.atan2(y, x)

            # Control law (simple)
            if angle > 0.1:
                linear = 0.0
                angular = 0.9
            elif angle < -0.1:
                linear = 0.0
                angular = -0.9
            else:
                linear = 0.2
                angular = 0.0

            self.publish(linear, angular)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF Problem")
            return


    def goalReached(self):

        if self.goal_received:

            # Ensure the goal timestamp is current for TF
            self.goal.header.stamp = rospy.Time(0)

            try:
                pose_transformed = self.tf_listener.transformPose('base_link', self.goal)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("TF Problem checking goal")
                return False

            dist = math.sqrt(
                pose_transformed.pose.position.x ** 2 +
                pose_transformed.pose.position.y ** 2
            )

            return dist < self.goal_tol

        return False

    
    def publish(self, lin_vel, ang_vel):
        move_cmd = Twist()
        move_cmd.linear.x = lin_vel
        move_cmd.angular.z = ang_vel
        rospy.loginfo("Commanding lv: %.2f, av: %.2f", lin_vel, ang_vel)
        self.cmd_vel_pub.publish(move_cmd)


if __name__ == '__main__':
    
    rospy.init_node('TurtlebotController', anonymous=False)

    rospy.loginfo("To stop TurtleBot CTRL + C")

    rate = 10
    robot = TurtlebotController(rate)
        
    rospy.on_shutdown(robot.shutdown)
        
    r = rospy.Rate(rate)
        
    while not rospy.is_shutdown():
        robot.command()
        r.sleep()
