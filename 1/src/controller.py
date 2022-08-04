#!/usr/bin/python3

from time import sleep
import rospy
import tf
import numpy as np
from math import radians, sqrt, pow
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Pose, Vector3

class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller" , anonymous=False)
        
        self.distance_vector_subscirber = rospy.Subscriber("/distance_vector" , Vector3 , callback=self.move_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        self.goal_point_publisher = rospy.Publisher("/goal_point" , Point , queue_size=10)

        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed")
        self.angular_speed = rospy.get_param("/controller/angular_speed")
        self.goal_angle_param = radians(rospy.get_param("/controller/goal_angle"))
        self.distance_epsilon = rospy.get_param("/controller/distance_epsilon")
        self.angle_epsilon = rospy.get_param("/controller/angle_epsilon")
        self.length = rospy.get_param("/controller/length")
        self.width = rospy.get_param("/controller/width")

        # goal params
        self.goal_point_index = 0
        self.goal_points = []
        self.goal_angle = 0
        
        # defining the states of our robot
        self.GO = 0
        self.ROTATE = 1

        # initial state
        self.state = self.GO 
    
    # getting path points
    def set_path(self):
        half_length = self.length / 2
        half_width = self.width / 2

        self.goal_points.append(Point(half_length, half_width, 0))
        self.goal_points.append(Point(-half_length, half_width, 0))
        self.goal_points.append(Point(-half_length, -half_width, 0))
        self.goal_points.append(Point(half_length, -half_width, 0))

    def set_next_goal(self):
        self.goal_point_index = (self.goal_point_index + 1) % len(self.goal_points)
        self.goal_point_publisher.publish(self.goal_points[self.goal_point_index])
          
    # checks whether there is an obstacle in front of the robot
    # or not
    def move_callback(self, vector: Vector3):
        if self.goal_point_index == 0 or self.goal_point_index == 2:
            self.goal_distance = abs(vector.x)
        else:
            self.goal_distance = abs(vector.y)

        if (self.goal_distance <= self.distance_epsilon):
            self.state = self.ROTATE
    
    # heading of the robot 
    def get_heading(self):
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return abs(yaw)

    def set_first_heading(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        position = msg.pose.pose.position

        d_up = self.width / 2 - position.y
        d_down = self.width / 2 + position.y
        d_left = self.length / 2 + position.x
        d_right = self.length / 2 - position.x
        d_min = min(d_up, d_down, d_left, d_right)
        d_a = 5

        if d_right == d_min:
            self.goal_angle = radians(0 + d_a)
            self.goal_point_index = 0
        elif d_left == d_min:
            self.goal_angle = radians(180 + d_a)
            self.goal_point_index = 3
        elif d_up == d_min:
            self.goal_angle = radians(90 + d_a)
            self.goal_point_index = 1
        elif d_down == d_min:
            self.goal_angle = radians(270 + d_a)
            self.goal_point_index = 2
        
        self.goal_point_publisher.publish(self.goal_points[self.goal_point_index])

    def rotate(self, angle):
        # start rotating   
        remaining = angle
        prev_angle = self.get_heading()
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
        # stop after loop
        while remaining > self.angle_epsilon:
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            remaining -= delta
            prev_angle = current_angle
        twist.angular.z = 0    
        self.cmd_publisher.publish(twist)
        rospy.sleep(1)
        self.state = self.GO
            
    def run(self):
        rospy.sleep(1)
        # get to the rectangle
        self.set_path()
        self.set_first_heading()
        self.rotate(self.goal_angle)
        # wait for 1 sec

        while not rospy.is_shutdown():
            # move if state is GO
            if self.state == self.GO:
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)
                continue

            # wait for 1 sec
            self.cmd_publisher.publish(Twist())
            rospy.sleep(1)

            # set_next_goal
            self.set_next_goal()

            # rotate
            self.rotate(self.goal_angle_param)

            # set state to GO
            self.state = self.GO

if __name__ == "__main__":
    controller = Controller()
    controller.run()