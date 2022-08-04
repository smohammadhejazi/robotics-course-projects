#!/usr/bin/python3


from cmath import log
from signal import signal

from matplotlib.pyplot import get
import rospy
import tf
import random
import numpy as np
from time import sleep
from math import radians, degrees, sqrt, pow, floor, atan2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Twist
from sensor_msgs.msg import LaserScan


class MapBuilder:
    def __init__(self) -> None:
        rospy.init_node("map_builder" , anonymous=False)
        self.yaw_star = 0

        self.front_dis = -1
        self.left_dis = -1
        self.behind_dis = -1
        self.right_dis = -1

        self.angle_epsilon = 0.01
        self.angular_speed = 0.5
        self.linear_speed = 0.2

        self.state = 0
        self.GO = 0
        self.ROTATE = 1

        self.min_distance = 1

        self.seperation_KP = 20
        self.seperation_KD = 40
        self.prev_seperation = 0
        self.time = rospy.get_time()


        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry, callback=self.odom_callback)


    def get_heading(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw
    
    def get_distance_seperation(self, msg : Odometry):
        current_orientation = msg.pose.pose.orientation
        # seperation
        yaw_star = self.yaw_star
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            current_orientation.x ,current_orientation.y ,current_orientation.z ,current_orientation.w
        ))

        if yaw_star < 0:
            yaw_star += radians(360)
        if yaw < 0:
            yaw += radians(360)

        seperation = 0
        if yaw_star > yaw and yaw_star - yaw > radians(360) - (yaw_star - yaw):
            seperation = -(radians(360) - (yaw_star - yaw))
        elif yaw_star > yaw and yaw_star - yaw < radians(360) - (yaw_star - yaw):
            seperation = yaw_star - yaw
        elif yaw > yaw_star and yaw - yaw_star > radians(360) - (yaw - yaw_star):
            seperation = radians(360) - (yaw - yaw_star)
        elif yaw > yaw_star and yaw - yaw_star < radians(360) - (yaw - yaw_star):
            seperation = -(yaw - yaw_star)

        return seperation
        
    def odom_callback(self, msg : Odometry):
        if self.state == self.ROTATE:
            twist = Twist()
            seperation = self.get_distance_seperation(msg)

            if abs(seperation) <= self.angle_epsilon:
                self.state = self.GO
                return

            current_time = rospy.get_time() 
            d_time = current_time - self.time
            self.time = current_time
            d_seperation = seperation - self.prev_seperation

            angular_velocity = (self.seperation_KP * seperation) + (self.seperation_KD * d_seperation)
            if angular_velocity < 0:
                twist.angular.z = max(angular_velocity, -self.angular_speed)
            else:
                twist.angular.z = min(angular_velocity, self.angular_speed)

            self.prev_seperation = seperation
            self.cmd_publisher.publish(twist)

        if self.state == self.GO:
            twist = Twist()
            twist.linear.x = min(self.front_dis * 0.5, self.linear_speed)
            self.cmd_publisher.publish(twist)

        
    def scan_callback(self, msg: LaserScan):
        laser_ranges = msg.ranges
        front_ranges = laser_ranges[-5:] + laser_ranges[0:6]
        left_ranges = laser_ranges[86:96]
        behind_ranges = laser_ranges[176:185]
        right_ranges = laser_ranges[266:275] 

        self.front_dis = min(front_ranges)
        self.right_dis = min(right_ranges)
        self.behind_dis = min(behind_ranges)
        self.left_dis = min(left_ranges)

        if self.state != self.ROTATE and self.front_dis < self.min_distance:
            if self.right_dis < self.min_distance and self.left_dis < self.min_distance:
                yaw = self.get_heading()
                self.yaw_star = radians(random.choice([30, -30])) + yaw
                self.state = self.ROTATE
            elif self.left_dis < self.min_distance:
                yaw = self.get_heading()
                self.yaw_star = radians(-30) + yaw
                self.state = self.ROTATE
            else:
                yaw = self.get_heading()
                self.yaw_star = radians(30) + yaw
                self.state = self.ROTATE

            
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    map_builder = MapBuilder()
    map_builder.run()