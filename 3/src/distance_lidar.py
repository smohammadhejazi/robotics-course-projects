#!/usr/bin/python3


import rospy
import tf
from time import sleep
from math import radians, sqrt, pow
from hw3_1.msg import ObstacleDistance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Twist
from sensor_msgs.msg import LaserScan
from copy import deepcopy


class DistanceLidar:
    def __init__(self) -> None:
        rospy.init_node("distance_lidar" , anonymous=False)
        self.angle_epsilon = 0.01
        self.angular_speed = 0.4
        self.done = False
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.distance_subscriber = rospy.Subscriber("/ClosestObstacle" , ObstacleDistance, callback=self.distance_callback)

    def get_heading(self):
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return abs(yaw)

    def rotate(self, angle):
        # start rotating   
        remaining = abs(angle)
        prev_angle = self.get_heading()
        twist = Twist()
        if angle < 0:
            twist.angular.z = -self.angular_speed
        else:
            twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
        # stop after loop
        while remaining > self.angle_epsilon:
            self.cmd_publisher.publish(twist)
            current_angle = self.get_heading()

            delta = abs(prev_angle - current_angle)
            remaining -= delta
            prev_angle = current_angle
        twist.angular.z = 0    
        self.cmd_publisher.publish(twist)
        rospy.sleep(1)

    def distance_callback(self, msg : ObstacleDistance):
        rospy.loginfo(self.done)
        if msg.distance < 1.5 and not self.done:
            current_ranges = deepcopy(self.ranges)
            index = current_ranges.index(min(current_ranges))
            rospy.loginfo("rotating: {}".format(index - 180))
            self.rotate(radians(index - 180))
            self.done = True
        elif msg.distance >= 1.5:
            self.done = False
            
    
    def scan_callback(self, msg : LaserScan):
        self.ranges = msg.ranges

            
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    sleep(1)
    distance = DistanceLidar()
    distance.run()
