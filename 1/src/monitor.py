#!/usr/bin/python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from math import pow, sqrt


class PathMonitor:
    def __init__(self) -> None:
        rospy.init_node("monitor" , anonymous=False)
        
        self.length = rospy.get_param("/controller/length")
        self.width = rospy.get_param("/controller/width")

        self.set_path()
        self.error = []
        self.error_time = []
        self.path = Path()
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.path_publisher = rospy.Publisher("/path" , Path , queue_size=10)


    def set_path(self):
        half_length = self.length / 2
        half_width = self.width / 2

        x1 = np.linspace(-half_length, half_length , 100)
        y1 = np.array([half_width]*100)

        y2 = np.linspace(half_width, -half_width , 100)
        x2 = np.array([half_length ]*100)

        x3 = np.linspace(-half_length, half_length , 100)
        y3 = np.array([-half_width]*100)

        y4 = np.linspace(-half_width, half_width , 100)
        x4 = np.array([-half_length]*100)

        self.x_points = np.concatenate([x1, x2, x3, x4])
        self.y_points = np.concatenate([y1, y2, y3, y4])

    def get_error(self, point: Point):
        min_error = min([sqrt(pow(self.x_points[i] - point.x, 2) + pow(self.y_points[i] - point.y, 2)) for i in range(self.x_points.size)])
        return min_error


    def show_error(self):
        
        plt.xlabel('time')
        plt.ylabel('error')
        plt.plot(self.error_time, self.error) 
        plt.show()

    def odom_callback(self, msg : Odometry):
        self.path.header = msg.header
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.error_time.append(rospy.get_time())
        self.error.append(self.get_error(msg.pose.pose.position))
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

           
if __name__ == "__main__":
    path_monitor = PathMonitor()
    rospy.on_shutdown(path_monitor.show_error)
    rospy.spin()