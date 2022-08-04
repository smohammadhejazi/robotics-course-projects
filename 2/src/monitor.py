#!/usr/bin/python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point
from math import *


class PathMonitor:
    def __init__(self) -> None:
        rospy.init_node("monitor" , anonymous=False)

        self.error = []
        self.error_time = []
        self.x_points = np.array([])
        self.y_points = np.array([])
        self.set_path()
        self.path = Path()
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.path_publisher = rospy.Publisher("/path" , Path , queue_size=10)

    def set_path(self):
        # rectangle

        # X1 = np.linspace(-3, 3 , 100)
        # Y1 = np.array([2]*100)

        # Y2 = np.linspace(2, -2 , 100)
        # X2 = np.array([3]*100)

        # X3 = np.linspace(3, -3 , 100)
        # Y3 = np.array([-2]*100)

        # Y4 = np.linspace(-2, 2 , 100)
        # X4 = np.array([-3]*100)

        # self.x_points = np.concatenate([X1, X2, X3, X4])
        # self.y_points = np.concatenate([Y1, Y2, Y3, Y4])

        # 1 

        # a = 0.17
        # k = tan(a)
        # X , Y = [] , []
        # for i in range(150):
        #     t = i / 20 * pi
        #     dx = a * exp(k * t) * cos(t)
        #     dy = a * exp(k * t) * sin(t)
        #     X.append(dx)
        #     Y.append(dy) 
        # self.x_points = np.array(X)
        # self.y_points = np.array(Y)

        # 2

        # X1 = np.linspace(-6., -2 , 50)
        # Y1 = np.zeros((50,))

        # x_dim, y_dim = 2,2
        # t = np.linspace(np.pi, 0, 100)
        # X2 = x_dim * np.cos(t) 
        # Y2 = y_dim * np.sin(t)

        # X3 = np.linspace(2, 6 , 50)
        # Y3 = np.zeros((50,))

        # x_dim, y_dim = 6,6
        # t = np.linspace(np.pi*2, np.pi, 200)
        # X4 = x_dim * np.cos(t) 
        # Y4 = y_dim * np.sin(t)

        # self.x_points = np.concatenate([X1, X2, X3, X4])
        # self.y_points = np.concatenate([Y1, Y2, Y3, Y4])

        # 3
        
        # growth_factor = 0.1
        # X , Y = [] , []

        # for i in range(400):
        #     t = i / 20 * pi
        #     dx = (1 + growth_factor * t) * cos(t)
        #     dy = (1 + growth_factor * t) * sin(t)
        #     X.append(dx)
        #     Y.append(dy) 

        # self.x_points = np.array(X)
        # self.y_points = np.array(Y)

        # 4

        X1 = np.linspace(-1, 1 , 100)
        Y1 = np.array([3]*100)

        X2 = np.linspace(1, 1 + 2**(1/2) , 100)
        Y2 = - (2**(1/2)) * (X2 - 1) + 3

        Y3 = np.linspace(1, -1 , 100)
        X3 = np.array([1 + 2**(1/2)]*100)

        X4 = np.linspace(1 + 2**(1/2), 1, 100)
        Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 

        X5 = np.linspace(1, -1 , 100)
        Y5 = np.array([-3]*100)

        X6 = np.linspace(-1, -1 - 2**(1/2) , 100)
        Y6 = - (2**(1/2)) * (X6 + 1) - 3 


        Y7 = np.linspace(-1, 1 , 100)
        X7 = np.array([- 1 - 2**(1/2)]*100)


        X8 = np.linspace(-1 - 2**(1/2), -1, 100)
        Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1

        self.x_points = np.concatenate([X1, X2, X3, X4, X5, X6, X7, X8])
        self.y_points = np.concatenate([Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8])
        
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