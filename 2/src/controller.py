#!/usr/bin/python3


import rospy
import tf
import numpy as np
from math import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point


class Controller:
    def __init__(self) -> None:
        rospy.init_node("controller" , anonymous=False)

        # subcribe publish
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed")
        self.angular_speed = rospy.get_param("/controller/angular_speed")
        self.distance_epsilon = rospy.get_param("/controller/distance_epsilon")
        self.angle_epsilon = rospy.get_param("/controller/angle_epsilon")

        # goal params
        self.twist = Twist()

        self.goal_point_index = 0
        self.goal_point = Point()
        self.distance_KP = 0.5
        self.distance_KI = 0.0001
        self.distance_KD = 0.001
        self.seperation_KP = 1
        self.seperation_KI = 0
        self.seperation_KD = 0
        self.distance_integral = 0
        self.seperation_integral = 0
        self.d_star = -0.02
        self.time = rospy.get_time()
        self.d_distance = 0
        self.d_seperation = 0
        self.x_points = np.array([])
        self.y_points = np.array([])

        self.set_path()
        self.set_first_goal()

    def set_next_goal(self):
        goal_point = Point()
        self.goal_point_index = (self.goal_point_index + 1) % self.x_points.size
        goal_point.x = self.x_points[self.goal_point_index]
        goal_point.y = self.y_points[self.goal_point_index]
        self.goal_point = goal_point
    
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


    def set_first_goal(self):
        goal_point = Point()
        msg = rospy.wait_for_message("/odom" , Odometry)
        pose = msg.pose.pose.position
        errors = [sqrt(pow(self.x_points[i] - pose.x, 2) + pow(self.y_points[i] - pose.y, 2)) for i in range(self.x_points.size)]
        closest_index = min(enumerate(errors), key=lambda x: x[1])[0]
        goal_point.x = self.x_points[closest_index]
        goal_point.y = self.y_points[closest_index]
        self.goal_point_index = closest_index
        self.goal_point = goal_point

    def get_distance_seperation(self, msg : Odometry):
        current_point = msg.pose.pose.position
        current_orientation = msg.pose.pose.orientation

        # distance
        pose = msg.pose.pose
        x_diff = self.goal_point.x - current_point.x
        y_diff = self.goal_point.y - current_point.y
        # z_diff = self.goal_point.z - current_point.z
        distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2)) + self.d_star
        if distance < 0:
            distance = 0

        # seperation
        yaw_star = atan2(self.goal_point.y - current_point.y, self.goal_point.x - current_point.x)
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

        return distance, seperation

    def odom_callback(self, msg : Odometry):
        twist = Twist()
        distance, seperation = self.get_distance_seperation(msg)
        current_time = rospy.get_time()   
        d_time = current_time - self.time
        self.time = current_time
        self.distance_integral += d_time * distance
        self.seperation_integral += d_time * seperation
        d_distance = distance - self.d_distance 
        d_seperation = seperation - self.d_seperation

        if distance <= self.distance_epsilon:
            self.set_next_goal()
        else:
            linear_speed = (self.distance_KP * distance) + (self.distance_KI * self.distance_integral) + (self.distance_KD * d_distance)
            if linear_speed < 0:
                twist.linear.x = max(linear_speed, -self.linear_speed)       
            else:
                twist.linear.x = min(linear_speed, self.linear_speed)

            rospy.loginfo("linear_speed: {} x: {}".format(linear_speed, twist.linear.x))

            angular_velocity = (self.seperation_KP * seperation) + (self.seperation_KI * self.seperation_integral) + (self.seperation_KD * d_seperation)
            if angular_velocity < 0:
                twist.angular.z = max(angular_velocity, -self.angular_speed)
            else:
                twist.angular.z = min(angular_velocity, self.angular_speed)

            self.d_distance = d_distance
            self.d_seperation = d_seperation

            self.cmd_publisher.publish(twist)

if __name__ == "__main__":
    controller = Controller()
    rospy.spin()