#!/usr/bin/python3


from cmath import log
from signal import signal
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

        self.x = 0
        self.y = -8
        self.goal_point_x = -8
        self.goal_point_y = -8
        self.goal_sector = 0
        self.yaw_star = 0


        self.distance_epsilon = 0.01
        self.angle_epsilon = 0.01
        self.angle_epsilon = 0.01
        self.angular_speed = 0.6
        self.linear_speed = 0.1

        self.min_distance = 4
        self.a = 1
        self.b = 1 / self.min_distance
        self.threshold = 0.6

        self.distance_KP = 1
        self.distance_KI = 0
        self.distance_KD = 0
        self.seperation_KP = 20
        self.seperation_KI = 0
        self.seperation_KD = 40
        self.distance_integral = 0
        self.seperation_integral = 0
        self.time = rospy.get_time()
        self.prev_distance  = 0
        self.prev_seperation = 0
        self.d_star = -0.2

        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry, callback=self.odom_callback)

    # [radian(-180), radian(180)]
    def get_heading(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw 

    def rotate_to_sector(self, sector_index):
        if sector_index < 36:
            self.rotate(radians(5 * sector_index))
        else:
            self.rotate(radians(5 * (sector_index - 72)))

    def get_distance_seperation(self, msg : Odometry):
        current_point = msg.pose.pose.position
        current_orientation = msg.pose.pose.orientation

        self.x = current_point.x
        self.y = current_point.y

        # distance
        x_diff = self.goal_point_x - current_point.x
        y_diff = self.goal_point_y - current_point.y
        distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2)) + self.d_star
        if distance < 0:
            distance = 0

        # seperation
        yaw_star = self.yaw_star

        if yaw_star > radians(180):
            yaw_star -= radians(360)

        seperation = radians(yaw_star)


        return distance, seperation

    def set_next_goal(self):
        self.goal_point_x = -8
        self.goal_point_y = -8
        
    def odom_callback(self, msg : Odometry):
        twist = Twist()
        distance, seperation = self.get_distance_seperation(msg)
        current_time = rospy.get_time() 
        d_time = current_time - self.time
        self.time = current_time
        self.distance_integral += d_time * distance
        self.seperation_integral += d_time * seperation
        d_distance = distance - self.prev_distance 
        d_seperation = seperation - self.prev_seperation

        if distance <= self.distance_epsilon:
            self.set_next_goal()
            self.distance_integral = 0
            self.seperation_integral = 0
        else:
            linear_speed = (self.distance_KP * distance) + (self.distance_KI * self.distance_integral) + (self.distance_KD * d_distance)
            if linear_speed < 0:
                twist.linear.x = max(linear_speed, -self.linear_speed)       
            else:
                twist.linear.x = min(linear_speed, self.linear_speed)

            angular_velocity = (self.seperation_KP * seperation) + (self.seperation_KI * self.seperation_integral) + (self.seperation_KD * d_seperation)
            if angular_velocity < 0:
                twist.angular.z = max(angular_velocity, -self.angular_speed)
            else:
                twist.angular.z = min(angular_velocity, self.angular_speed)

            self.prev_distance = distance
            self.prev_seperation = seperation

            self.cmd_publisher.publish(twist)

    def buildSectors(self, laser_ranges):
        sectors = []
        # 360 / 5 = 72 total sectors
        for i in range(72):
            # each degree
            newSector = []
            for j in range(5):
                value = laser_ranges[i * 5 + j]
                value = self.a - self.b * value
                value = 0 if value < 0 else value
                newSector.append(value)
            sectors.append(newSector)
        return sectors

    def get_densities(self, sectors):
        densities = []
        for sector in sectors:
            density = 0
            for value in sector:
                density += value
            densities.append(density)
        return densities

    
    def smooth_sectors(self, densities):
        c_densities = densities.copy()
        length = len(c_densities)
        for i in range(length):
            prev_i = i - 1 if i >= 1 else length - 1
            next_i = i + 1 if i < length - 1 else 0
            densities[i] = (c_densities[prev_i] + 2 * c_densities[i] + c_densities[next_i]) / 5
        return densities
        

    def get_goal_sector(self):
        degree = degrees(atan2(self.goal_point_y - self.y, self.goal_point_x - self.x))
        degree = degree + 360 if degree < 0 else degree
        
        robot_heading = degrees(self.get_heading())
        robot_heading = robot_heading + 360 if robot_heading < 0 else robot_heading

        delta = degree - robot_heading
        delta = delta + 360 if delta < 0 else delta

        for i in range(72):
            if i * 5 <= delta < (i + 1) * 5:
                # rospy.loginfo("degree: {}, robot_heading: {}, delta: {}, i: {}".format(degree, robot_heading, delta, i))
                return i

    def get_vallies(self, smooth_densities):
        vallies = []
        idx = 0
        while idx < 72:
            valley = []
            while idx < 72:
                if smooth_densities[idx] < self.threshold:
                    valley.append(idx)
                    idx += 1
                else:
                    if len(valley) > 0:
                        vallies.append(valley)
                    break
            if idx == 72 and len(valley) > 0:
                vallies.append(valley)
            idx += 1

        if len(vallies) >=2 and vallies[0][0] == 0 and vallies[-1][-1] == 72 - 1:
            first = vallies.pop(0)
            last = vallies.pop()
            combined = last + first
            vallies.insert(0, combined)

        return vallies

    def get_heading_angle(self, goal_sector_index, vallies):
        # if goal sector is in valley, choose that sector
        for i, valley in enumerate(vallies):
            if goal_sector_index in valley:
                return goal_sector_index * 5 + 2.5

        
        min_d = 99999
        for i, valley in enumerate(vallies):
            d1 = abs(valley[0] - goal_sector_index)
            d2 = abs(valley[-1] - goal_sector_index)
            d3 = abs(72 - d1)
            d4 = abs(72 - d2)
            min_d_v = min(d1, d2, d3, d4)
            if min_d_v < min_d:
                min_d = min_d_v
                chosen_valley_idx = i

        chosen_valley = vallies[chosen_valley_idx]
        # rospy.loginfo("chosen idx: {}".format(chosen_valley_idx))
        # rospy.loginfo("chosen idx: {}".format(chosen_valley))
        if len(chosen_valley) % 2 == 0:
            return chosen_valley[int(len(chosen_valley) / 2)] * 5
        else:
            return chosen_valley[int(floor(len(chosen_valley) / 2))] * 5 + 2.5
        

        
    def scan_callback(self, msg: LaserScan):
        laser_ranges = msg.ranges
        sectors = self.buildSectors(laser_ranges)
        densities = self.get_densities(sectors)
        smooth_densities = self.smooth_sectors(densities)
        goal_sector_index = self.get_goal_sector()
        vallies = self.get_vallies(smooth_densities)

        # [0, radian(360) )
        heading_angle = self.get_heading_angle(goal_sector_index, vallies)
        self.yaw_star = radians(heading_angle)

            
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    map_builder = MapBuilder()
    map_builder.run()