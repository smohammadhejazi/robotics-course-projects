#!/usr/bin/python3


import rospy
from math import sqrt
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from hw3_1.srv import GetDistance, GetDistanceResponse


OBSTACLE_DISTANCE_PATH = "/home/mohammad/Desktop/catkin_ws/src/hw3_1/src/obstacle_positions.txt"


class DistanceService:
    def __init__(self) -> None:
        rospy.init_node("distance_service" , anonymous=False)
        self.obstacles = {}
        self.read_distance_file()
        self.x = 0
        self.y = 0
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry, callback=self.odom_callback)
        self.get_distance_service = rospy.Service('GetDistance', GetDistance, self.get_distance_service_callback)
    

    def read_distance_file(self):
        with open(OBSTACLE_DISTANCE_PATH) as file:
            for line in file:
                obstacle_name, position_tuple = line.split("\t")
                position_x, position_y = position_tuple.replace(" ", "").replace("\n", "").strip("()").split(",")
                position_x = float(position_x)
                position_y = float(position_y)
                point = Point()
                point.x = position_x
                point.y = position_y
                self.obstacles[obstacle_name] = point


    def calculate_distance(self, obstacle_point: Point):
        return sqrt(pow(obstacle_point.x - self.x, 2) + pow(obstacle_point.y - self.y, 2))

    
    def odom_callback(self, msg : Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


    def get_distance_service_callback(self, request : GetDistance):
        obstacle_point = self.obstacles[request.obstacle_name]
        distance_to_obstacle = self.calculate_distance(obstacle_point)

        # rospy.loginfo('obstacle: {} distance: {}'
        #     .format(request.obstacle_name, distance_to_obstacle))

        response = GetDistanceResponse()
        response.distance = distance_to_obstacle

        return response

            
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    distance_service = DistanceService()
    distance_service.run()
