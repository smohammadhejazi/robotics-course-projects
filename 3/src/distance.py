#!/usr/bin/python3


from time import sleep
import rospy
from math import sqrt
from hw3_1.msg import ObstacleDistance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point


OBSTACLE_DISTANCE_PATH = "/home/mohammad/Desktop/catkin_ws/src/hw3_1/src/obstacle_positions.txt"


class Distance:
    def __init__(self) -> None:
        rospy.init_node("distance" , anonymous=False)
        self.obstacles = {}
        self.read_distance_file()
        self.closest_obstacle_publisher = rospy.Publisher('/ClosestObstacle' , ObstacleDistance , queue_size=10)
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry, callback=self.odom_callback)


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


    def calculate_minimum_distance(self, robot_point: Point):
        min_obstacle = None
        min_distance = None
        for obstacle, point in self.obstacles.items():
            distance = sqrt(pow(robot_point.x - point.x, 2) + pow(robot_point.y - point.y, 2))
            if min_obstacle is None:
                min_obstacle = obstacle
                min_distance = distance
            elif distance < min_distance:
                min_obstacle = obstacle
                min_distance = distance
        return min_obstacle, min_distance


    def odom_callback(self, msg : Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        min_obstacle, min_distance = self.calculate_minimum_distance(pose.pose.position)
        
        obstacleDistance = ObstacleDistance(min_obstacle, min_distance)
        self.closest_obstacle_publisher.publish(obstacleDistance)
        rospy.loginfo('obstacle: {} distance: {}'
            .format(obstacleDistance.obstacle_name, obstacleDistance.distance))

            
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    sleep(0.2)
    distance = Distance()
    distance.run()
