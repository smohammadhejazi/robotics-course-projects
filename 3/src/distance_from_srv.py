#!/usr/bin/python3


from time import sleep
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from hw3_1.msg import ObstacleDistance
from hw3_1.srv import GetDistance, GetDistanceResponse


OBSTACLE_DISTANCE_PATH = "/home/mohammad/Desktop/catkin_ws/src/hw3_1/src/obstacle_positions.txt"


class DistanceFromSrv:
    def __init__(self) -> None:
        rospy.init_node("distance_from_srv" , anonymous=False)
        self.obstacles = {}
        self.read_distance_file()
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry, callback=self.odom_callback)
        self.closest_obstacle_publisher = rospy.Publisher('/ClosestObstacle' , ObstacleDistance , queue_size=10)
        rospy.wait_for_service("GetDistance")
        try:
            self.get_distance_proxy = rospy.ServiceProxy('GetDistance', GetDistance)
        except rospy.ServiceException as e:
            print(e)


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

    def calculate_minimum_distance(self):
        min_obstacle = None
        min_distance = None
        for obstacle in self.obstacles:
            response = self.get_distance_proxy(obstacle)
            distance = response.distance

            if min_distance is None:
                min_obstacle = obstacle
                min_distance = distance
            elif distance < min_distance:
                min_obstacle = obstacle
                min_distance = distance

        return min_obstacle, min_distance


    def odom_callback(self, msg : Odometry):
        min_obstacle, min_distance = self.calculate_minimum_distance()
        obstacle_object = ObstacleDistance(min_obstacle, min_distance)
        self.closest_obstacle_publisher.publish(obstacle_object)
        rospy.loginfo('closest obstacle: {} distance: {}'
            .format(obstacle_object.obstacle_name, obstacle_object.distance))

            
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    sleep(0.5)
    distance = DistanceFromSrv()
    distance.run()
