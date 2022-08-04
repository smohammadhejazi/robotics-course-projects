#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Vector3

class DistanceVector:
    def __init__(self) -> None:
        rospy.init_node("distance_vector" , anonymous=False)
    
        self.goal_point = None
        self.goal_point_subscriber = rospy.Subscriber("/goal_point" , Point , callback=self.goal_point_callback)
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.distance_vector_publisher = rospy.Publisher("/distance_vector" , Vector3 , queue_size=10)
        
    def goal_point_callback(self, goal_point : Point):
        self.goal_point = goal_point    

    def odom_callback(self, msg : Odometry):
        if self.goal_point == None:
            return

        vector = Vector3()
        pose = Pose()
        pose = msg.pose.pose
        vector.x = self.goal_point.x - pose.position.x
        vector.y = self.goal_point.y - pose.position.y
        vector.z = self.goal_point.z - pose.position.z
        self.distance_vector_publisher.publish(vector)
        
if __name__ == "__main__":
    path_monitor = DistanceVector()
    rospy.spin()