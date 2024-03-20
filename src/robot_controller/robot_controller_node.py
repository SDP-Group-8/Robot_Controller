#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import tf.transformations as tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class RobotControllerNode:

    def __init__(self):
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.height_goal_pub = rospy.Publisher("/linear_vel", Float32, queue_size=10)

        self.last_odom = Odometry()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.init_node("robot_controller_node", anonymous=True)
        self.height_goal_msg = Float32()

    def odom_callback(self, odom: Odometry):
        self.last_odom = odom

    def set_vertical_velocity(self, height: float):
        self.height_goal = height
        self.height_goal_pub.publish(self.height_goal_msg)

    def set_goal(self, x: float, y: float, theta: float):
        goal_msg = MoveBaseGoal()
        rospy.loginfo(f"Odom is: ({self.last_odom.pose.pose.position.x}, {self.last_odom.pose.pose.position.y})")

        goal_msg.target_pose.pose.position = Point(
            self.last_odom.pose.pose.position.x + x,
            self.last_odom.pose.pose.position.y + y,
            0
        )

        goal_msg.target_pose.header.frame_id = "odom"
        goal_msg.target_pose.header.stamp = rospy.Time.now()

        quaternion = tf.quaternion_from_euler(0, 0, theta)

        goal_msg.target_pose.pose.orientation.x = quaternion[0]
        goal_msg.target_pose.pose.orientation.y = quaternion[1]
        goal_msg.target_pose.pose.orientation.z = quaternion[2]
        goal_msg.target_pose.pose.orientation.w = quaternion[3]

        wait = self.client.wait_for_server(timeout=rospy.Duration(0.5))
        while not wait:
            rospy.logwarn("Waiting for server")
            wait = self.client.wait_for_server(timeout=rospy.Duration(0.5))

        self.client.send_goal(goal_msg)
        wait = self.client.wait_for_result(timeout=rospy.Duration(0.5))
        while not wait:
            rospy.logwarn("Failed to set goal")
            wait = self.client.wait_for_result(timeout=rospy.Duration(0.5))

        return self.client.get_result()
