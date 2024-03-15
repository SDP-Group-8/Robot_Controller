#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import tf.transformations as tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class RobotControllerNode:

    def __init__(self):
        self.height_goal_pub = rospy.Publisher("/linear_vel", Float32, queue_size=10)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.init_node("robot_controller_node", anonymous=True)
        self.height_goal_msg = Float32()

    def set_vertical_velocity(self, height: float):
        self.height_goal = height
        self.height_goal_pub.publish(self.height_goal_msg)

    def set_goal(self, x: float, y: float, theta: float):
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y

        goal_msg.target_pose.header.frame_id = "odom"
        goal_msg.target_pose.header.stamp = rospy.Time.now()

        quaternion = tf.quaternion_from_euler(theta, 0, 0)

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
