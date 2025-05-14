#!/usr/bin/env python3
import rospy
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def random_goal():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base!")

    success_count = 0
    fail_count = 0

    while not rospy.is_shutdown() and fail_count < 3:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = random.uniform(-0.5, 0.5)
        goal.target_pose.pose.position.y = random.uniform(-0.5, 0.5)
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: x={goal.target_pose.pose.position.x:.2f}, y={goal.target_pose.pose.position.y:.2f}")
        client.send_goal(goal)
        client.wait_for_result()

        if client.get_result():
            rospy.loginfo("Goal reached!")
            success_count += 1
        else:
            rospy.logwarn("Goal failed.")
            fail_count += 1

if __name__ == '__main__':
    rospy.init_node('random_goal_sender')
    random_goal()
