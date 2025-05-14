#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class PIDController:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback)
        self.target_yaw = math.radians(90)  # Target: 90 degrees
        self.Kp = rospy.get_param("~Kp", 1.0)
        self.Ki = rospy.get_param("~Ki", 0.0)
        self.error_sum = 0.0
        self.last_time = rospy.Time.now()

    def callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        yaw = self.get_yaw_from_quaternion(orientation_q)

        error = self.target_yaw - yaw
        self.error_sum += error
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        control = self.Kp * error + self.Ki * self.error_sum * dt

        twist = Twist()
        twist.angular.z = max(min(control, 0.5), -0.5)  # Limit angular speed
        self.pub.publish(twist)

        if abs(error) < 0.01:
            rospy.loginfo("Target heading reached.")
            twist.angular.z = 0.0
            self.pub.publish(twist)
            rospy.signal_shutdown("Done")

    def get_yaw_from_quaternion(self, q):
        import tf
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

if __name__ == '__main__':
    rospy.init_node('pid_heading_controller')
    PIDController()
    rospy.spin()
