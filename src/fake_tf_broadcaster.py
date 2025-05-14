#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def broadcast():
    rospy.init_node('fake_tf_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        br.sendTransform(t)
        rate.sleep()

if __name__ == '__main__':
    broadcast()
