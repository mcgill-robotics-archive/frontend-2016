#!/usr/bin/env python

import rospy
import geometry_msgs
from tf.transformations import quaternion_from_euler


def testPosePublish():
    rospy.init_node("test_pose")
    # Create publisher
    posePublisher = rospy.Publisher("/test_pose_topic",
                              geometry_msgs.msg.PoseStamped, queue_size=1)
    rate = rospy.Rate(1)

    # Create message and set properties
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "test_pose_msg"
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    z = 0

    while not rospy.is_shutdown():
        z += 0.15
        q = quaternion_from_euler(0,0,z)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        posePublisher.publish(pose)
        rate.sleep()


if __name__ == "__main__":
    try:
        testPosePublish()
    except rospy.ROSInterruptException:
        pass
