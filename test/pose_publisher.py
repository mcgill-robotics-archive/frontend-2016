#!/usr/bin/env python

import rospy
import geometry_msgs
from std_msgs.msg import Int64
from tf.transformations import quaternion_from_euler

def testPosePublish():
  rospy.init_node("test_pose")
  # Create publisher for test pose topic
  pubPose = rospy.Publisher("/test_pose_topic", geometry_msgs.msg.PoseStamped, queue_size=1)

  # Set test message rate
  rate = rospy.Rate(1)

  # create message
  pose = geometry_msgs.msg.PoseStamped()
  pose.header.stamp = rospy.Time.now()
  pose.header.frame_id = "test_pose_stuff"      
  pose.pose.position.x = 0
  pose.pose.position.y = 0
  pose.pose.position.z = 0
  z = 0

  while not rospy.is_shutdown():
    z += .15
    q = quaternion_from_euler(0,0,z)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    pubPose.publish(pose)
    rate.sleep()

if __name__ == "__main__":
  try:
    testPosePublish()
  except rospy.ROSInterruptException:
    pass
