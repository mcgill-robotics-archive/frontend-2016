#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Int64
from std_msgs.msg import Pose
from std_msgs.msg import Quaternion
from std_msgs.msg import Point
from std_msgs.msg import PoseStamped

def createPoseMessage():
  angle = random.random() * 2 * math.pi
  quat = Quaternion(angle, 0, 0, 1)
  return PoseStamped(Header(), Pose(Point(0, 0, 0), Quaternion(quat)))

def testPublish():
  rospy.init_node("frontend_test_publisher")

  # Create publisher for test text topic
  pub = rospy.Publisher("/test_text_topic", Int64, queue_size=1)
  posePub = rospy.Publisher("/test_pose_topic", PoseStamped, queue_size=1)

  # Set test message rate
  rate = rospy.Rate(1)

  count = 0

  # Run until stopped.
  while not rospy.is_shutdown():
    posemsg = createPoseMessage()
    posePub.publish(posemsg)

    msg = Int64(count)
    count += 1
    pub.publish(msg)

    rate.sleep()

if __name__ == "__main__":
  try:
    testPublish()
  except rospy.ROSInterruptException:
    pass

