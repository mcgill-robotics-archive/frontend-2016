#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PointStamped


def testPublish():
    rospy.init_node("frontend_test_publisher")

    # Create publisher for test text topic
    pub = rospy.Publisher("/test_plot_topic", PointStamped, queue_size=1)
    rate = rospy.Rate(3)
    count = 0

    while not rospy.is_shutdown():
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.point.x = count
        point.point.y = math.sin(count / 10.0)
        count += 1
        pub.publish(point)

        rate.sleep()


if __name__ == "__main__":
    try:
        testPublish()
    except rospy.ROSInterruptException:
        pass

