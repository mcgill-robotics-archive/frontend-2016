#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64

def testPublish():
    rospy.init_node("frontend_test_publisher")

    # Create publisher for test text topic
    pub = rospy.Publisher("/test_text_topic", Int64, queue_size=1)

    # Set test message rate
    rate = rospy.Rate(1)

    count = 0

    # Run until stopped.
    while not rospy.is_shutdown():
        msg = Int64(count)
        count += 1
        pub.publish(msg)

    rate.sleep()

if __name__ == "__main__":
    try:
        testPublish()
    except rospy.ROSInterruptException:
        pass

