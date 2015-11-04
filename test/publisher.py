#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64

if __name__ == "__main__":
    # Initialize node.
    rospy.init_node("frontend_test_publisher")

    # Create publisher: name the topic, specify the message type and set the
    # size of the outgoing message queue.
    pub = rospy.Publisher("/test_text_topic", Int64, queue_size=1)

    # Set the publishing rate in Hz.
    rate = rospy.Rate(1)

    # Maintain count.
    count = 0

    # Run until stopped.
    while not rospy.is_shutdown():
        # Create message.
        msg = Int64(count)

        # Increment count.
        count += 1

        # Publish message.
        pub.publish(msg)

        # Wait until it's time to publish again.
        rate.sleep()

