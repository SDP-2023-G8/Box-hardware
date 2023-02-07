#!/usr/bin/env python
import rospy
import magnet
from time import time, sleep

def pub_accelerometer():
    # Initialize node and publisher
    rospy.init_node('accelerometer_publisher')
    pub = rospy.Publisher('/accelerometer/acc', AccelStamped, queue_size=10)
    rate = rospy.Rate(100) # 100 Hz publishing rate

    # Continuously publish accelerometer readings
    while not rospy.is_shutdown():
        # Create an AccelStamped message
        msg = AccelStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        pub_accelerometer()
    except rospy.ROSInterruptException:
        pass
