#!/usr/bin/env python
import rospy
from geometry_msgs.msg import AccelStamped

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
  
        msg.accel.linear.x = get_acceleration_x()
        msg.accel.linear.y = get_acceleration_y()
        msg.accel.linear.z = get_acceleration_z()
        
        pub.publish(msg)
        rate.sleep()

def get_acceleration_x():
    
    return 0.0

def get_acceleration_y():
    
    return 0.0

def get_acceleration_z():
    
    return 9.8

if __name__ == '__main__':
    try:
        pub_accelerometer()
    except rospy.ROSInterruptException:
        pass
