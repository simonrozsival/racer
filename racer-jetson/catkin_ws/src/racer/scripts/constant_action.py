#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

if __name__ == '__main__':
    rospy.init_node('constant_action', anonymous=True)

    twist = rospy.Publisher("/racer/commands", Twist, queue_size=1)
    ackermann = rospy.Publisher("/car_1/command", AckermannDrive, queue_size=1)

    throttle = rospy.get_param("~throttle", 0.5)
    steering = rospy.get_param("~steering", 0.8)

    twist_msg = Twist()
    twist_msg.linear.x = throttle
    twist_msg.angular.z = steering

    ackermann_msg = AckermannDrive()
    ackermann_msg.steering_angle = steering
    ackermann_msg.speed = throttle

    print(twist_msg)
    print(ackermann_msg)

    rate = rospy.Rate(25)

    while rospy.is_shutdown() is False:
        twist.publish(twist_msg)
        ackermann.publish(ackermann_msg)
        rate.sleep()
