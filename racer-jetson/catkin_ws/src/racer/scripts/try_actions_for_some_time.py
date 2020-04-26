#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

if __name__ == '__main__':
    rospy.init_node('constant_action', anonymous=True)

    twist_topic = rospy.get_param("~twist_topic", "/racer/commands")
    ackermann_topic = rospy.get_param("~ackermann_topic", "/car_1/command")

    twist = rospy.Publisher(twist_topic, Twist, queue_size=1)
    ackermann = rospy.Publisher(ackermann_topic, AckermannDrive, queue_size=1)

    throttle = 0.1
    steering = 0.2

    twist_msg = Twist()
    twist_msg.linear.x = throttle
    twist_msg.angular.z = steering

    ackermann_msg = AckermannDrive()
    ackermann_msg.steering_angle = steering
    ackermann_msg.speed = throttle

    print(twist_msg)
    print(ackermann_msg)

    rate = rospy.Rate(10)

    n = 0
    while rospy.is_shutdown() is False:
        if n % 100 == 0:
            throttle = throttle + 0.05
            if n % 2000 == 0:
                throttle = 0
                steering = steering + 0.05
                if steering > 1.0:
                    steering = -1.0

        twist_msg.linear.x = throttle
        twist_msg.angular.z = steering
        ackermann_msg.speed = throttle
        ackermann_msg.steering_angle = steering

        twist.publish(twist_msg)
        ackermann.publish(ackermann_msg)
        rate.sleep()

        n = n + 1
