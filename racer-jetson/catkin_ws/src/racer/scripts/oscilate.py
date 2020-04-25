#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

if __name__ == '__main__':
    rospy.init_node('constant_action', anonymous=True)

    ackermann_topic = rospy.get_param("~ackermann_topic", "/car_1/command")
    ackermann = rospy.Publisher(ackermann_topic, AckermannDrive, queue_size=1)

    options = [
        [-1, 1],
        [-0.9, 0.9],
        [-0.8, 0.8],
        [-0.7, 0.7],
        [-0.6, 0.6],
        [-0.5, 0.5],
        [-0.4, 0.4],
        [-0.3, 0.3],
        [-0.2, 0.2],
        [-0.1, 0.1],
    ]

    rate = rospy.Rate(1.0 / 10.0)

    i = 0
    while rospy.is_shutdown() is False:
        for o in options:
            for k in range(1, 10):
                msg = AckermannDrive()
                msg.steering_angle = o[i % 2]
                msg.speed = 0.0

                ackermann.publish(msg)
                i = i + 1

                rate.sleep()
