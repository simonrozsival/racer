#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

if __name__ == '__main__':
    rospy.init_node('constant_action', anonymous=True)

    ackermann_topic = rospy.get_param("~ackermann_topic", "/car_1/command")
    ackermann = rospy.Publisher(ackermann_topic, AckermannDrive, queue_size=1)

    rate = rospy.Rate(10.0)

    steering = np.arange(0.0, 1.0, 0.1)
    steering = np.append(steering, np.arange(-0.1, -1.0, -0.1))

    while rospy.is_shutdown() is False:
        for s in steering:
            for m in [1.0, 0.75, 0.5]:
                o = [0.0, m]
                for i in range(1, 10):
                    msg = AckermannDrive()
                    msg.steering_angle = s
                    msg.speed = o[i % 2]

                    print("t: {}, s: {}".format(msg.speed, msg.steering_angle))

                    for j in range(200):
                        ackermann.publish(msg)
                        rate.sleep()
