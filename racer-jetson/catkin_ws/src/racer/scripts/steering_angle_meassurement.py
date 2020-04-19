#!/usr/bin/env python

import rospy
import math
from datetime import datetime

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive

global pts
global delay
global angle

pts = []
prev = datetime.now()
angle = 0

# Function to find the circle on
# which the given three points lie


def findCircle(x1, y1, x2, y2, x3, y3):
    x12 = x1 - x2
    x13 = x1 - x3

    y12 = y1 - y2
    y13 = y1 - y3

    y31 = y3 - y1
    y21 = y2 - y1

    x31 = x3 - x1
    x21 = x2 - x1

    # x1^2 - x3^2
    sx13 = pow(x1, 2) - pow(x3, 2)

    # y1^2 - y3^2
    sy13 = pow(y1, 2) - pow(y3, 2)

    sx21 = pow(x2, 2) - pow(x1, 2)
    sy21 = pow(y2, 2) - pow(y1, 2)

    f = (((sx13) * (x12) + (sy13) *
          (x12) + (sx21) * (x13) +
          (sy21) * (x13)) // (2 *
                              ((y31) * (x12) - (y21) * (x13))))

    g = (((sx13) * (y12) + (sy13) * (y12) +
          (sx21) * (y13) + (sy21) * (y13)) //
         (2 * ((x31) * (y12) - (x21) * (y13))))

    c = (-pow(x1, 2) - pow(y1, 2) -
         2 * g * x1 - 2 * f * y1)

    # eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
    # where centre is (h = -g, k = -f) and
    # radius r as r^2 = h^2 + k^2 - c
    h = -g
    k = -f
    sqr_of_r = h * h + k * k - c

    # r is the radius
    r = round(math.sqrt(sqr_of_r), 5)

    print("Centre = (", h, ", ", k, ")")
    print("Radius = ", r)


def command(msg):
    global angle
    if msg.steering_angle != angle:
        print("Next steering angle: {}".format(msg.steering_angle))
        angle = msg.steering_angle


def odometry(msg):
    global pts
    global prev

    now = datetime.now()
    if (now - prev).total_seconds() > 1.0:
        prev = now
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        pts.append([x, y])
        if len(pts) > 3:
            pts.pop(0)
            findCircle(x, y, pts[0][0], pts[0][1], pts[1][0], pts[1][1])


if __name__ == '__main__':

    rospy.init_node('steering_angle_meassurement', anonymous=True)

    odom_topic = rospy.get_param(
        "odom_topic", "/car_1/base/odom")
    command_topic = rospy.get_param(
        "command_topic", "/car_1/command")

    rospy.Subscriber(odom_topic, Odometry, odometry)
    rospy.Subscriber(command_topic, AckermannDrive, command)

    rospy.spin()
