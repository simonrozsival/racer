#!/usr/bin/env python

import rospy
import math
import numpy as np
from datetime import datetime

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import MarkerArray, Marker

global pts
global delay
global angle
global throttle
global viz_pub
global radii

pts = []
prev = datetime.now()
angle = 0
throttle = 0
viz_pub = None
radii = []

# Function to find the circle on
# which the given three points lie


def findCircle(p1, p2, p3):
    """
    Returns the center and radius of the circle passing the given 3 points.
    In case the 3 points form a line, returns (None, infinity).
    """
    temp = p2[0] * p2[0] + p2[1] * p2[1]
    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

    if abs(det) < 1.0e-6:
        return 0, 0, np.inf

    # Center of circle
    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

    radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
    return cx, cy, radius


def command(msg):
    global angle
    global throttle
    global radii
    if msg.steering_angle != angle or msg.speed != throttle:
        if len(radii) > 0:
            r = np.average(np.array(radii))
            radii = []
            print("{}, {}, {}".format(throttle, angle, r))

        angle = msg.steering_angle
        throttle = msg.speed


def odometry(msg):
    global pts
    global prev
    global viz_pub

    now = datetime.now()
    if (now - prev).total_seconds() > 0.5:
        prev = now
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        pts.append([x, y])
        while len(pts) > 3:
            pts.pop(0)

        if len(pts) == 3:
            cx, cy, r = findCircle(pts[0], pts[1], pts[2])
            radii.append(r)

            if not viz_pub is None:
                viz = MarkerArray()

                i = 0
                for pt in pts:
                    m = Marker()
                    m.action = Marker.ADD
                    m.lifetime = rospy.Duration.from_sec(0.5)
                    m.type = Marker.SPHERE
                    m.pose.position.x = pt[0]
                    m.pose.position.y = pt[1]
                    m.pose.position.z = 0.1
                    m.pose.orientation.w = 1.0
                    m.scale.x = 0.2
                    m.scale.y = 0.2
                    m.scale.z = 0.001
                    m.color.b = 1.0
                    m.color.a = 1.0
                    m.id = i
                    m.header.frame_id = "odom"
                    viz.markers.append(m)
                    i = i + 1

                m = Marker()
                m.action = Marker.ADD
                m.lifetime = rospy.Duration.from_sec(0.5)
                m.type = Marker.SPHERE
                m.pose.position.x = cx
                m.pose.position.y = cy
                m.pose.position.z = 0.05
                m.pose.orientation.w = 1.0
                m.scale.x = 2 * r
                m.scale.y = 2 * r
                m.scale.z = 0.001
                m.color.g = 1.0
                m.color.a = 0.2
                m.header.frame_id = "odom"
                m.id = 3
                viz.markers.append(m)

                viz_pub.publish(viz)


if __name__ == '__main__':

    global viz_pub

    rospy.init_node('steering_angle_meassurement', anonymous=True)

    odom_topic = rospy.get_param(
        "odom_topic", "/car_1/base/odom")
    command_topic = rospy.get_param(
        "command_topic", "/car_1/command")
    viz_topic = rospy.get_param(
        "viz_topic", "/racer/visualization/circle")

    rospy.Subscriber(odom_topic, Odometry, odometry)
    rospy.Subscriber(command_topic, AckermannDrive, command)
    viz_pub = rospy.Publisher(viz_topic, MarkerArray, queue_size=1)

    rospy.spin()
