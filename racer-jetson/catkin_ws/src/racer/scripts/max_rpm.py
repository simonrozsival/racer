#!/usr/bin/env python

"""
This script/node collects the inputs to the vehicle (most likely from the RC transmitter)
and the odometry of the vehicle and stores it in a CSV file. The input can be realtime
or it could be replayed from a ROS bag.
"""

import rospy
import math
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

previous_total_revolutions = 0
previous_revolutions_time = float("inf")
latest_total_revolutions = 0
latest_revolutions_time = 0
max_rpm = 0

def encoder_callback(msg):
    global latest_total_revolutions
    global latest_revolutions_time

    latest_total_revolutions = msg.data
    latest_revolutions_time = rospy.Time.now().to_sec()

rospy.init_node('max_rpm', anonymous=True)

rospy.Subscriber("/racer/wheel_encoders", Float64, encoder_callback)

rate = rospy.Rate(25)

while rospy.is_shutdown() is False:
    # note to future self: threre is no need for an equivalent of 'spin once' in rospy

    dt = latest_revolutions_time - previous_revolutions_time
    dr = latest_total_revolutions - previous_total_revolutions
    previous_revolutions_time = latest_revolutions_time
    previous_total_revolutions = latest_total_revolutions

    rpm = 3 * 60 * dr / dt if dt > 0 else 0
    if rpm > max_rpm:
        max_rpm = rpm
    #    print(max_rpm)
    print(rpm)

    rate.sleep()
