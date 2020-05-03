#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

global speed_history
global window
global pub

speed_history = []
window = 5


def odometry_callback(msg):
    global speed_history

    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    speed = math.sqrt(vx ** 2 + vy ** 2)
    speed_history.append(speed)

    if len(speed_history) > window:
        speed_history = speed_history[-window:]

        avg_speed = sum(speed_history) / len(speed_history)

        wheel_diameter = 0.1
        gear_ratio = 18
        wheel_circumference = math.pi * wheel_diameter
        wheel_rotations_per_second = avg_speed / wheel_circumference
        wheel_rpm = 60 * wheel_rotations_per_second
        motor_rpm = wheel_rpm * gear_ratio

        msg = Float64()
        msg.data = motor_rpm
        pub.publish(msg)


def wheel_encoder(
    ground_truth_topic,
    pub_topic
):
    global pub
    rospy.Subscriber(ground_truth_topic, Odometry, odometry_callback)
    pub = rospy.Publisher(pub_topic, Float64, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('wheel_encoder_from_gazebo')
        wheel_encoder(
            rospy.get_param('~odom_topic', '/car_1/ground_truth'),
            rospy.get_param('~topic', '/racer/motor_rpm')
        )
    except rospy.ROSInterruptException:
        pass
