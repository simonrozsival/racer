#!/usr/bin/env python

"""
This script/node collects the inputs to the vehicle (most likely from the RC transmitter)
and the odometry of the vehicle and stores it in a CSV file. The input can be realtime
or it could be replayed from a ROS bag.
"""

import rospy
import math
import tf
import time

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64


class State:
    # constants
    max_steering_angle = math.radians(24.0)
    encoder_to_motor_gear_ratio = 3.5
    wheel_to_motor_gear_ratio = encoder_to_motor_gear_ratio * 3
    wheel_radius = 0.05

    # the control inputs
    throttle_input = 0
    steering_angle_input = 0

    # the state of actuators
    motor_rpm = 0
    steering_angle = 0

    # the configuration of the ve=hicle
    x = 0
    y = 0
    heading_angle = 0

    # the velocity of the vehicle
    speed = 0
    slip_angle = 0
    yaw_rate = 0

    # inner state
    has_published = False

    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        o = msg.pose.pose.orientation
        quat = (o.x, o.y, o.z, o.w)
        self.heading_angle = tf.transformations.euler_from_quaternion(quat)[2]

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.sqrt(vx**2 + vy**2)
        self.slip_angle = math.atan2(vy, vx)
        self.yaw_rate = msg.twist.twist.angular.z

    def input_callback(self, msg):
        self.throttle_input = msg.speed
        self.steering_angle_input = msg.steering_angle
        self.steering_angle = self.steering_angle_input * self.max_steering_angle

    def rpm_callback(self, msg):
        self.motor_rpm = msg.data

    def publish(self):
        if self.has_published is False:
            print("time,throttle_input,steering_angle_input,x,y,heading_angle,speed,slip_angle,yaw_rate,motor_rpm,steering_angle,estimated_longitudinal_slip")
            self.has_published = True

        values = [time.time(), self.throttle_input, self.steering_angle_input, self.x, self.y, self.heading_angle, self.speed,
                  self.slip_angle, self.yaw_rate, self.motor_rpm, self.steering_angle, self.estimate_longitudinal_slip()]
        line = ",".join([str(x) for x in values])
        # for now, just publish it to STDOUT and suppose that the user redirects it into a file
        print(line)

    def estimate_longitudinal_slip(self):
        wheel_rpm = self.motor_rpm / self.wheel_to_motor_gear_ratio
        wheel_angular_velocity = wheel_rpm / 30 * math.pi  # rad s^-1
        rolling_speed = wheel_angular_velocity * self.wheel_radius

        if self.speed == 0:
            return 0

        return (rolling_speed - self.speed) / self.speed


if __name__ == '__main__':
    rospy.init_node('collect_telemetry', anonymous=True)

    state = State()

    rospy.Subscriber("/car_1/base/odom", Odometry, state.odometry_callback)
    rospy.Subscriber("/car_1/command", AckermannDrive, state.input_callback)
    rospy.Subscriber("/racer/motor_rpm", Float64, state.rpm_callback)

    rate = rospy.Rate(25)

    while rospy.is_shutdown() is False:
        state.publish()
        rate.sleep()
