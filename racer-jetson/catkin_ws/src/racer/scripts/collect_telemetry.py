#!/usr/bin/env python

"""
This script/node collects the inputs to the vehicle (most likely from the RC transmitter)
and the odometry of the vehicle and stores it in a CSV file. The input can be realtime
or it could be replayed from a ROS bag.
"""

import rospy
import math
import tf

from sensor_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class State:
    # constants
    max_steering_angle = 24.0 / 180.0 * math.pi # radians
    encoder_to_motor_gear_ratio = 3.5
    wheel_to_motor_gear_ratio = encoder_to_motor_gear_ratio * 3
    wheel_radius = 0.1

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
    previous_total_revolutions = 0
    previous_revolutions_time = float("inf")
    latest_total_revolutions = 0
    latest_revolutions_time = None

    def odometry_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.heading_angle = tf.transformations.euler_from_quaternion(msg.pose.orientation)[2]

        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.speed = math.sqrt(vx**2 + vy**2)
        self.slip_angle = math.atan2(vy, vx)
        self.yaw_rate = msg.twist.angular.z

    def input_callback(self, msg):
        self.throttle_input = msg.linear.x
        self.steering_angle_input = msg.angular.z
        self.steering_angle = self.steering_angle_input * self.max_steering_angle

    def encoder_callback(self, msg):
        self.latest_total_revolutions = msg.data
        self.latest_revolutions_time = rospy.Time.now().to_sec()

    def publish(self):
        if self.has_published is False:
            print("throttle_input,steering_angle_input,x,y,heading_angle,speed,slip_angle,yaw_rate,motor_rpm,steering_angle,estimated_longitudinal_slip")
            self.has_published = True

        self.calculate_motor_rpm()

        line = ",".join([self.throttle_input, self.steering_angle_input, self.x, self.y, self.heading_angle, self.speed, self.slip_angle, self.yaw_rate, self.motor_rpm, self.steering_angle, self.estimate_longitudinal_slip()])
        print(line) # for now, just publish it to STDOUT and suppose that the user redirects it into a file

    def calculate_motor_rpm(self):
        dt = self.latest_revolutions_time - self.previous_revolutions_time
        dr = self.latest_total_revolutions - self.previous_total_revolutions
        rotations_per_second = dr / dt if dt > 0 else 0
        self.motor_rpm = rotations_per_second * 60

        self.previous_revolutions_time = self.latest_revolutions_time
        self.previous_total_revolutions = self.latest_total_revolutions

    def estimate_longitudinal_slip(self):
        wheel_rpm = self.motor_rpm * motor_to_wheel_geer_ratio
        wheel_angular_velocity = wheel_rpm / 30 * math.pi # rad s^-1
        rolling_speed = wheel_angular_velocity * self.wheel_radius

        return (rolling_speed - self.speed) / self.speed

if __name__ == 'main':
    rospy.init_node('collect_telemetry', anonymous=True)

    state = State()

    rospy.Subscriber("/odom", Odometry, state.odometry_callback)
    rospy.Subscriber("/racer/commands", Twist, state.input_callback)
    rospy.Subscriber("/racer/wheel_encoders", Float64, state.encoder_callback)

    rate = rospy.Rate(25)

    while rospy.is_shutdown() is False:
        rospy.spinOnce()
        state.publish()
        rate.sleep()