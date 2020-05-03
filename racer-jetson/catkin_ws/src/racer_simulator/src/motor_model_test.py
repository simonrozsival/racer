#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDrive

global max_rpm
global x

max_rpm = 18000
x = [7.99889787e+02, 8.00123797e+02, 1.51154446e-02, 5.83921900e+00,
     3.79599215e+00, 2.68911294e+00]


global measured_rpm
global throttle
global steering

measured_rpm = 0
throttle = 0
steering = 0


def command_msg(msg):
    global throttle
    global steering
    throttle = msg.speed
    steering = msg.steering_angle


def rpm_msg(msg):
    global measured_rpm
    measured_rpm = msg.data


def max_torque(current_rpm):
    global x
    return (1 - current_rpm) * x[0]


def drive_torque(current_rpm, throttle):
    return max_torque(current_rpm) * throttle


def load_torque(current_rpm, steering_angle):
    global x
    return (current_rpm ** x[2]) * ((x[3] + x[4] * abs(steering_angle)) ** x[5])


def predict_new_rpm(rpm, dt):
    global throttle
    global steering
    global x
    global max_rpm

    norm_rpm = rpm / max_rpm
    normalized_drive_torque = drive_torque(norm_rpm, throttle)
    normalized_load_torque = load_torque(norm_rpm, steering)
    normalized_rpm_change_rate = (
        normalized_drive_torque - normalized_load_torque) / x[1]

    return max(0.0,
               rpm + normalized_rpm_change_rate * max_rpm * dt)


if __name__ == '__main__':
    try:
        rospy.init_node('motor_model_test')

        rospy.Subscriber("/car_1/command", AckermannDrive, command_msg)
        rospy.Subscriber("/racer/motor_rpm", Float64, rpm_msg)

        rate = rospy.Rate(25)

        rate.sleep()
        rate.sleep()
        rate.sleep()

        # print("throttle,steering,rpm")
        rpm = measured_rpm

        while not rospy.is_shutdown():

            # predict the next RPM
            dt = 1.0 / 25.0
            rpm = predict_new_rpm(rpm, dt)
            #

            # print("predicted: {} RPM".format(rpm))
            # print("measured:  {} RPM".format(measured_rpm))
            print("error:     {} RPM".format(measured_rpm - rpm))
            # print("")

            # print("{},{},{}".format(throttle, steering, measured_rpm))

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
