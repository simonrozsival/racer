#!/usr/bin/env python

import rospy
import math
from datetime import datetime
import time

from control_msgs.msg import JointControllerState

global left_prev
global left_target
global left_start
global left_reached

global right_prev
global right_target
global right_start
global right_reached

left_start = 0
left_prev = 0
left_target = 0
left_reached = False

right_start = 0
right_prev = 0
right_target = 0
right_reached = False


def left_update(state):
    global left_prev
    global left_target
    global left_start
    global left_reached

    # if left_target != state.set_point:
    #     left_prev = state.process_value
    #     left_target = state.set_point
    #     left_start = datetime.now()
    #     left_reached = False
    # elif not left_reached and abs(state.error) < 0.05:
    #     left_reached = True
    #     duration = datetime.now() - left_start
    #     print("left, {}, {}, {}".format(
    #         left_prev, state.process_value, duration.total_seconds()))
    print("left,{},{}".format(time.time(), state.process_value))


def right_update(state):
    global right_prev
    global right_target
    global right_start
    global right_reached

    # if right_target != state.set_point:
    #     right_prev = state.process_value
    #     right_target = state.set_point
    #     right_start = datetime.now()
    #     right_reached = False
    # elif not right_reached and abs(state.error) < 0.05:
    #     right_reached = True
    #     duration = datetime.now() - right_start
    #     print("right, {}, {}, {}".format(
    #         right_prev, state.process_value, duration.total_seconds()))
    print("right,{},{}".format(time.time(), state.process_value))


if __name__ == '__main__':

    rospy.init_node('measure_steering_adjustment', anonymous=True)

    left_topic = rospy.get_param(
        "left", "/car_1/right_steering_hinge_position_controller/state")
    right_topic = rospy.get_param(
        "right", "/car_1/left_steering_hinge_position_controller/state")

    # print("wheel,from,to,s")
    # print("wheel,timestamp,error")
    print("wheel,timestamp,state")

    rospy.Subscriber(
        left_topic, JointControllerState, left_update)

    rospy.Subscriber(
        right_topic, JointControllerState, right_update)

    rospy.spin()
