#!/usr/bin/env python

import rospy
import math
from datetime import datetime

from control_msgs.msg import JointControllerState

global prev
global target
global start
global reached

prev = 0
target = 0
start = 0
reached = False


def update(state):
    global prev
    global target
    global start
    global reached

    if target != state.set_point:
        prev = state.process_value
        target = state.set_point
        start = datetime.now()
        reached = False
    elif not reached and abs(state.error) < 0.1:
        reached = True
        duration = datetime.now() - start
        print("{}, {}, {}".format(
            prev, state.process_value, duration.total_seconds()))


if __name__ == '__main__':

    rospy.init_node('meassure_steering_adjustment', anonymous=True)

    topic = rospy.get_param(
        "topic", "/car_1/left_steering_hinge_position_controller/state")

    print("from, to, ms")

    rospy.Subscriber(
        topic, JointControllerState, update)

    rospy.spin()
