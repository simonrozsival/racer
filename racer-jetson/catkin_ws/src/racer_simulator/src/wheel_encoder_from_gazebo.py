#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetLinkState


def wheel_angular_vel(link, base):
    # the angular velocity is calculate against the car's base link frame in the Y axis
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state = rospy.ServiceProxy(
            '/gazebo/get_link_state', GetLinkState)
        res = get_link_state(link, base)
        if not res.success:
            return 0
        else:
            return res.link_state.twist.angular.y
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))


def calculate_motor_rpm(angular_vel):
    wheel_rpm = angular_vel * 60 / (2 * math.pi)
    gear_ratio = 18
    return wheel_rpm * gear_ratio


def wheel_encoder(
    left_wheel_link,
    right_wheel_link,
    base_link,
    pub_topic
):
    pub = rospy.Publisher(pub_topic, Float64, queue_size=1)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        # we'll takte the maximum of the two rear wheels
        # to account for a differential (if any)
        angular_vel = max(
            wheel_angular_vel(left_wheel_link, base_link),
            wheel_angular_vel(right_wheel_link, base_link))
        motor_rpm = calculate_motor_rpm(angular_vel)
        pub.publish(motor_rpm)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('wheel_encoder_from_gazebo')
        wheel_encoder(
            rospy.get_param('~left_wheel_frame_id'),
            rospy.get_param('~right_wheel_frame_id'),
            rospy.get_param('~base_link_frame_id'),
            rospy.get_param('~topic')
        )
    except rospy.ROSInterruptException:
        pass
