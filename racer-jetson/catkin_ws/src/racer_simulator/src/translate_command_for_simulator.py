#!/usr/bin/env python

import rospy
import math

from ackermann_msgs.msg import AckermannDrive

def command_translator(
    input_command_topic,
    output_command_topic,
    max_steering_angle,
    max_speed
):
    pub = rospy.Publisher(output_command_topic, AckermannDrive, queue_size=1)

    def translate(msg):
        out = AckermannDrive()
        out.steering_angle = msg.steering_angle / max_steering_angle
        out.speed = msg.speed / max_speed
        pub.publish(out)

    rospy.Subscriber(input_command_topic, AckermannDrive, translate, queue_size=1)

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('command_translator')    
        command_translator(
            rospy.get_param('~input_command_topic', '/racer/ackermann_commands'),
            rospy.get_param('~output_command_topic', '/car_1/command'),
            rospy.get_param('~max_steering_angle', math.radians(26.57)),
            rospy.get_param('~max_speed', 10)
        )
    except rospy.ROSInterruptException:
        pass
