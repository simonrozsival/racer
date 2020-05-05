#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from racer_msgs.msg import RacingLine
from std_msgs.msg import String

global line
line = None
        
def racing_line(msg):
    global line
    line = msg

def lookahead_from_speed(odom):
    vx = odom.twist.twist.linear.x
    vy = odom.twist.twist.linear.y
    speed = math.sqrt(vx ** 2 + vy ** 2)

    min_lookahead = 2
    max_lookahead = 5 # meters
    max_speed = 3 # meters per second

    lookahead = max_lookahead * (speed / max_speed)

    return max(min_lookahead, min(max_lookahead, lookahead))

def find_nearest_point(line, odom):
    index = 0
    min_distance = None

    for i in range(len(line.points)):
        dx = line.points[i].x - odom.pose.pose.position.x
        dy = line.points[i].y - odom.pose.pose.position.y
        distance_sq = dx**2 + dy**2

        if min_distance is None or distance_sq <= min_distance:
            min_distance = distance_sq
            index = i

    return index

def find_goal(line, odom, start_index):
    lookahead = lookahead_from_speed(odom)

    def distance_sq(a, b):
        dx = line.points[a].x - line.points[b].x
        dy = line.points[a].y - line.points[b].y
        return dx ** 2 + dy ** 2

    for i in range(len(line.points)):
        index = (start_index + i) % len(line.points)
        if distance_sq(start_index, index) > lookahead ** 2:
            print("distance: {}, lookahead: {}".format(math.sqrt(distance_sq(start_index, index)), lookahead))
            return line.points[index]

def pose_from(pt, seq):
    pose = PoseStamped()
    pose.header.seq         = seq
    pose.header.stamp       = rospy.Time.now()
    pose.header.frame_id    = 'map'
    pose.pose.position.x    = pt.x
    pose.pose.position.y    = pt.y
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose

def pure_pursuit_goal_finder(
    odom_topic,
    racing_line_topic,
    angle_goal_topic,
    velocity_goal_topic,
    visualize_nearest_point_topic,
    adaptive_lookahead_topic
):
    ang_goal_pub = rospy.Publisher(angle_goal_topic, PoseStamped, queue_size = 1)
    vel_goal_pub = rospy.Publisher(velocity_goal_topic, PoseStamped, queue_size = 1)
    nearest_point_pub = rospy.Publisher(visualize_nearest_point_topic, PoseStamped, queue_size = 1)
    adaptive_lookahead_pub = rospy.Publisher(adaptive_lookahead_topic, String, queue_size = 1)

    global seq
    seq = 0

    def odom(msg):
        global line
        global seq

        if line is not None:
            nearest_point_index = find_nearest_point(line, msg)
            goal = find_goal(line, msg, nearest_point_index)
            
            goal_pose = pose_from(goal, seq)
            ang_goal_pub.publish(goal_pose)
            vel_goal_pub.publish(goal_pose)
            seq = seq + 1

            nearest_pose = pose_from(line.points[nearest_point_index], seq)
            nearest_point_pub.publish(nearest_pose)
            seq = seq + 1

            adaptive_lookahead_pub.publish('unrestricted') # caution, brake, unrestricted


    rospy.Subscriber(odom_topic, Odometry, odom, queue_size=1)
    rospy.Subscriber(racing_line_topic, RacingLine, racing_line)

    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('pure_pursuit_goal_finder')    
        pure_pursuit_goal_finder(
            rospy.get_param('~odom_topic', '/car_1/base/odom'),
            rospy.get_param('~racing_line_topic', '/racer/racing_line'),
            rospy.get_param('~angle_goal_topic', '/car_1/purepursuit_control/ang_goal'),
            rospy.get_param('~velocity_goal_topic', '/car_1/purepursuit_control/vel_goal'),
            rospy.get_param('~visualize_nearest_point_topic', '/car_1/purepursuit_control/visualize_nearest_point'),
            rospy.get_param('~adaptive_lookahead_topic', '/car_1/purepursuit_control/adaptive_lookahead')
        )
    except rospy.ROSInterruptException:
        pass
