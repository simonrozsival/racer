#!/usr/bin/env python

"""
This script listens for racing line messages and transforms them into a rviz visualizations.
"""

import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from racer_msgs.msg import RacingLine
from visualization_msgs.msg import Marker, MarkerArray

def create_corner_point_marker(corner, id):
    marker = Marker()

    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    marker.ns = "racing-line"
    marker.id = id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = corner.point.x
    marker.pose.position.y = corner.point.y
    marker.pose.position.z = 0

    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.1

    relative_speed = corner.maximum_speed.data / 767.5

    marker.color.r = 1 - relative_speed
    marker.color.g = relative_speed
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker

def create_line_strip(points, id):
    marker = Marker()

    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()

    marker.ns = "racing-line"
    marker.id = id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    for pt in points:
        point = Point()
        point.x = pt.x
        point.y = pt.y
        point.z = 0
        marker.points.append(point)

    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    return marker

def create_marker_array(msg):
    control_points = MarkerArray()

    i = 0
    for corner in msg.corners:
        control_points.markers.append(create_corner_point_marker(corner.turn_in, i))
        control_points.markers.append(create_corner_point_marker(corner.apex, i + 1))
        control_points.markers.append(create_corner_point_marker(corner.exit, i + 2))
        i = i + 3

    control_points.markers.append(create_line_strip(msg.points, i))

    return control_points

def racing_line(racing_line_topic, visualization_topic):
    pub = rospy.Publisher(visualization_topic, MarkerArray, queue_size=1)
    def update(msg):
        markers = create_marker_array(msg)
        pub.publish(markers)

    rospy.Subscriber("/racer/racing_line", RacingLine, update)

    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('racing_line_visualization', anonymous=True) 
        racing_line(
            rospy.get_param('~racing_line_topic', '/racer/racing_line'),
            rospy.get_param('~visualization_topic', '/racer/visualization/racing_line')
        )
    except rospy.ROSInterruptException:
        pass
