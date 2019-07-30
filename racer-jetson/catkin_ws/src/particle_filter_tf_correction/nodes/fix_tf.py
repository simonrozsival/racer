#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

def pose_update(msg, args):
  listener, map_frame, odom_frame, base_link_frame = args
  br = tf.TransformBroadcaster()

  base_link_position, base_link_rotation_q = listener.lookupTransform(odom_frame, base_link_frame, rospy.Time(0))
  base_link_euler = tf.transformations.euler_from_quaternion(base_link_rotation_q)

  o = msg.pose.pose.orientation
  quat = (o.x, o.y, o.z, o.w)
  absolute_euler = tf.transformations.euler_from_quaternion(quat)

  dx = msg.pose.pose.position.x - base_link_position[0]
  dy = msg.pose.pose.position.y - base_link_position[1]
  dtheta = base_link_euler[2] - base_link_euler[2]

  br.sendTransform((dx, dy, 0), tf.transformations.quaternion_from_euler(0, 0, dtheta), rospy.Time.now(), odom_frame, map_frame)

if __name__ == '__main__':
  rospy.init_node('fix_tf')
  
  pose_topic = rospy.get_param('~pose_topic', '/odom')
  map_frame = rospy.get_param('~map_frame', '/map')
  odom_frame = rospy.get_param('~odom_frame', '/odom')
  base_link_frame = rospy.get_param('~base_link_frame', '/base_link')
  
  listener = tf.TransformListener()
  rospy.Subscriber(pose_topic, Odometry, pose_update, (listener, map_frame, odom_frame, base_link_frame))
  rospy.spin()
