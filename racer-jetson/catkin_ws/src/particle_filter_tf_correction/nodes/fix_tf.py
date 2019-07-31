#!/usr/bin/env python

import rospy
import tf
import numpy as np

from tf import transformations as t
from nav_msgs.msg import Odometry

def pose_update(msg, args):
  listener, map_frame, odom_frame, laser_frame = args
  br = tf.TransformBroadcaster()

  # odom -> laser transformation
  trans, rot = listener.lookupTransform(odom_frame, laser_frame, rospy.Time(0))
  odom_to_laser = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))

  # map -> laser transformation (from the msg)
  p = msg.pose.pose.position
  o = msg.pose.pose.orientation
  map_to_laser = t.concatenate_matrices(t.translation_matrix((p.x, p.y, p.z)), t.quaternion_matrix((o.x, o.y, o.z, o.w)))

  # map -> odom = (map -> laser) * (laser -> odom) = (map -> laser) * (odom -> laser)^-1
  laser_to_odom = t.inverse_matrix(odom_to_laser)
  map_to_odom = np.matmul(map_to_laser, laser_to_odom)
  map_to_odom_trans = t.translation_from_matrix(map_to_odom)
  map_to_odom_rot = t.quaternion_from_matrix(map_to_odom)

  br.sendTransform(map_to_odom_trans, map_to_odom_rot, rospy.Time.now(), odom_frame, map_frame)

if __name__ == '__main__':
  rospy.init_node('fix_tf')
  
  pose_topic = rospy.get_param('~pose_topic', '/odom')
  map_frame = rospy.get_param('~map_frame', '/map')
  odom_frame = rospy.get_param('~odom_frame', '/odom')
  laser_frame = rospy.get_param('~laser_frame', '/laser')
  
  listener = tf.TransformListener()
  rospy.Subscriber(pose_topic, Odometry, pose_update, (listener, map_frame, odom_frame, laser_frame))
  rospy.spin()
