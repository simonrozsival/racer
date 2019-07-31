#!/usr/bin/env python

import rospy
import tf
import numpy as np

from tf import transformations as t
from nav_msgs.msg import Odometry

def pose_update(msg, args):
  listener, map_frame, odom_frame, base_link_frame = args
  br = tf.TransformBroadcaster()

  # odom -> base_link transformation
  trans, rot = listener.lookupTransform(odom_frame, base_link_frame, rospy.Time(0))
  odom_to_base = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))

  # map -> base_link transformation (from the msg)
  p = msg.pose.pose.position
  o = msg.pose.pose.orientation
  map_to_base = t.concatenate_matrices(t.translation_matrix((p.x, p.y, p.z)), t.quaternion_matrix((o.x, o.y, o.z, o.w)))

  # map -> odom = (map -> base_link) * (base_link -> odom) = (map -> base_link) * (odom -> base_link)^-1
  base_to_odom = t.inverse_matrix(odom_to_base)
  map_to_odom = np.matmul(map_to_base, base_to_odom)
  map_to_odom_trans = t.translation_from_matrix(map_to_odom)
  map_to_odom_rot = t.quaternion_from_matrix(map_to_odom)

  br.sendTransform(map_to_odom_trans, map_to_odom_rot, rospy.Time.now(), odom_frame, map_frame)

if __name__ == '__main__':
  rospy.init_node('fix_tf')
  
  pose_topic = rospy.get_param('~pose_topic', '/odom')
  map_frame = rospy.get_param('~map_frame', '/map')
  odom_frame = rospy.get_param('~odom_frame', '/odom')
  base_link_frame = rospy.get_param('~base_link_frame', '/base_link')
  
  listener = tf.TransformListener()
  rospy.Subscriber(pose_topic, Odometry, pose_update, (listener, map_frame, odom_frame, base_link_frame))
  rospy.spin()
