#!/usr/bin/env python

import rospy
import tf.transformations as t

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

last_pose = None
last_time = None

def pose_update(msg, args):
  global last_pose
  global last_time
  (odom_pub, child_frame) = args

  odom = Odometry()
  odom.header = msg.header
  odom.child_frame_id = child_frame
  odom.pose = msg.pose

  if last_pose is not None:
    dt = (rospy.Time.now() - last_time).to_sec()
    odom.twist.twist.linear.x = (last_pose.position.x - msg.pose.pose.position.x) / dt
    odom.twist.twist.linear.y = (last_pose.position.y - msg.pose.pose.position.y) / dt

    o = msg.pose.pose.orientation
    quat = (o.x, o.y, o.z, o.w)
    prev_yaw = t.euler_from_quaternion(quat)[2]

    o = last_pose.orientation
    quat = (o.x, o.y, o.z, o.w)
    yaw = t.euler_from_quaternion(quat)[2]

    odom.twist.twist.angular.z = (yaw - prev_yaw) / dt

    odom_pub.publish(odom)

  last_time = rospy.Time.now()
  last_pose = msg.pose.pose

if __name__ == '__main__':
  rospy.init_node('pose_estimate_to_odom')

  pose_topic = rospy.get_param('~pose_topic', '/robot_pose_ekf/odom_combined')
  odom_topic = rospy.get_param('~odom_topic', '/racer/odometry_combined')
  child_frame = rospy.get_param('~child_frame', 'base_link')

  odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=1)

  rospy.Subscriber(pose_topic, PoseWithCovarianceStamped, pose_update, (odom_pub, child_frame))
  rospy.spin()
