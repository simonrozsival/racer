#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>

ros::Publisher pub;
sensor_msgs::Imu corrected_msg;
double covariance;
std::string imu_frame_id;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  corrected_msg.header.stamp = msg->header.stamp;
  corrected_msg.header.frame_id = imu_frame_id;

  // remap axes:
  // -y -> x' <==> pitch' = -roll
  //  x -> y' <==> roll' = pitch

  // @todo: find a better way to do this

  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);
  tfScalar roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  corrected_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(-pitch, roll, yaw);

  corrected_msg.linear_acceleration.x = -msg->linear_acceleration.y;
  corrected_msg.linear_acceleration.y = msg->linear_acceleration.x;
  corrected_msg.linear_acceleration.z = msg->linear_acceleration.z;

  corrected_msg.angular_velocity.x = -msg->angular_velocity.y;
  corrected_msg.angular_velocity.y = msg->angular_velocity.x;
  corrected_msg.angular_velocity.z = msg->angular_velocity.z;

  // add covariance values
  corrected_msg.orientation_covariance = { covariance, 0, 0, 0, covariance, 0, 0, 0, covariance };
  corrected_msg.angular_velocity_covariance = { covariance, 0, 0, 0, covariance, 0, 0, 0, covariance };
  corrected_msg.linear_acceleration_covariance = { covariance, 0, 0, 0, covariance, 0, 0, 0, covariance };

  pub.publish(corrected_msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "fix_imu");
  ros::NodeHandle nh("~");

  std::string imu_input_topic, imu_output_topic;
  nh.param<std::string>("imu_frame_id", imu_frame_id, "imu");
  nh.param<std::string>("imu_input_topic", imu_input_topic, "/imu_raw");
  nh.param<std::string>("imu_output_topic", imu_output_topic, "/imu_data");
  nh.param<double>("covariance", covariance, 0.001);

  const auto sub = nh.subscribe<sensor_msgs::Imu>(imu_input_topic, 1, &imu_callback);
  pub = nh.advertise<sensor_msgs::Imu>(imu_output_topic, 1, false);

  ros::spin();
  return 0;
}