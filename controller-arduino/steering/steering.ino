/*
 * Raceing car steering
 * - receives commands from the '/racer/driving/commands' topic from ROS on a Raspberry PI through serial port
 * - translates throttle and steering commands into servo control
 * - publishes actions in the '/racer/driving/status' channel
 *
 * Please make sure you have installed the `ros_lib` library for running ROS node on an Arduino with the
 * `ros-kinetic-angles` package installed (so the `geometry_msgs/Twist.h` header file is available).
 */

#include <stdio.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <Servo.h>

// #define BAUD_RATE 115200
#define BAUD_RATE 9600

#define STEERING_PIN 9
#define ESC_PIN 10

#define STEERING_RIGHT 60
#define STEERING_CENTER 90
#define STEERING_LEFT 120

#define SPEED_MIN_REVERSE 85
#define SPEED_MAX_REVERSE 16

#define SPEED_FULL_STOP 94

#define SPEED_MIN_FORWARD 95
#define SPEED_FULL_FORWARD 110

const char driving_topic[] = "/racer/driving/commands";
const char driving_status_topic[] = "/racer/driving/status";

double fmap(double value, double in_min, double in_max, double out_min, double out_max);
void drive_callback(const geometry_msgs::Twist& twist_msg);

ros::NodeHandle node_handle;

std_msgs::String status_msg;

ros::Publisher driving_status(driving_status_topic, &status_msg);
ros::Subscriber<geometry_msgs::Twist> driving(driving_topic, &drive_callback);

Servo steering_servo;
Servo esc_servo; // the ESC works like a Servo

void setup() {
  Serial.begin(BAUD_RATE);

  node_handle.initNode();
  node_handle.advertise(driving_status);
  node_handle.subscribe(driving);  

  steering_servo.attach(STEERING_PIN);
  esc_servo.attach(ESC_PIN);

  steering_servo.write(STEERING_CENTER) ;
  esc_servo.write(SPEED_FULL_STOP);
}

int speed = SPEED_FULL_STOP;

void loop() {
  node_handle.spinOnce();
  delay(1);
}

void drive_callback(const geometry_msgs::Twist& twist_msg) {
  int steering_angle = fmap(twist_msg.angular.z, 0.0, 1.0, STEERING_LEFT, STEERING_RIGHT);
  int throttle = fmap(twist_msg.linear.x, 0.0, 1.0, SPEED_FULL_STOP, SPEED_FULL_FORWARD);

  steering_servo.write(steering_angle);
  esc_servo.write(throttle);
}

double fmap(double value, double in_min, double in_max, double out_min, double out_max) {
  value = constrain(value, in_min, in_max);
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

