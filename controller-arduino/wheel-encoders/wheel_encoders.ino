#include <ros.h>
#include <std_msgs/Float64.h>

#define LEFT_REAR_WHEEL_ENCODER_PIN 2
#define CHANGES_PER_REVOLUTION 8

#define DISTANCE_TOPIC "/racer/wheel_encoders"

ros::NodeHandle  nh;
std_msgs::Float64 msg;
ros::Publisher distance_topic(DISTANCE_TOPIC, &msg);

long steps_accumulator = 0;

void encoder_callback() {
  steps_accumulator++;
}

void setup() {
  nh.initNode();
  nh.advertise(distance_topic);

  pinMode(LEFT_REAR_WHEEL_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_REAR_WHEEL_ENCODER_PIN), encoder_callback, CHANGE);
} 

void loop() {    
  msg.data = revolutions(steps_accumulator);
  distance_topic.publish(&msg);

  nh.spinOnce();
}

double revolutions(int steps) {
  return steps / (double)CHANGES_PER_REVOLUTION;
}

