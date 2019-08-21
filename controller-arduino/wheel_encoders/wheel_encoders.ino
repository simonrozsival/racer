#include <ros.h>
#include <std_msgs/Float64.h>

// The pin which we use to connect the encoder to must be
// able to handle an interrupt. Check the pin layout of your Arduino.
#define ENCODER_PIN 2

#define REVOLUTIONS_TOPIC "/racer/wheel_encoders"

ros::NodeHandle  nh;
std_msgs::Float64 msg;
ros::Publisher revolutions_topic(REVOLUTIONS_TOPIC, &msg);

long steps_accumulator = 0;
const double poles_per_revolution = 8.0;

void encoder_callback() {
  ++steps_accumulator;
}

void setup() {
  nh.initNode();
  nh.advertise(revolutions_topic);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoder_callback, CHANGE);
} 

void loop() {    
  msg.data = revolutions(steps_accumulator);
  revolutions_topic.publish(&msg);

  nh.spinOnce();
  
  delay(10); // ms
}

inline double revolutions(int steps) {
  return steps / poles_per_revolution;
}
