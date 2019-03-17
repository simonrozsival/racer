/*
   Raceing car steering
   - receives commands from the '/racer/driving/commands' topic from ROS on a Raspberry PI through serial port
   - translates throttle and steering commands into servo control
   - publishes actions in the '/racer/driving/status' channel
   - if the user takes over control using the RC controler, the PWM signals are forwarded to the servos
     and the corresponding messages are published in the `/racer/driving/commands` topic

   Please make sure you have installed the `ros_lib` library for running ROS node on an Arduino with the
   `ros-kinetic-angles` package installed (so the `geometry_msgs/Twist.h` header file is available).
*/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>
#include "PWM.hpp"

#define STEERING_LEFT_PWM 1000
#define STEERING_CENTER_PWM 1500
#define STEERING_RIGHT_PWM 2000

#define THROTTLE_REVERSE_PWM 1000
#define THROTTLE_NONE_PWM 1500
#define THROTTLE_FULL_PWM 2000

#define PWM_OFF_CENTER_TOLERANCE 100

#define PIN_RC_INPUT_THROTTLE 3
#define PIN_RC_INPUT_STEERING 2

#define PIN_OUTPUT_THROTTLE 10
#define PIN_OUTPUT_STEERING 9

#define DRIVING_TOPIC "/racer/commands"

double fmap(double value, double in_min, double in_max, double out_min, double out_max);
void drive_callback(const geometry_msgs::Twist& twist_msg);

ros::NodeHandle node_handle;
geometry_msgs::Twist manual_driving_msg;

ros::Publisher manual_driving(DRIVING_TOPIC, &manual_driving_msg);
ros::Subscriber<geometry_msgs::Twist> driving(DRIVING_TOPIC, &drive_callback);

PWM rc_input_steering(PIN_RC_INPUT_STEERING);
PWM rc_input_throttle(PIN_RC_INPUT_THROTTLE);

Servo servo_steering;
Servo servo_throttle;

bool autonomous_mode;

int steering_angle = STEERING_CENTER_PWM;
int throttle = THROTTLE_NONE_PWM;

int read_rc_input(const PWM* rc, int center_pwm)
{
  int value = rc->getValue();
  if (abs(value - center_pwm) < PWM_OFF_CENTER_TOLERANCE)
  {
    value = center_pwm;
  }

  return value;
}

void setup() {
  node_handle.initNode();
  node_handle.advertise(manual_driving);
  node_handle.subscribe(driving);

  rc_input_steering.begin(true);
  rc_input_throttle.begin(true);

  servo_steering.attach(PIN_OUTPUT_STEERING);
  servo_throttle.attach(PIN_OUTPUT_THROTTLE);

  autonomous_mode = true;
}

void loop() {
  node_handle.spinOnce();

  int steering_pwm = read_rc_input(&rc_input_steering, STEERING_CENTER_PWM);
  int throttle_pwm = read_rc_input(&rc_input_throttle, THROTTLE_NONE_PWM);

  if (autonomous_mode && (steering_pwm != STEERING_CENTER_PWM || throttle_pwm != THROTTLE_NONE_PWM))
  {
    autonomous_mode = false;
  }

  if (autonomous_mode)
  {
    steering_pwm = steering_angle;
    throttle_pwm = throttle;
  }
  else
  {
    // translate the RC controller inputs into steering commands
    manual_driving_msg.linear.x = fmap(throttle_pwm, THROTTLE_REVERSE_PWM, THROTTLE_FULL_PWM, -1.0, 1.0);
    manual_driving_msg.angular.z = -1 * fmap(steering_pwm, STEERING_LEFT_PWM, STEERING_RIGHT_PWM, -1.0, 1.0);
    manual_driving.publish(&manual_driving_msg);
  }

  servo_steering.writeMicroseconds(steering_pwm);
  servo_throttle.writeMicroseconds(throttle_pwm);

  delay(20); // 20ms - 1 PWM duty cycle
}

void drive_callback(const geometry_msgs::Twist& twist_msg) {
   steering_angle = fmap(twist_msg.angular.z, -1.0, 1.0, STEERING_RIGHT_PWM, STEERING_LEFT_PWM);
   throttle = fmap(twist_msg.linear.x, 0.0, 1.0, THROTTLE_NONE_PWM, THROTTLE_FULL_PWM);
}

double fmap(double value, double in_min, double in_max, double out_min, double out_max) {
  value = constrain(value, in_min, in_max);
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
