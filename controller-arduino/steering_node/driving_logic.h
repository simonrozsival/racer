#ifndef DRIVING_LOGIC_H_
#define DRIVING_LOGIC_H_

#include <ros.h>
#include <geometry_msgs/Twist.h>

#include "./utils.h"
#include "./rc_input.h"

bool autonomous_mode, reverse;
bool has_autonomous_steering_data;
int autonomous_steering_angle, autonomous_throttle;

void init_driving_logic() {
  has_autonomous_steering_data = false;
  autonomous_mode = true;
  reverse = false;
  autonomous_steering_angle = STEERING_CENTER_PWM;
  autonomous_throttle = THROTTLE_NONE_PWM;
}

void drive_callback(const geometry_msgs::Twist& twist_msg) {
  has_autonomous_steering_data = true;
  autonomous_steering_angle = fmap(twist_msg.angular.z, -1.0, 1.0, STEERING_RIGHT_PWM, STEERING_LEFT_PWM);
  autonomous_throttle = fmap(twist_msg.linear.x, -1.0, 1.0, THROTTLE_REVERSE_PWM, THROTTLE_FULL_PWM);
}

bool is_offcenter(const int value, int center_pwm) {
  return abs(value - center_pwm) > (int)PWM_OFF_CENTER_TOLERANCE;
}

void select_inputs(int* steering_pwm, int* throttle_pwm) {
  *steering_pwm = rc_input[STEERING];
  *throttle_pwm = rc_input[THROTTLE];

  if (autonomous_mode) {
    if (is_offcenter(*steering_pwm, (int)STEERING_CENTER_PWM) || is_offcenter(*throttle_pwm, (int)THROTTLE_NONE_PWM)) {
      // user took control over the vehicle
      autonomous_mode = false;
    } else if (has_autonomous_steering_data) {
      *steering_pwm = autonomous_steering_angle;
      *throttle_pwm = autonomous_throttle;
    }
  }
}

#endif
