#ifndef DRIVING_LOGIC_H_
#define DRIVING_LOGIC_H_

#include <ros.h>
#include <geometry_msgs/Twist.h>

#include "./utils.h"
#include "./rc_input.h"

bool reverse;
bool autonomous_mode;
bool has_autonomous_steering_data;
int autonomous_steering_angle;

void init_driving_logic() {
  autonomous_mode = true;
  has_autonomous_steering_data = false;
  reverse = false;
  autonomous_steering_angle = STEERING_CENTER_PWM;
}

void drive_callback(const geometry_msgs::Twist& twist_msg) {
  has_autonomous_steering_data = true;
  autonomous_steering_angle = fmap(twist_msg.angular.z, -1.0, 1.0, STEERING_RIGHT_PWM, STEERING_LEFT_PWM);
}

void select_inputs(int* steering_pwm, int* throttle_pwm) {
  *steering_pwm = rc_input[STEERING];
  *throttle_pwm = rc_input[THROTTLE];

  if (autonomous_mode) {
    if (abs(*steering_pwm - STEERING_CENTER_PWM) > PWM_OFF_CENTER_TOLERANCE) {
      autonomous_mode = false;
    } else if (has_autonomous_steering_data) {
      *steering_pwm = autonomous_steering_angle;
    }
  }
}

#endif
