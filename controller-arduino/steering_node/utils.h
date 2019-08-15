#ifndef UTILS_H_
#define UTILS_H_

#define DUTY_CYCLE_MS 20

#define STEERING_LEFT_PWM 1200
#define STEERING_CENTER_PWM 1500
#define STEERING_RIGHT_PWM 1800

#define THROTTLE_REVERSE_PWM 1000
#define THROTTLE_MIN_REVERSE_PWM 1300
#define THROTTLE_NONE_PWM 1500
#define THROTTLE_MIN_FORWARD_PWM 1700
#define THROTTLE_FORWARD_PWM 2000

#define PWM_OFF_CENTER_TOLERANCE 100

double fmap(double value, double in_min, double in_max, double out_min, double out_max) {
  value = constrain(value, in_min, in_max);
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif
