/*
   Raceing car steering
   - receives commands from the '/racer/commands' topic from ROS through serial port
   - translates throttle and steering commands into servo control
   - if the user takes over control using the RC controler, the PWM signals are forwarded to the servos
     and the corresponding messages are published in the `/racer/commands` topic and any further commands
     received through this topic are ignored

   Please make sure you have installed the `ros_lib` library for running ROS node on an Arduino with the
   `ros-kinetic-angles` package installed (so the `geometry_msgs/Twist.h` header file is available).
*/

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Servo.h>

#define DUTY_CYCLE_MS 20

#define STEERING_LEFT_PWM 1000
#define STEERING_CENTER_PWM 1500
#define STEERING_RIGHT_PWM 2000

#define THROTTLE_REVERSE_PWM 1000
#define THROTTLE_NONE_PWM 1500
#define THROTTLE_FULL_PWM 2000

#define PWM_OFF_CENTER_TOLERANCE 100

#include "driving_logic.h"
#include "utils.h"

#define PIN_OUTPUT_THROTTLE 10
#define PIN_OUTPUT_STEERING 9

#define DRIVING_TOPIC "/racer/commands"

ros::NodeHandle node_handle;
geometry_msgs::Twist manual_driving_msg;

ros::Publisher manual_driving(DRIVING_TOPIC, &manual_driving_msg);
ros::Subscriber<geometry_msgs::Twist> driving(DRIVING_TOPIC, &drive_callback);

Servo servo_steering;
Servo servo_throttle;

bool reverse;

void setup() {
  node_handle.initNode();
  node_handle.advertise(manual_driving);
  node_handle.subscribe(driving);

  attach_rc_input_interrupts();
  init_dricing_logic();

  servo_steering.attach(PIN_OUTPUT_STEERING);
  servo_throttle.attach(PIN_OUTPUT_THROTTLE);

  autonomous_mode = true;
}

void loop() {
  node_handle.spinOnce();

  int steering_pwm, throttle_pwm;
  select_inputs(&steering_pwm, &throttle_pwm);

  servo_steering.writeMicroseconds(steering_pwm);

  if (!reverse && throttle_pwm < THROTTLE_FULL_PWM) {
    // switching into reverse is a bit more complicated than just setting PWM between THROTTLE_REVERSE_PWM and THROTTLE_NONE_PWM
    // the PWM has to be set to THROTTLE_REVERSE_PWM for a while, then return back to THROTTLE_NONE_PWM and then go to the `autonomous_throttle` value
    // this will dealy the algorithm a bit and it will 
    servo_throttle.writeMicroseconds(THROTTLE_REVERSE_PWM);
    delay(2 * DUTY_CYCLE_MS);
    servo_throttle.writeMicroseconds(THROTTLE_NONE_PWM);
    delay(2 * DUTY_CYCLE_MS);
  }

  servo_throttle.writeMicroseconds(throttle_pwm);
  reverse = autonomous_throttle < THROTTLE_FULL_PWM;

  if (!autonomous_mode) {
    // translate the RC controller inputs into steering commands
    manual_driving_msg.linear.x = fmap(throttle_pwm, THROTTLE_REVERSE_PWM, THROTTLE_FULL_PWM, -1.0, 1.0);
    manual_driving_msg.angular.z = -1 * fmap(steering_pwm, STEERING_LEFT_PWM, STEERING_RIGHT_PWM, -1.0, 1.0);
    manual_driving.publish(&manual_driving_msg);
  }

  delay(DUTY_CYCLE_MS);
}
