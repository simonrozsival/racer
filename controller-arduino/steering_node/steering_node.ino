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
#include <std_msgs/Int32.h>
#include <Servo.h>

#include "./utils.h"
#include "./driving_logic.h"

#define PIN_OUTPUT_THROTTLE 10
#define PIN_OUTPUT_STEERING 9
#define PIN_LED 13

#define DRIVING_TOPIC "/racer/commands"

int blinking_counter = 0;

ros::NodeHandle node_handle;
geometry_msgs::Twist manual_driving_msg;
std_msgs::Int32 pwm_msg;

ros::Publisher manual_driving(DRIVING_TOPIC, &manual_driving_msg);
ros::Publisher pwm_pub("/racer/pwm", &pwm_msg);
ros::Subscriber<geometry_msgs::Twist> driving(DRIVING_TOPIC, &drive_callback);

Servo servo_steering;
Servo servo_throttle;

void setup() {
  node_handle.initNode();
  node_handle.advertise(manual_driving);
  node_handle.advertise(pwm_pub);
  node_handle.subscribe(driving);

  attach_rc_input_interrupts();
  init_driving_logic();

  servo_steering.attach(PIN_OUTPUT_STEERING);
  servo_throttle.attach(PIN_OUTPUT_THROTTLE);
  
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  node_handle.spinOnce();

  int steering_pwm, throttle_pwm;
  select_inputs(&steering_pwm, &throttle_pwm);

  servo_steering.writeMicroseconds(steering_pwm);

  if (!reverse && throttle_pwm < (int)THROTTLE_NONE_PWM - (int)PWM_OFF_CENTER_TOLERANCE) {
    // switching into reverse is a bit more complicated than just setting PWM between THROTTLE_REVERSE_PWM and THROTTLE_NONE_PWM
    // the PWM has to be set to THROTTLE_REVERSE_PWM for a while, then return back to THROTTLE_NONE_PWM and then go to the `autonomous_throttle` value
    // this will dealy the algorithm a bit and it will 
    servo_throttle.writeMicroseconds(THROTTLE_REVERSE_PWM);
    delay(1 * DUTY_CYCLE_MS);
    servo_throttle.writeMicroseconds(THROTTLE_NONE_PWM);
    delay(3 * DUTY_CYCLE_MS); // 3 was the minimum constant which worked for me
  }

  pwm_msg.data = throttle_pwm;
  pwm_pub.publish(&pwm_msg);
  
  servo_throttle.writeMicroseconds(throttle_pwm);
  reverse = throttle_pwm < (int)THROTTLE_NONE_PWM - (int)PWM_OFF_CENTER_TOLERANCE;

  if (!autonomous_mode) {
    // translate the RC controller inputs into steering commands
    manual_driving_msg.linear.x = fmap(throttle_pwm, THROTTLE_REVERSE_PWM, THROTTLE_FORWARD_PWM, -1.0, 1.0);
    manual_driving_msg.angular.z = -1 * fmap(steering_pwm, STEERING_LEFT_PWM, STEERING_RIGHT_PWM, -1.0, 1.0);
    manual_driving.publish(&manual_driving_msg);
  } else if (++blinking_counter % 10 == 0) {
    digitalWrite(PIN_LED, blinking_counter % 20 == 0 ? HIGH : LOW);
  }

  delay(DUTY_CYCLE_MS);
}
