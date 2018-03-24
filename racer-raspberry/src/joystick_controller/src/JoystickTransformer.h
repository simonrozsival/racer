#ifndef JOYSTICK_TRASNFORMER_H_
#define JOYSTICK_TRASNFORMER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define DRIVE_BUTTON 0
#define THROTTLE_AXIS 1
#define STEERING_AXIS 0

class JoystickTransformer
{
  public:
    JoystickTransformer(const ros::Publisher &publisher);
    void process_joystick_input(const sensor_msgs::Joy::ConstPtr &msg);

  private:
    const ros::Publisher &steering_;

    void move(float throttle, float angle);
    void stop();
};

#endif