#include "JoystickTransformer.h"

JoystickTransformer::JoystickTransformer(const ros::Publisher &publisher)
    : steering_(publisher)
{
}

void JoystickTransformer::process_joystick_input(const sensor_msgs::Joy::ConstPtr &joy)
{
    bool stop = !joy->buttons[DRIVE_BUTTON];
    if (stop)
    {
        this->stop();
        return;
    }

    float throttle = joy->axes[THROTTLE_AXIS];
    float angle = joy->axes[STEERING_AXIS];

    move(throttle, angle);
}

void JoystickTransformer::stop()
{
    move(0, 0);
}

void JoystickTransformer::move(float throttle, float angle)
{
    throttle = throttle > 0 ? throttle : 0;

    geometry_msgs::Twist msg;
    msg.linear.x = throttle;
    msg.angular.z = angle;

    steering_.publish(msg);
}
