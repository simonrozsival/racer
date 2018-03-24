#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define DRIVING_TOPIC "/racer/driving/commands"

#define DRIVE_BUTTON 0
#define THROTTLE_AXIS 1
#define STEERING_AXIS 0

class JoystickTransformer
{
  public:
    JoystickTransformer(const ros::Publisher& publisher);
    void process_joystick_input(const sensor_msgs::Joy::ConstPtr& msg);

  private:
    const ros::Publisher& steering_;

    void move(float throttle, float angle);
    void stop();
}

JoystickTransformer::JoystickTransformer(const ros::Publisher& publisher)
  : steering_(publisher)
{
}

void JoystickTransformer::process_joystick_input(const sensor_msgs::Joy::ConstPtr& msg)
{
  bool stop = !msg[DRIVE_BUTTON];
  if (stop)
  {
    stop();
    return;
  }

  float throttle = joy->axes[TROTTLE_AXIS];
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_controller");
  ros::NodeHandle nh;

  ros::Publisher steering_pub = nh.advertise<geometry_msgs::Twist>(DRIVING_TOPIC, 1, true);
  JoystickTransformer transformer(steering_pub);

  ros::Subscriber joystick_input_sub = nh.subscribe<sensor_msgs::Joy>(JOY_TOPIC, 1, &JoystickTransformer::process_joystick_input, &transformer);

  ros::spin();
}
