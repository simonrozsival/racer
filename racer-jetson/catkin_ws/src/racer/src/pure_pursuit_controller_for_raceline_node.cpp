#include <iostream>
#include <optional>
#include <vector>

#include <ackermann_msgs/AckermannDrive.h>
#include <ros/ros.h>

#include "racer/following_strategies/pure_pursuit.h"
#include "racer/following_strategies/target_locator.h"
#include "racer/track/racing_line.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/vehicle_chassis.h"

#include "racer_ros/circuit_progress_monitoring.h"
#include "racer_ros/config/circuit.h"
#include "racer_ros/utils.h"

#include "racer_msgs/RacingLine.h"
#include "racer_msgs/State.h"

std::shared_ptr<racer::vehicle_model::kinematic::model> model;
std::shared_ptr<racer::vehicle_model::vehicle_chassis> vehicle;

std::optional<racer::vehicle_model::kinematic::state> last_known_state;
std::optional<racer::track::racing_line> racing_line;

void state_update(const racer_msgs::State::ConstPtr msg)
{
    racer::vehicle_configuration position = {msg->x, msg->y, msg->heading_angle};
    last_known_state = {position, msg->motor_rpm, msg->steering_angle};
}

void racing_line_update(const racer_msgs::RacingLine::ConstPtr msg)
{
    racing_line = racer_ros::msg_to_racing_line(msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pure_pursuit_controller_for_raceline_node");
    ros::NodeHandle node("~");

    std::string state_topic, racing_line_topic, ackermann_commands_topic,
        twist_commands_topic;
    node.param<std::string>("state_topic", state_topic, "/racer/state");
    node.param<std::string>("racing_line_topic", racing_line_topic,
                            "/racer/racing_line");
    node.param<std::string>("ackermann_commands_topic", ackermann_commands_topic,
                            "/racer/ackermann_commands");
    node.param<std::string>("tiwst_commands_topic", twist_commands_topic,
                            "/racer/commands");

    std::string nearest_point_topic, target_topic;
    node.param<std::string>("nearest_point_topic", nearest_point_topic,
                            "/car_1/purepursuit_control/visualize_nearest_point");
    node.param<std::string>("target_topic", target_topic,
                            "/car_1/purepursuit_control/ang_goal");

    auto state_sub =
        node.subscribe<racer_msgs::State>(state_topic, 1, &state_update);
    auto racing_line_sub = node.subscribe<racer_msgs::RacingLine>(
        racing_line_topic, 1, &racing_line_update);

    auto ackermann_pub = node.advertise<ackermann_msgs::AckermannDrive>(
        ackermann_commands_topic, 1);
    auto twist_pub =
        node.advertise<geometry_msgs::Twist>(twist_commands_topic, 1);

    auto nearest_point_pub =
        node.advertise<geometry_msgs::PoseStamped>(nearest_point_topic, 1);
    auto target_pub = node.advertise<geometry_msgs::PoseStamped>(target_topic, 1);

    vehicle = racer::vehicle_model::vehicle_chassis::simulator();
    model = std::make_shared<racer::vehicle_model::kinematic::model>(vehicle);

    racer::following_strategies::target_locator<
        racer::vehicle_model::kinematic::state>
        target_locator{5 * vehicle->wheelbase,  // min lookahead
                       15 * vehicle->wheelbase, // max lookahead
                       vehicle->motor->max_rpm()};
    racer::following_strategies::pure_pursuit<
        racer::vehicle_model::kinematic::state>
        strategy(vehicle->wheelbase);

    double frequency; // Hz
    node.param<double>("update_frequency_hz", frequency, 30.0);

    ros::Rate publish_frequency{frequency};

    ROS_INFO("==> PURE PURSUIT CONTROLLER FOR RACELINE NODE is ready to go");
    while (ros::ok())
    {
        if (racing_line && last_known_state)
        {
            // select angle and speed
            const auto target =
                target_locator.find_target(*last_known_state, *racing_line);
            const auto angle =
                strategy.select_steering_angle(*last_known_state, target);
            const auto closest_pt_index = racing_line->closest_point_along_the_spline(
                last_known_state->position());
            const auto speed = racing_line->spline()[closest_pt_index].maximum_speed;

            const auto action =
                model->action_from_speed_and_steering_angle(speed, angle);

            ackermann_pub.publish(racer_ros::action_to_ackermann_msg(action));
            twist_pub.publish(racer_ros::action_to_twist_msg(action));

            // Visualization
            nearest_point_pub.publish(racer_ros::pose_msg_from_point(
                racing_line->spline()[closest_pt_index].coordinate));
            target_pub.publish(racer_ros::pose_msg_from_point(target.location()));
        }

        ros::spinOnce();
        publish_frequency.sleep();
    }

    return 0;
}
