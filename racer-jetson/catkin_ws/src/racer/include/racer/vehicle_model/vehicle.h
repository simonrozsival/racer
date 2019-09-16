#ifndef VEHICLE_H_
#define VEHICLE_H_

namespace racer::vehicle_model
{

struct vehicle
{
    const double distance_of_center_of_gravity_to_rear_axle, wheelbase, max_speed, max_reversing_speed, max_acceleration;
    const double width, length;
    const double max_steering_speed, max_steering_angle;

    vehicle(
        double cog_offset,
        double wheelbase,
        double width,
        double length,
        double max_steering_speed,
        double max_steering_angle,
        double max_speed,
        double max_reversing_speed,
        double max_acceleration)
        : distance_of_center_of_gravity_to_rear_axle(cog_offset),
          wheelbase(wheelbase),
          max_speed(max_speed),
          max_reversing_speed(max_reversing_speed),
          max_acceleration(max_acceleration),
          width(width),
          length(length),
          max_steering_speed(max_steering_speed),
          max_steering_angle(max_steering_angle)
    {
    }

    double radius() const
    {
        double dx = length / 2;
        double dy = width / 2;
        return sqrt(dx * dx + dy * dy);
    }
};

} // namespace racer::vehicle_model

#endif
