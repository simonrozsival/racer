#ifndef BASE_VEHICLE_MODEL_H_
#define BASE_VEHICLE_MODEL_H_

#include <iostream>

#include "../math/base_integrator.h"
#include "../math/primitives.h"

namespace racing {

    struct vehicle_position {
        const double x, y, heading_angle;

        vehicle_position(double x, double y, double heading_angle)
            : x(x), y(y), heading_angle(heading_angle)
        {
        }

        bool operator==(const vehicle_position& other) const {
            return x == other.x && y == other.y && heading_angle == other.heading_angle;
        }

        vehicle_position operator+(const vehicle_position& other) const {
            return vehicle_position(x + other.x, y + other.y, heading_angle + other.heading_angle);
        }

        math::vector location() const {
            return math::vector(x, y);
        }

        vehicle_position integrate(const std::unique_ptr<math::base_integrator>& integrator) const {
            return vehicle_position(
                integrator->integrate(x),
                integrator->integrate(y),
                integrator->integrate(heading_angle)
            );
        }
    };

    struct vehicle_properties {
        const double distance_of_center_of_gravity_to_rear_axle, wheelbase, max_speed, max_acceleration;
        const double width, length;
        const double max_steering_speed, max_steering_angle;

        vehicle_properties(
            double cog_offset,
            double wheelbase,
            double width,
            double length,
            double max_steering_speed,
            double max_steering_angle,
            double max_speed,
            double max_acceleration)
            : distance_of_center_of_gravity_to_rear_axle(cog_offset),
            wheelbase(wheelbase),
            width(width),
            length(length),
            max_steering_speed(max_steering_speed),
            max_steering_angle(max_steering_angle),
            max_speed(max_speed),
            max_acceleration(max_acceleration)
        {
        }

        double radius() const {
            double dx = length / 2;
            double dy = width / 2;
            return sqrt(dx * dx + dy * dy);
        }
    };

    template <typename TState, typename TAction>
    class base_vehicle_model {
    public:
        virtual std::unique_ptr<TState> predict(const TState& state, const TAction& action) const = 0;
    };

}

#endif