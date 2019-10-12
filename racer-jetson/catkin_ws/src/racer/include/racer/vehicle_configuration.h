#ifndef vehicle_configuration_H_
#define vehicle_configuration_H_

#include "racer/math.h"

namespace racer
{

struct vehicle_configuration
{
private:
    racer::math::point location_;
    double heading_angle_;
    bool is_valid_;

public:
    vehicle_configuration(double x, double y, double heading_angle)
        : location_{x, y}, heading_angle_{heading_angle}, is_valid_{true}
    {
    }

    vehicle_configuration(racer::math::point location, double heading_angle)
        : location_{location}, heading_angle_{heading_angle}, is_valid_{true}
    {
    }

    vehicle_configuration()
        : location_{0, 0}, heading_angle_{0}, is_valid_{false}
    {
    }

    vehicle_configuration(const vehicle_configuration &copy_from) = default;
    vehicle_configuration &operator=(const vehicle_configuration &copy_from) = default;

    vehicle_configuration(vehicle_configuration &&pos) = default;
    vehicle_configuration &operator=(vehicle_configuration &&pos) = default;

    inline bool operator==(const vehicle_configuration &other) const
    {
        return location_ == other.location_ && heading_angle_ == other.heading_angle_;
    }

    inline bool is_valid() const
    {
        return is_valid_;
    }

    vehicle_configuration operator+(const vehicle_configuration &other) const
    {
        return vehicle_configuration(location_ + other.location_, heading_angle_ + other.heading_angle_);
    }

    inline const racer::math::vector &location() const { return location_; }
    inline const double &heading_angle() const { return heading_angle_; }
};

} // namespace racer

#endif
