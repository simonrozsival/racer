#pragma once

#include <array>
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

namespace racer::vehicle
{
    struct rpm
    {
    private:
        double rpm_;

    public:
        rpm(double rpm) : rpm_{rpm} {}

        rpm(const rpm &other) = default;
        rpm &operator=(const rpm &other) = default;

        rpm(rpm &&other) = default;
        rpm &operator=(rpm &&other) = default;

        inline bool operator==(const rpm &other) const { return *this == other.rpm_; }

        inline bool operator==(double other_rpm) const
        {
            return std::abs(rpm_ - other_rpm) < 1e-3;
        }

        rpm operator+(const rpm &other) const { return rpm{rpm_ + other.rpm_}; }

        rpm operator*(const double &factor) const { return rpm{rpm_ * factor}; }

        inline operator double() const { return rpm_; }

        constexpr auto to_radians_per_second() const
        {
            return rpm_ * (2 * M_PI / 60);
        }
    };
} // namespace racer::vehicle