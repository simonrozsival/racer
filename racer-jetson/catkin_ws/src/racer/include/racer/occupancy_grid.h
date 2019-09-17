#ifndef OCCUPANCY_GRID_H_
#define OCCUPANCY_GRID_H_

#include <vector>
#include <list>
#include <iostream>
#include <algorithm>
#include <vector>
#include <limits>
#include <cmath>

#include "racer/math/primitives.h"
#include "racer/vehicle_model/vehicle.h"
#include "racer/vehicle_model/kinematic_bicycle_model.h"

namespace racer
{

class occupancy_grid
{
public:
    const double cell_size;

    occupancy_grid(
        const std::vector<uint8_t> data,
        const uint32_t width,
        const uint32_t height,
        const double cell_size,
        const racer::math::point origin)
        : cell_size(cell_size),
          data_(data),
          width_(width),
          height_(height),
          size_(data.size()),
          origin_(origin)
    {
    }

    uint8_t value_at(double x, double y) const
    {
        int x_cell = int((x - origin_.x) / cell_size);
        int y_cell = int((y - origin_.y) / cell_size);
        const std::size_t index = index_of(x_cell, y_cell);

        uint8_t val = index < 0 || index >= size_
                          ? max_value() // no information
                          : data_[index];

        return val;
    }

    bool collides(double x, double y) const
    {
        return value_at(x, y) >= 50;
    }

    double distance_to_closest_obstacle(const racer::math::point &center, double max_radius) const
    {
        if (collides(center.x, center.y))
        {
            return 0;
        }

        double min_so_far = max_radius;
        const double number_of_samples = 60;
        const double step = 2 * pi / number_of_samples;

        for (double angle = 0; angle < 2 * pi; angle += step)
        {
            min_so_far = std::min(min_so_far, find_distance(center, angle, min_so_far));
        }

        return min_so_far;
    }

    static constexpr uint8_t max_value()
    {
        return std::numeric_limits<uint8_t>::max();
    }

    std::vector<uint8_t> raw_data() const { return data_; }
    const int rows() const { return height_; }
    const int cols() const { return width_; }

private:
    const std::vector<uint8_t> data_;
    const uint32_t width_;
    const uint32_t height_;
    const std::size_t size_;
    const racer::math::point origin_;

    std::size_t index_of(std::size_t x, std::size_t y) const
    {
        return y * width_ + x;
    }

    double find_distance(const racer::math::point &point, const double angle, const double max_radius) const
    {
        // this algorithm could be more accurate
        double distance = 0;
        const auto step = racer::math::vector(cell_size * std::cos(angle), cell_size * std::sin(angle));

        double x = point.x - origin_.x;
        double y = point.y - origin_.y;

        while (distance < max_radius)
        {
            x += step.x;
            y += step.y;

            if (collides(x, y))
            {
                return distance;
            }

            distance += cell_size;
        }

        return std::min(max_radius, distance);
    }
};

} // namespace racer

#endif
