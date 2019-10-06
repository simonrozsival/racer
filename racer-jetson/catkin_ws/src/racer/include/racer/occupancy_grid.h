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

    occupancy_grid()
        : cell_size_{0},
          data_{},
          width_{0},
          height_{0},
          size_{0},
          origin_{}
    {
    }

    occupancy_grid(
        const std::vector<uint8_t> data,
        const uint32_t width,
        const uint32_t height,
        const double cell_size,
        const racer::math::point origin)
        : cell_size_(cell_size),
          data_(data),
          width_(width),
          height_(height),
          size_(data.size()),
          origin_(origin)
    {
    }

    occupancy_grid(const occupancy_grid& grid) = default;
    occupancy_grid& operator=(const occupancy_grid& grid) = default;

    occupancy_grid(occupancy_grid&& grid) = default;
    occupancy_grid& operator=(occupancy_grid&& grid) = default;

    uint8_t value_at(double x, double y) const
    {
        int x_cell = int((x - origin_.x()) / cell_size_);
        int y_cell = int((y - origin_.y()) / cell_size_);
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
        if (collides(center.x(), center.y()))
        {
            return 0;
        }

        double min_so_far = max_radius;
        const double number_of_samples = 60;
        const double step = 2 * M_PI / number_of_samples;

        for (double angle = 0; angle < 2 * M_PI; angle += step)
        {
            min_so_far = std::min(min_so_far, find_distance(center, angle, min_so_far));
        }

        return min_so_far;
    }

    static constexpr uint8_t max_value()
    {
        return std::numeric_limits<uint8_t>::max();
    }

    bool is_valid() const {
        return size_ > 0;
    }

    std::vector<uint8_t> raw_data() const { return data_; }
    int rows() const { return height_; }
    int cols() const { return width_; }
    double cell_size() const { return cell_size_; }

private:
    double cell_size_;
    std::vector<uint8_t> data_;
    uint32_t width_;
    uint32_t height_;
    std::size_t size_;
    racer::math::point origin_;

    std::size_t index_of(std::size_t x, std::size_t y) const
    {
        return y * width_ + x;
    }

    double find_distance(const racer::math::point &point, const double angle, const double max_radius) const
    {
        // this algorithm could be more accurate
        double distance = 0;
        const auto step = racer::math::vector(cell_size_ * std::cos(angle), cell_size_ * std::sin(angle));

        auto pt = point - origin_;

        while (distance < max_radius)
        {
            pt = pt + step;

            if (collides(pt.x(), pt.y()))
            {
                return distance;
            }

            distance += cell_size_;
        }

        return std::min(max_radius, distance);
    }
};

} // namespace racer

#endif
