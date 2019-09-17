#ifndef OCCUPANCY_GRID_H_
#define OCCUPANCY_GRID_H_

#include <vector>
#include <list>
#include <iostream>
#include <algorithm>
#include <vector>
#include <limits>

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
        const std::vector<int8_t> data,
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

    int8_t value_at(double x, double y) const
    {
        int x_cell = int((x - origin_.x) / cell_size);
        int y_cell = int((y - origin_.y) / cell_size);
        const std::size_t index = index_of(x_cell, y_cell);

        return index < 0 || index >= size_
                   ? max_value() // no information
                   : data_[index];
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
            double distance_to_obstacle = find_distance(center, angle, max_radius);
            min_so_far = std::min(min_so_far, distance_to_obstacle);
        }

        return min_so_far;
    }

    static std::unique_ptr<occupancy_grid> load(std::vector<std::string> lines, double cell_size)
    {
        uint16_t width = (uint16_t)lines[0].length();
        uint16_t height = (uint16_t)lines.size();
        std::size_t size = width * height;

        std::vector<int8_t> data;
        data.resize(size);

        for (std::size_t i = 0; i < height; ++i)
        {
            for (std::size_t j = 0; j < width; ++j)
            {
                const std::size_t index = i * width + j;
                data[index] = lines[i][j] == ' ' ? 0 : max_value();
            }
        }

        return std::make_unique<occupancy_grid>(data, width, height, cell_size, racer::math::point(0, 0));
    }

    static constexpr int8_t max_value()
    {
        return std::numeric_limits<int8_t>::max();
    }

    std::vector<int8_t> raw_data() const { return data_; }
    const int rows() const { return height_; }
    const int cols() const { return width_; }

private:
    const std::vector<int8_t> data_;
    const uint32_t width_;
    const uint32_t height_;
    const std::size_t size_;
    const racer::math::point origin_;

    std::size_t index_of(std::size_t x, std::size_t y) const
    {
        return y * width_ + x;
    }

    bool is_occupied(std::size_t x, std::size_t y) const
    {
        const std::size_t index = index_of(x, y);
        return index < 0 || index >= size_ || data_[index] > 0;
    }

    double find_distance(const racer::math::point &point, const double angle, const double max_radius) const
    {
        // this algorithm could be more accurate
        double distance = 0;
        const racer::math::vector step(cell_size * cos(angle), cell_size * sin(angle));

        double x = point.x - origin_.x;
        double y = point.y - origin_.y;

        while (distance < max_radius)
        {
            x += step.x;
            y += step.y;

            if (x < 0 || y < 0 || is_occupied(std::size_t(x / cell_size), std::size_t(y / cell_size)))
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
