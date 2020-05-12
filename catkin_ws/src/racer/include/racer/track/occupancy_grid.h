#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "racer/math.h"
#include "racer/vehicle/configuration.h"

namespace racer::track
{
class occupancy_grid
{
public:
  occupancy_grid(const std::vector<int8_t> data, const uint32_t width, const uint32_t height, const double cell_size,
                 const racer::math::point origin)
      : cell_size_(cell_size), cell_size_sq_(cell_size * cell_size), data_(data), width_(width), height_(height), size_(data.size()), origin_(origin)
  {
  }

  occupancy_grid(const occupancy_grid &grid) = delete;
  occupancy_grid &operator=(const occupancy_grid &grid) = delete;

  occupancy_grid(occupancy_grid &&grid) = delete;
  occupancy_grid &operator=(occupancy_grid &&grid) = delete;

  std::unique_ptr<occupancy_grid> inflate(double radius) const
  {
    std::vector<int8_t> inflated{data_.begin(), data_.end()}; // start with a copy
    const int r = static_cast<int>(std::ceil(radius / cell_size_));
    const int r2 = r * r;

    for (std::size_t i{0}; i < width_; ++i)
    {
      for (std::size_t j{0}; j < height_; ++j)
      {
        const std::size_t index = index_of(i, j);
        if (!is_dangerous(data_[index]))
        {
          continue;
        }

        for (int dx{-r}; dx <= r; ++dx)
        {
          for (int dy{-r}; dy <= r; ++dy)
          {
            bool is_out_of_bounds = (dx < 0 && i < static_cast<std::size_t>(std::abs(dx))) ||
                                    (dy < 0 && j < static_cast<std::size_t>(std::abs(dy))) ||
                                    (dx > 0 && width_ <= i + static_cast<std::size_t>(std::abs(dx))) ||
                                    (dy > 0 && height_ <= j + static_cast<std::size_t>(std::abs(dy))) ||
                                    (dx == 0 && dy == 0);
            if (is_out_of_bounds)
            {
              continue;
            }

            if (dx * dx + dy * dy <= r2)
            {
              std::size_t x = i + dx;
              std::size_t y = j + dy;
              auto offset_index = index_of(x, y);
              inflated[offset_index] = data_[index];
            }
          }
        }
      }
    }

    return std::make_unique<occupancy_grid>(inflated, width_, height_, cell_size_, origin_);
  }

  inline int8_t value_at(double x, double y) const
  {
    int x_cell = int((x - origin_.x()) / cell_size_);
    int y_cell = int((y - origin_.y()) / cell_size_);
    const std::size_t index = index_of(x_cell, y_cell);

    int8_t val = index < 0 || index >= size_ ? max_value() // no information
                                             : data_[index];

    return val;
  }

  inline bool collides(racer::math::point point) const
  {
    return collides(point.x(), point.y());
  }

  inline bool collides(double x, double y) const
  {
    return is_dangerous(value_at(x, y));
  }

  double distance_to_closest_obstacle(const racer::math::point &center, double max_radius) const
  {
    if (collides(center))
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

  double find_distance(const racer::math::point &point, const double angle, const double max_radius) const
  {
    // this algorithm could be more accurate
    double distance = 0;
    const auto step = racer::math::vector(cell_size_ * std::cos(angle), cell_size_ * std::sin(angle));

    auto pt = point;

    while (distance < max_radius)
    {
      pt = pt + step;

      if (collides(pt))
        return distance;

      distance += cell_size_;
    }

    return std::min(max_radius, distance);
  }

  bool are_directly_visible(const racer::math::point &a, const racer::math::point &b) const
  {
    double distance = (a - b).length();
    auto step = (b - a) * (cell_size_ / distance);

    auto pt = a;
    while (pt.distance_sq(b) >= cell_size_sq_)
    {
      pt += step;
      if (collides(pt))
      {
        return false;
      }
    }

    return true;
  }

  racer::vehicle::configuration move_towards_center(const racer::vehicle::configuration &cfg) const
  {
    const double max_radius = 5.0; // meters
    const racer::math::angle left = cfg.heading_angle() + racer::math::angle::from_degrees(90);
    const racer::math::angle right = cfg.heading_angle() - racer::math::angle::from_degrees(90);

    const double distance_left = find_distance(cfg.location(), left, max_radius);
    const double distance_right = find_distance(cfg.location(), right, max_radius);
    const double radius = (distance_left + distance_right) / 2;

    const double shift_right = distance_right - radius;
    const racer::math::point center = cfg.location() + racer::math::vector(shift_right, right);
    return {center, cfg.heading_angle()};
  }

  static constexpr int8_t max_value()
  {
    return std::numeric_limits<int8_t>::max();
  }

  std::vector<int8_t> raw_data() const
  {
    return data_;
  }
  int rows() const
  {
    return height_;
  }
  int cols() const
  {
    return width_;
  }
  double cell_size() const
  {
    return cell_size_;
  }
  racer::math::point origin() const
  {
    return origin_;
  }

private:
  double cell_size_, cell_size_sq_;
  std::vector<int8_t> data_;
  uint32_t width_;
  uint32_t height_;
  std::size_t size_;
  racer::math::point origin_;

  inline std::size_t index_of(std::size_t x, std::size_t y) const
  {
    return y * width_ + x;
  }

  inline bool is_dangerous(int8_t value) const
  {
    return value >= 50;
  }
};

} // namespace racer
