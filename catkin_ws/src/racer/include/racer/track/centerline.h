#pragma once

#include <assert.h>
#include <iostream>
#include <vector>

#include "racer/math.h"
#include "racer/track/occupancy_grid.h"
#include "racer/sehs/space_exploration.h"
#include "racer/vehicle/configuration.h"

namespace racer::track
{

  class centerline
  {
  private:
    centerline(std::vector<racer::math::point> points, double width)
        : points_{points}, width_{width}
    {
    }

  public:
    static centerline find(const racer::vehicle::configuration &start,
                           const std::shared_ptr<racer::track::occupancy_grid> grid,
                           const std::vector<racer::math::point> &checkpoints)
    {
      const auto adjusted_start = grid->move_towards_center(start);
      const auto radius =
          grid->distance_to_closest_obstacle(adjusted_start.location(), 5.0);

      racer::sehs::space_exploration space_exploration{0.6 * radius, 2.0 * radius,
                                                       12};
      const auto circles =
          space_exploration.explore_grid(grid, adjusted_start, checkpoints);

      std::vector<racer::math::point> points;
      points.reserve(circles.size());
      for (const auto &c : circles)
      {
        points.push_back(c.center());
      }

      return centerline{points, 2 * radius};
    }

    bool empty() const { return points_.empty(); }
    double width() const { return width_; }
    const std::vector<racer::math::point> points() const { return points_; }

  private:
    const std::vector<racer::math::point> points_;
    const double width_;
  };

} // namespace racer::track
