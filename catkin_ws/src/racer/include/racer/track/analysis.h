#pragma once

#include <algorithm>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include "racer/math.h"
#include "racer/track/occupancy_grid.h"
#include "racer/vehicle/configuration.h"

using namespace racer::math;

namespace racer::track
{

  class analysis
  {
  public:
    analysis(const double d_min)
        : min_distance_between_waypoints_sq_{d_min * d_min} {}

    const std::vector<point>
    find_pivot_points(const std::vector<circle> &circle_path,
                      const std::shared_ptr<racer::track::occupancy_grid> grid) const
    {
      std::vector<point> pivot_points;

      if (circle_path.empty())
      {
        return {};
      }

      auto prev_circle = circle_path.front();
      auto last_circle = prev_circle;

      for (auto &next_step : circle_path)
      {
        bool are_directly_visible =
            grid->are_directly_visible(last_circle.center(), next_step.center());
        if (!are_directly_visible)
        {
          pivot_points.push_back(prev_circle.center());
          last_circle = std::move(prev_circle);
        }

        prev_circle = std::move(next_step);
      }

      return pivot_points;
    }

    const std::vector<point>
    remove_insignificant_turns(const std::vector<point> &points) const
    {
      std::vector<bool> used(points.size(),
                             true); // all points are considered at the beginning

      for (std::size_t i = 0; i < points.size(); ++i)
      {
        double angle = angle_at(i, points, used);
        if (angle > max_angle_)
        {
          used[i] = false;
        }
      }

      std::vector<point> remaining;
      for (std::size_t i = 0; i < points.size(); ++i)
      {
        if (used[i])
        {
          remaining.push_back(points[i]);
        }
      }

      return remaining;
    }

    const std::vector<point> merge_close(const std::vector<point> &points) const
    {
      std::vector<bool> used(points.size(),
                             true); // all points are used at the beginning

      std::size_t i_max, max_close = 0;
      do
      {
        max_close = 0;

        for (std::size_t i = 0; i < points.size(); ++i)
        {
          if (!used[i])
            continue;
          const auto close = close_points(points, used, i);
          if (close.size() > max_close)
          {
            max_close = close.size();
            i_max = i;
          }
        }

        if (max_close > 0)
        {
          auto to_remove = close_points(points, used, i_max);
          to_remove.insert(i_max);
          const auto i_mid = pick_corner_point_index(points, to_remove, used);
          to_remove.erase(i_mid);

          // remove all the points except for the mid one
          for (std::size_t i : to_remove)
          {
            used[i] = false;
          }
        }
      } while (max_close > 0);

      std::vector<point> remaining;
      for (std::size_t i = 0; i < points.size(); ++i)
      {
        if (used[i])
        {
          remaining.push_back(points[i]);
        }
      }

      return remaining;
    }

  private:
    const double min_distance_between_waypoints_sq_;
    const double max_angle_{M_PI * 4.0 / 5.0};

    const std::set<std::size_t> close_points(const std::vector<point> &points,
                                             const std::vector<bool> &used,
                                             std::size_t i) const
    {
      std::set<std::size_t> close;

      std::size_t j = next(i, used);
      while (j != i && points[i].distance_sq(points[j]) <
                           min_distance_between_waypoints_sq_)
      {
        auto res = close.insert(j);
        if (!res.second)
        {
          break;
        }

        j = next(j, used);
      }

      j = prev(i, used);
      while (j != i && points[i].distance_sq(points[j]) <
                           min_distance_between_waypoints_sq_)
      {
        auto res = close.insert(j);
        if (!res.second)
        {
          break;
        }

        j = prev(j, used);
      }

      return close;
    }

    const std::size_t
    pick_corner_point_index(const std::vector<racer::math::point> &points,
                            const std::set<std::size_t> &indices,
                            const std::vector<bool> &used) const
    {
      if (indices.empty())
      {
        throw std::runtime_error(
            "Cannot pick a corner point from empty set of points.");
      }

      double acutest_angle = 2 * M_PI;
      std::size_t corner_i = 0;

      for (const auto i : indices)
      {
        double angle = angle_at(i, points, used);
        if (angle < acutest_angle)
        {
          acutest_angle = angle;
          corner_i = i;
        }
      }

      return corner_i;
    }

    const double angle_at(std::size_t i,
                          const std::vector<racer::math::point> &points,
                          const std::vector<bool> &used) const
    {
      const auto a = points[next(i, used)] - points[i];
      const auto b = points[prev(i, used)] - points[i];
      const auto A = a.length();
      const auto B = b.length();

      if (A == 0 || B == 0)
        return 0;

      return std::acos(a.dot(b) / (A * B));
    }

    const std::size_t next(std::size_t i, const std::vector<bool> &used) const
    {
      std::size_t j = i;
      do
      {
        j = (j + 1) % used.size();
      } while (j != i && !used[j]);

      return j;
    }

    const std::size_t prev(std::size_t i, const std::vector<bool> &used) const
    {
      std::size_t j = i;
      do
      {
        j = (j + used.size() - 1) % used.size();
      } while (j != i && !used[j]);

      return j;
    }
  };

} // namespace racer