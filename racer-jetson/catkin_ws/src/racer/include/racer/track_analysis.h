#ifndef TRACK_ANALYSIS_H_
#define TRACK_ANALYSIS_H_

#include <vector>
#include <list>
#include <set>
#include <utility>
#include <algorithm>

#include "racer/math/primitives.h"
#include "racer/occupancy_grid.h"

using namespace racer::math;

namespace racer
{

class track_analysis
{
public:
    track_analysis(
        const occupancy_grid &grid,
        const double min_distance_between_waypoints)
        : grid_(grid),
          min_distance_between_waypoints_(min_distance_between_waypoints)
    {
    }

    const std::vector<point> find_pivot_points(const std::list<circle> circle_path) const
    {
        std::vector<point> pivot_points;

        auto prev_circle = std::make_unique<circle>(circle_path.front());
        auto last_circle = std::make_unique<circle>(*prev_circle);

        // first iteration
        for (const auto &next_step : circle_path)
        {
            if (!are_directly_visible(last_circle->center(), next_step.center()))
            {
                pivot_points.push_back(prev_circle->center());
                last_circle = std::move(prev_circle);
            }

            prev_circle = std::make_unique<circle>(next_step);
        }

        // continue from the beginning
        for (const auto &next_step : circle_path)
        {
            if (next_step.center() == pivot_points.front())
            {
                break;
            }

            if (!are_directly_visible(last_circle->center(), next_step.center()))
            {
                pivot_points.push_back(prev_circle->center());
                last_circle = std::move(prev_circle);
            }

            prev_circle = std::make_unique<circle>(next_step);
        }

        return pivot_points;
    }

    std::list<point> find_corners(std::vector<point> pivot_points, double max_angle)
    {
        const auto sharp_turns = remove_insignificant_turns(pivot_points, max_angle);
        return merge_close(sharp_turns, min_distance_between_waypoints_);
    }

private:
    const occupancy_grid &grid_;
    const double min_distance_between_waypoints_;

    bool are_directly_visible(const point &a, const point &b) const
    {
        double cell_size_sq = grid_.cell_size() * grid_.cell_size();
        double distance = (a - b).length();
        auto step = (b - a) * (grid_.cell_size() / distance);

        auto pt = a;
        while (pt.distance_sq(b) >= cell_size_sq)
        {
            pt += step;
            if (grid_.collides(pt.x(), pt.y()))
            {
                return false;
            }
        }

        return true;
    }

    const std::vector<point> remove_insignificant_turns(const std::vector<point> &points, double max_angle) const
    {
        std::vector<bool> used(points.size(), true); // all points are used at the beginning

        for (std::size_t i = 0; i < points.size(); ++i)
        {
            double angle = angle_at(i, points, used);
            if (angle > max_angle)
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

    const std::list<point> merge_close(const std::vector<point> &points, double d_min) const
    {
        std::vector<bool> used(points.size(), true); // all points are used at the beginning

        std::size_t i_max, max_close = 0;
        do
        {
            max_close = 0;

            for (std::size_t i = 0; i < points.size(); ++i)
            {
                if (!used[i])
                    continue;
                const auto close = close_points(points, used, i, d_min);
                if (close.size() > max_close)
                {
                    max_close = close.size();
                    i_max = i;
                }
            }

            if (max_close > 0)
            {
                auto to_remove = close_points(points, used, i_max, d_min);
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

        std::list<point> remaining;
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            if (used[i])
            {
                remaining.push_back(points[i]);
            }
        }

        return remaining;
    }

    const std::set<std::size_t> close_points(const std::vector<point> &points, const std::vector<bool> &used, std::size_t i, double d_min) const
    {
        std::set<std::size_t> close;

        std::size_t j = next(i, used);
        while (j != i && points[i].distance_sq(points[j]) < pow(d_min, 2))
        {
            auto res = close.insert(j);
            if (!res.second)
            {
                break;
            }

            j = next(j, used);
        }

        j = prev(i, used);
        while (j != i && points[i].distance_sq(points[j]) < pow(d_min, 2))
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

    const std::size_t pick_corner_point_index(const std::vector<racer::math::point> &points, const std::set<std::size_t> &indices, const std::vector<bool> &used) const
    {
        if (indices.empty())
        {
            throw std::runtime_error("Cannot pick a corner point from empty set of points.");
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

    const double angle_at(std::size_t i, const std::vector<racer::math::point> &points, const std::vector<bool> &used) const
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

#endif
