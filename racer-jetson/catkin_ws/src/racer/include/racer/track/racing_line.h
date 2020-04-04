#pragma once

#include <iostream>
#include <vector>
#include <assert.h>

#include "racer/vehicle_configuration.h"
#include "racer/occupancy_grid.h"
#include "racer/math.h"
#include "racer/sehs/space_exploration.h"
#include "racer/splines/catmull_rom.h"
#include "racer/track/centerline.h"

namespace racer::track
{
struct point {
    const racer::math::point coordinate;
    const double maximum_speed;

    point(racer::math::point grid_coord, double speed)
        : coordinate{grid_coord}, maximum_speed{speed}
    {
    }
};

struct corner {
    const point turn_in;
    const point apex;
    const point exit;

    corner(point t, point a, point e)
        : turn_in{t}, apex{a}, exit{e}
    {
    }
};

class racing_line {
public:
    static std::unique_ptr<racing_line> basic_from_apexes(
        std::shared_ptr<racer::occupancy_grid> grid,
        std::vector<racer::math::point> corner_locations,
        double max_speed)
    {
        std::vector<corner> corners;
        corners.reserve(corner_locations.size());

        for (std::size_t i = 0; i < corner_locations.size(); ++i) {
            std::size_t prev = i == 0 ? corner_locations.size() - 1 : i - 1;
            std::size_t next = (i + 1) % corner_locations.size();

            const auto apex = corner_locations[i];
            const auto to_prev = corner_locations[prev] - apex;
            const auto to_next = corner_locations[next] - apex;

            // calculate turn-in and exit points
            const auto tangent = (corner_locations[next] - corner_locations[prev]).normalized();

            const auto turn_in = (-1.0 * tangent).interpolate_with(to_prev.normalized(), 0.5, 0.5) * (to_prev.length() * 0.3) + apex;
            const auto exit = tangent.interpolate_with(to_next.normalized(), 0.5, 0.5) * (to_next.length() * 0.3) + apex;

            // the smaller the angle at the apex, the smaller speed
            const auto cos_angle = to_prev.dot(to_next) / (to_prev.length() * to_next.length());
            const auto speed_proportion = 1 - (cos_angle + 1) / 2.0;
            const auto min_speed = std::max(0.25, speed_proportion * max_speed);

            std::cout << "angle: " << acos(cos_angle) << ", speed prop: " << speed_proportion << std::endl;
            bool slow_turn_in = speed_proportion < 0.5;

            corners.emplace_back(
                point{turn_in, slow_turn_in ? std::min(0.6 * max_speed, 3 * min_speed) : max_speed},
                point{apex, min_speed},
                point{exit, max_speed});
        }

        return std::make_unique<racing_line>(corners);
    }

    racing_line(std::vector<corner> corners)
        : corners_{corners},
          spline_{enumerate_spline(corners)}
    {
    }
    
    racing_line(std::vector<corner> corners, std::vector<point> spline)
        : corners_{corners},
          spline_{spline}
    {
    }

    std::size_t closest_point_along_the_spline(racer::math::point pt) const {
        std::size_t closest = 0;
        double min_distance = pt.distance_sq(spline_[0].coordinate);

        for (std::size_t i{1}; i < spline_.size(); ++i) {
        const auto dist_sq = pt.distance_sq(spline_[i].coordinate);
            if (dist_sq <= min_distance) {
                min_distance = dist_sq;
                closest = i;
            }
        }

        return closest;
    }

    const std::vector<corner>& corners() const {
        return corners_;
    }

    const std::vector<point>& spline() const {
        return spline_;
    }

private:
    std::vector<corner> corners_;
    std::vector<point> spline_;

    std::vector<point> enumerate_spline(std::vector<corner> corners) const {
        std::vector<racer::math::point> control_points;
        std::vector<double> maximum_speeds;
        for (const auto &corner : corners) {
            control_points.push_back(corner.turn_in.coordinate);
            maximum_speeds.push_back(corner.turn_in.maximum_speed);

            control_points.push_back(corner.apex.coordinate);
            maximum_speeds.push_back(corner.apex.maximum_speed);

            control_points.push_back(corner.exit.coordinate);
            maximum_speeds.push_back(corner.exit.maximum_speed);
        }

        std::vector<point> points;

        // loop around the control points with a window of four consecutive control points a, b, c, d
        for (std::size_t b{0}; b < control_points.size(); ++b) {
            std::size_t a = b == 0 ? control_points.size() - 1 : b - 1;
            std::size_t c = (b + 1) % control_points.size();
            std::size_t d = (b + 2) % control_points.size();

            const double step = 0.1; // meters
            const auto next_segment = racer::splines::catmull_rom::enumerate_segment(
                control_points[a],
                control_points[b],
                control_points[c],
                control_points[d],
                step
            );

            // maximum speed for the points between `b` and `c` is the maximum speed
            // at the end of the segment, which is the point `c`
            const double max = maximum_speeds[c];
            for (const auto &pt : next_segment) {
                points.emplace_back(pt, max);
            }
        }

        return points;
    }
};

}