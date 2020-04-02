#pragma once

#include <iostream>
#include <vector>
#include <assert.h>

#include "racer/vehicle_configuration.h"
#include "racer/occupancy_grid.h"
#include "racer/math.h"
#include "racer/sehs/space_exploration.h"
#include "racer/track/centerline.h"

namespace racer::track
{
struct corner_point {
    const racer::math::point grid_coordinate;
    const centerline::coordinate centerline_coordinate;
    const double maximum_speed;

    corner_point(racer::math::point grid_coord, centerline::coordinate centerline_coord, double speed)
        : grid_coordinate{grid_coord}, centerline_coordinate{centerline_coord}, maximum_speed{speed}
    {
    }
};

struct corner {
    const corner_point turn_in;
    const corner_point apex;
    const corner_point exit;

    corner(corner_point t, corner_point a, corner_point e)
        : turn_in{t}, apex{a}, exit{e}
    {
    }

    bool is_valid() const {
        return turn_in.centerline_coordinate.distance_along < apex.centerline_coordinate.distance_along
            && apex.centerline_coordinate.distance_along < exit.centerline_coordinate.distance_along;
    }
};

class racing_line {
public:
    const std::vector<corner> corners;

    bool is_valid() const {
        for (std::size_t i{0}; i < corners.size(); ++i) {
            const std::size_t next = (i + 1) % corners.size();
            if (!corners[i].is_valid()
                || corners[next].turn_in.centerline_coordinate.distance_along < corners[i].exit.centerline_coordinate.distance_along) {
                return false;
            }
        }
    }

    static std::unique_ptr<racing_line> construct_trivial(centerline line, std::vector<racer::math::point> corner_locations, double maximum_speed) {
        std::vector<corner> corners;
        corners.reserve(corner_locations.size());

        for (std::size_t i = 0; i < corner_locations.size(); ++i) {
            std::size_t prev = i == 0 ? corner_locations.size() - 1 : i - 1;
            std::size_t next = (i + 1) % corner_locations.size();

            const auto apex = corner_locations[i];
            const auto to_prev = corner_locations[prev] - apex;
            const auto to_next = corner_locations[next] - apex;
            const auto tangent = (corner_locations[next] - corner_locations[prev]).normalized();

            const auto turn_in = (-1.0 * tangent).interpolate_with(to_prev.normalized(), 0.5, 0.5) * (to_prev.length() * 0.33) + apex;
            const auto exit = tangent.interpolate_with(to_next.normalized(), 0.5, 0.5) * (to_next.length() * 0.33) + apex;

            corners.emplace_back(
                corner_point{ turn_in, line.coordinate_of(turn_in), maximum_speed },
                corner_point{ apex, line.coordinate_of(apex), maximum_speed / 5.0 },
                corner_point{ exit, line.coordinate_of(exit), maximum_speed });
        }

        return std::make_unique<racing_line>(line, corners);
    }

    racing_line(centerline line, std::vector<corner> corners)
        : line_{line}, corners{corners}
    {
    }

private:
    const centerline line_;
};

}