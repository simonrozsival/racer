#pragma once

#include <list>
#include <vector>
#include <algorithm>

#include "racer/math.h"
#include "racer/occupancy_grid.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/vehicle_chassis.h"

namespace racer::track
{

class collision_detection
{
    using footprint = std::vector<racer::math::point>;

private:
    std::shared_ptr<occupancy_grid> original_grid_;
    std::shared_ptr<occupancy_grid> inflated_grid_outer_radius_;

    std::vector<footprint> footprints_;

public:
    collision_detection(
        std::shared_ptr<occupancy_grid> grid,
        std::shared_ptr<racer::vehicle_model::vehicle_chassis> chassis,
        const std::size_t precision)
        : original_grid_{grid}
    {
        auto outer_radius = chassis->radius();
        inflated_grid_outer_radius_ = grid->inflate(outer_radius);

        const double step = 2 * M_PI / double(precision);
        for (double angle = 0; angle < 2 * M_PI; angle += step)
        {
            auto fp = calculate_footprint_of(angle, chassis, grid->cell_size());
            footprints_.push_back(fp);
        }
    }

    bool collides(const racer::vehicle_configuration &configuration) const
    {
        return maybe_collides(configuration) && definitely_collides(configuration);
    }

private:
    bool maybe_collides(const racer::vehicle_configuration &configuration) const
    {
        return inflated_grid_outer_radius_->collides(configuration.location());
    }

    bool definitely_collides(const racer::vehicle_configuration &configuration) const
    {
        auto fp = footprint_for(configuration.heading_angle());
        return std::any_of(
            std::begin(fp),
            std::end(fp),
            [&](const auto &cell_offset) {
                return original_grid_->collides(configuration.location() + cell_offset);
            });
    }

    const footprint &footprint_for(const racer::math::angle &angle) const
    {
        const int index = int((angle.to_normal_angle() / (2.0 * M_PI)) * footprints_.size());
        return footprints_[index];
    }

    static footprint calculate_footprint_of(double angle, std::shared_ptr<racer::vehicle_model::vehicle_chassis> chassis, double cell_size)
    {
        std::list<racer::math::point> cells;
oval
        double a = chassis->wheelbase + cell_size / 2;
        double b = chassis->width + cell_size / 2;

        racer::math::rectangle basic_shape(
            {a / 2, b / 2},
            {-a / 2, b / 2},
            {-a / 2, -b / 2},
            {a / 2, -b / 2});

        const auto rotated_shape = basic_shape.rotate(angle);

        // bounding rect
        double minX = std::min({rotated_shape.A.x(), rotated_shape.B.x(), rotated_shape.C.x(), rotated_shape.D.x()});
        double minY = std::min({rotated_shape.A.y(), rotated_shape.B.y(), rotated_shape.C.y(), rotated_shape.D.y()});
        double maxX = std::max({rotated_shape.A.x(), rotated_shape.B.x(), rotated_shape.C.x(), rotated_shape.D.x()});
        double maxY = std::max({rotated_shape.A.y(), rotated_shape.B.y(), rotated_shape.C.y(), rotated_shape.D.y()});

        for (double x = minX; x < maxX; x += cell_size)
        {
            for (double y = maxY; y > minY; y -= cell_size)
            {
                racer::math::rectangle cell(
                    {x, y},
                    {x + cell_size, y},
                    {x + cell_size, y + cell_size},
                    {x, y + cell_size});

                // Improvement: skip the "inner" cells within the rectangle and keep
                // only the cells on the edge of the vehicle to improve the performance.

                if (cell.intersects(rotated_shape))
                {
                    cells.emplace_back(x + cell_size / 2, y + cell_size / 2);
                }
            }
        }

        cells.sort();
        cells.unique();
        return {
            std::vector<racer::math::point>{std::begin(cells), std::end(cells)}};
    }
}; // namespace racer::track

} // namespace racer::track
