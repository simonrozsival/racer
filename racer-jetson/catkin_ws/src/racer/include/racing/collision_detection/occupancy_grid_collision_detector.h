#ifndef OCCUPANCY_GRID_COLLISION_DETECTOR_H_
#define OCCUPANCY_GRID_COLLISION_DETECTOR_H_

#include "racing/vehicle_model/vehicle.h"
#include "racing/vehicle_model/vehicle_position.h"
#include "./occupancy_grid.h"

namespace racing {

    class collision_detector {
    public:
        collision_detector(std::vector<footprint> footprints)
            : footprints_(footprints)
        {
        }

        static std::unique_ptr<collision_detector> precalculate(std::size_t rotations, const racing::vehicle& vehicle, double cell_size) {
            std::vector<footprint> footprints;

            const double step = pi / (double)rotations;
            for (double angle = 0; angle < 2 * pi; angle += step) {
                footprints.push_back(footprint::of(angle, vehicle, cell_size));
            }

            return std::make_unique<collision_detector>(footprints);
        }

        bool collides(const vehicle_position& position, const occupancy_grid& grid) const {
            const auto& footprint = footprint_for_rotation(math::angle(position.heading_angle));
            return grid.collides(position.x, position.y, footprint);
        }

    private:
        const footprint& footprint_for_rotation(math::angle angle) const {
            const int index = int((angle / (2.0 * pi)) * footprints_.size());
            return footprints_[index];
        }

        std::vector<footprint> footprints_;
    };

}

#endif
