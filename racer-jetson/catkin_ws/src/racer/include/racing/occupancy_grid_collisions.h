#ifndef OCCUPANCY_GRID_COLLISION_H_
#define OCCUPANCY_GRID_COLLISION_H_

#include <vector>
#include <list>
#include <iostream>
#include <algorithm>
#include <vector>

#include "../math/primitives.h"
#include "./kinematic_bicycle_model.h"

namespace racing {

    struct index2d {
        const int x, y;
        index2d(int x, int y) : x(x), y(y) { }

        bool operator ==(const index2d& other) const {
            return x == other.x && y == other.y;
        }

        bool operator <(const index2d& other) const {
            return y < other.y && x < other.x;
        }
    };

    struct footprint {
        const std::list<index2d> occupied_cells;

        static footprint of(double angle, const racing::vehicle_properties& vehicle, double cell_size) {
            std::list<index2d> cells;

            double a = vehicle.wheelbase + cell_size / 2;
            double b = vehicle.width + cell_size / 2;

            math::rectangle basic_shape(
                math::point(a / 2, b / 2),
                math::point(-a / 2, b / 2),
                math::point(-a / 2, -b / 2),
                math::point(a / 2, -b / 2)
            );

            auto rotated_shape = basic_shape.rotate(angle);

            // bounding rect
            double minX = std::min({ rotated_shape.A.x, rotated_shape.B.x, rotated_shape.C.x, rotated_shape.D.x });
            double minY = std::min({ rotated_shape.A.y, rotated_shape.B.y, rotated_shape.C.y, rotated_shape.D.y });
            double maxX = std::max({ rotated_shape.A.x, rotated_shape.B.x, rotated_shape.C.x, rotated_shape.D.x });
            double maxY = std::max({ rotated_shape.A.y, rotated_shape.B.y, rotated_shape.C.y, rotated_shape.D.y });

            for (double x = minX; x < maxX; x += cell_size) {
                for (double y = maxY; y > minY; y -= cell_size) {
                    math::rectangle cell(
                        math::point(x, y),
                        math::point(x + cell_size, y),
                        math::point(x + cell_size, y + cell_size),
                        math::point(x, y + cell_size)
                    );

                    if (cell.intersects(rotated_shape)) {
                        cells.push_back(
                            index2d(
                                int(floor(x / cell_size + 0.5)),
                                int(floor(y / cell_size + 0.5))
                            ));
                    }
                }
            }

            cells.sort();
            cells.unique();
            return footprint(cells);
        }

        bool operator ==(const footprint& other) const {
            return occupied_cells == other.occupied_cells;
        }

    private:
        footprint(std::list<index2d> cells)
            : occupied_cells(cells)
        {
        }
    };

    class occupancy_grid {
    public:
        const double cell_size;

        occupancy_grid(
            const std::vector<signed char> data,
            const uint32_t width,
            const uint32_t height,
            const double cell_size,
            const math::point origin)
            : data_(data),
            width_(width),
            height_(height),
            cell_size(cell_size),
            size_(data.size()),
            origin_(origin)
        {
        }

        bool collides(double x, double y, const footprint& fp) const {
            int x_cell = int((x - origin_.x) / cell_size);
            int y_cell = int((y - origin_.y) / cell_size);

            return std::any_of(
                fp.occupied_cells.cbegin(),
                fp.occupied_cells.cend(),
                [this, x_cell, y_cell](const auto & cell) {
                    return is_occupied(x_cell + cell.x, y_cell + cell.y);
                });
        }

        bool is_occupied(int x, int y) const {
            const int index = index_of(x, y);
            return index < 0 || index >= size_ || data_[index] > 0;
        }

        double distance_to_closest_obstacle(const math::point& center, double max_radius) const {
            int x = int((center.x - origin_.x) / cell_size);
            int y = int((center.y - origin_.y) / cell_size);

            if (is_occupied(x, y)) {
                return 0;
            }

            double min_so_far = max_radius;
            const double number_of_samples = 60;
            const double step = 2 * pi / number_of_samples;

            for (double angle = 0; angle < 2 * pi; angle += step) {
                double distance_to_obstacle = find_distance(center, angle, max_radius);
                min_so_far = std::min(min_so_far, distance_to_obstacle);
            }

            return min_so_far;
        }

        static std::unique_ptr<occupancy_grid> load(std::vector<std::string> lines, double cell_size) {
            uint16_t width = (uint16_t)lines[0].length();
            uint16_t height = (uint16_t)lines.size();
            std::size_t size = width * height;

            std::vector<signed char> data;
            data.resize(size);

            for (std::size_t i = 0; i < height; ++i) {
                for (std::size_t j = 0; j < width; ++j) {
                    const std::size_t index = i * width + j;
                    data[index] = lines[i][j] == ' ' ? 0 : 100;
                }
            }

            return std::make_unique<occupancy_grid>(data, width, height, cell_size, math::point(0, 0));
        }
    private:
        const std::vector<signed char> data_;
        const uint32_t width_;
        const uint32_t height_;
        const std::size_t size_;
        const math::point origin_;

        int index_of(int x, int y) const {
            return y * width_ + x;
        }

        double find_distance(const math::point& point, const double angle, const double max_radius) const {
            // this algorithm could be more accurate
            double distance = 0;
            const math::vector step(cell_size * cos(angle), cell_size * sin(angle));

            double x = point.x - origin_.x;
            double y = point.y - origin_.y;

            while (distance < max_radius) {
                x += step.x;
                y += step.y;

                if (is_occupied(int(x / cell_size), int(y / cell_size))) {
                    return distance;
                }

                distance += cell_size;
            }

            return std::min(max_radius, distance);
        }
    };

    class collision_detector {
    public:
        collision_detector(std::vector<footprint> footprints)
            : footprints_(footprints)
        {
        }

        static std::unique_ptr<collision_detector> precalculate(std::size_t rotations, const racing::vehicle_properties& vehicle, double cell_size) {
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
