#ifndef FOOTPRINT_H_
#define FOOTPRINT_H_

#include "math/primitives.h"
#include "racing/vehicle_model/vehicle.h"

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

        static footprint of(double angle, const racing::vehicle& vehicle, double cell_size) {
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

}

#endif