#ifndef PLOT_H_
#define PLOT_H_

#include <iostream>
#include <vector>
#include <list>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include "matplotlib-cpp/matplotlibcpp.h"
#pragma GCC diagnostic pop

#include "standalone-experiments/input.h"

#include "racer/math/primitives.h"
#include "racer/vehicle_position.h"

namespace plt = matplotlibcpp;

void plot_circles(const std::list<racer::math::circle> &circles, const std::string &format, const double cell_size)
{
    std::vector<double> circles_x, circles_y;
    for (const auto &circle : circles)
    {
        circles_x.push_back(circle.center.x / cell_size);
        circles_y.push_back(circle.center.y / cell_size);
    }
    plt::plot(circles_x, circles_y, format);
}

void plot_points(const std::list<racer::math::point> &points, const std::string &format, const double cell_size)
{
    std::vector<double> points_x, points_y;
    for (const auto &point : points)
    {
        points_x.push_back(point.x / cell_size);
        points_y.push_back(point.y / cell_size);
    }
    plt::plot(points_x, points_y, format);
}

void plot_grid(const std::unique_ptr<racer::occupancy_grid> &occupancy_grid, unsigned char *img)
{
    img = new unsigned char[occupancy_grid->cols() * occupancy_grid->rows()];
    const auto raw_grid = occupancy_grid->raw_data();
    for (std::size_t i = 0; i < raw_grid.size(); ++i)
        img[i] = (unsigned char)(25.0 * (double)(255 - raw_grid[i]) / 255.0); // 10% alpha inverted bw color

    plt::imshow(img, occupancy_grid->rows(), occupancy_grid->cols(), 1, {{"cmap", "gray"}});
}

void plot_track_analysis(
    const track_analysis_input &config,
    const std::list<racer::math::circle> &circles,
    const std::list<racer::math::point> &waypoints)
{
    plt::title("Path of Circles");
    plt::grid(true);
    plt::xlim(0, config.occupancy_grid->cols());
    plt::ylim(0, config.occupancy_grid->rows());
    plt::subplot(1, 1, 1);

    unsigned char *img = nullptr;
    plot_grid(config.occupancy_grid, img);
    plot_points(config.checkpoints, "bx", config.occupancy_grid->cell_size);
    plot_points({config.initial_position.location()}, "rx", config.occupancy_grid->cell_size);

    plot_circles(circles, "r-", config.occupancy_grid->cell_size);
    plot_points(waypoints, "go", config.occupancy_grid->cell_size);

    plt::show();

    delete[] img;
}

#endif
