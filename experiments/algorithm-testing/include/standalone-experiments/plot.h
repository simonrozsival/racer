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
#include "racer/vehicle_configuration.h"

namespace plt = matplotlibcpp;

void plot_vehicle_configuration(const racer::vehicle_configuration &position, const double length, const double cell_size)
{
    auto end = racer::math::vector(length / cell_size, 0).rotate(-position.heading_angle());

    std::vector<double> x, y, u, v;
    x.push_back(position.location().x() / cell_size);
    y.push_back(position.location().y() / cell_size);
    u.push_back(end.x());
    v.push_back(end.y());

    plt::quiver(x, y, u, v);
}

void plot_path_of_circles(const std::string &name, const std::list<racer::math::circle> &circles, const std::string &format, const double cell_size)
{
    std::vector<double> circles_x, circles_y;
    for (const auto &circle : circles)
    {
        circles_x.push_back(circle.center().x() / cell_size);
        circles_y.push_back(circle.center().y() / cell_size);
    }
    plt::plot(circles_x, circles_y, format, {{"label", name}});
}

void plot_circles(const std::list<racer::math::point> &points, const racer::occupancy_grid &occupancy_grid, const double radius, unsigned char *img)
{
    img = new unsigned char[occupancy_grid.cols() * occupancy_grid.rows() * 4];
    const auto raw_grid = occupancy_grid.raw_data();
    for (std::size_t row = 0; row < (std::size_t)occupancy_grid.rows(); ++row)
    {
        for (std::size_t col = 0; col < (std::size_t)occupancy_grid.cols(); ++col)
        {
            racer::math::point c(col * occupancy_grid.cell_size(), row * occupancy_grid.cell_size());

            std::size_t number_of_circles = 0;
            for (const auto &p : points)
            {
                if (c.distance_sq(p) < std::pow(radius, 2))
                {
                    number_of_circles++;
                }
            }

            std::size_t index = row * occupancy_grid.cols() * 4 + col * 4;
            img[index] = 0;
            img[index + 1] = 255;
            img[index + 2] = 0;
            img[index + 3] = number_of_circles * 30;
        }
    }

    plt::imshow(img, occupancy_grid.rows(), occupancy_grid.cols(), 4);
}

void plot_points(const std::string &name, const std::list<racer::math::point> &points, const std::string &format, const double cell_size)
{
    std::vector<double> points_x, points_y;
    for (const auto &point : points)
    {
        points_x.push_back(point.x() / cell_size);
        points_y.push_back(point.y() / cell_size);
    }
    double size = cell_size * 200;
    plt::plot(points_x, points_y, format, {{"label", name}, {"markersize", std::to_string(size)}});
}

void plot_trajectory(const std::string &name, const racer::vehicle_model::kinematic_bicycle_model::trajectory trajectory, const double cell_size)
{
    std::list<racer::math::point> points;
    for (const auto &step : trajectory.steps())
    {
        points.push_back(step.step().position());
    }

    plot_points(name, points, "k.", cell_size);
    plot_points(name, points, "r-", cell_size);
}

void plot_grid(const racer::occupancy_grid &occupancy_grid, unsigned char *img)
{
    img = new unsigned char[occupancy_grid.cols() * occupancy_grid.rows() * 4];
    const auto raw_grid = occupancy_grid.raw_data();
    for (std::size_t i = 0; i < raw_grid.size(); ++i)
    {
        std::size_t index = i * 4;
        img[index] = img[index + 1] = img[index + 2] = (unsigned char)(255 - raw_grid[i]); // make obstacles dark
        img[index + 3] = raw_grid[i] < 50 ? 255 : 50;                                      // alpha
    }

    plt::imshow(img, occupancy_grid.rows(), occupancy_grid.cols(), 4);
}

void plot_track_analysis(
    const track_analysis_input &config,
    const std::list<racer::math::circle> &circles,
    const std::list<racer::math::point> &raw_waypoints,
    const std::list<racer::math::point> &waypoints)
{
    // plt::title("Corner Detection");
    plt::grid(true);
    plt::xlim(0, config.occupancy_grid.cols());
    plt::ylim(0, config.occupancy_grid.rows());
    plt::subplot(1, 1, 1);

    unsigned char *grid_img = nullptr;
    plot_grid(config.occupancy_grid, grid_img);
    plot_points("Check points", config.checkpoints, "bx", config.occupancy_grid.cell_size());

    plot_path_of_circles("Path", circles, "k-", config.occupancy_grid.cell_size());
    plot_points("Merged corner points", raw_waypoints, "ro", config.occupancy_grid.cell_size());

    unsigned char *circles_img = nullptr;
    plot_circles(waypoints, config.occupancy_grid, config.min_distance_between_waypoints, circles_img);
    plot_points("Corners", waypoints, "go", config.occupancy_grid.cell_size());
    plot_vehicle_configuration(config.initial_position, config.radius, config.occupancy_grid.cell_size());

    plt::legend();
    plt::show();

    delete[] grid_img;
    delete[] circles_img;
}

void plot_trajectory(
    const track_analysis_input &config,
    const racer::vehicle_model::kinematic_bicycle_model::trajectory &trajectory)
{
    plt::title("Trajectory");
    plt::grid(true);
    plt::xlim(0, config.occupancy_grid.cols());
    plt::ylim(0, config.occupancy_grid.rows());
    plt::subplot(1, 1, 1);

    unsigned char *grid_img = nullptr;
    plot_grid(config.occupancy_grid, grid_img);
    plot_points("Check points", config.checkpoints, "bx", config.occupancy_grid.cell_size());

    plot_vehicle_configuration(config.initial_position, config.radius, config.occupancy_grid.cell_size());
    plot_trajectory("Found trajectory", trajectory, config.occupancy_grid.cell_size());

    plt::legend();
    plt::show();

    delete[] grid_img;
}

#endif
