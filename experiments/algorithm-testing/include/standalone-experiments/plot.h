#pragma once

#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include "matplotlib-cpp/matplotlibcpp.h"
#pragma GCC diagnostic pop

#include "standalone-experiments/input.h"

#include "racer/circuit.h"
#include "racer/math.h"
#include "racer/track/centerline.h"
#include "racer/trajectory.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/kinematic_model.h"

using namespace racer::vehicle_model;
namespace plt = matplotlibcpp;

std::string speed_color(double speed_percentage)
{
  uint8_t green = speed_percentage * 255.0;
  uint8_t red = 255 - green;

  char buf[8];
  sprintf(buf, "#%02X%02X00", red, green);
  return buf;
}

void plot_vehicle_configuration(
    const racer::vehicle_configuration &configuration, std::string color,
    const double cell_size)
{
  auto direction =
      racer::math::vector(1, 0).rotate(configuration.heading_angle());

  std::vector<double> x, y, u, v;
  x.push_back(configuration.location().x() / cell_size);
  y.push_back(configuration.location().y() / cell_size);
  u.push_back(direction.x() / cell_size);
  v.push_back(direction.y() / cell_size);

  plt::quiver(x, y, u, v, {{"color", color}, {"zorder", "5"}});
}

void plot_path_of_circles(const std::string &name,
                          const std::vector<racer::math::circle> &circles,
                          const std::string &format, const double cell_size)
{
  std::vector<double> circles_x, circles_y;
  for (const auto &circle : circles)
  {
    circles_x.push_back(circle.center().x() / cell_size);
    circles_y.push_back(circle.center().y() / cell_size);
  }
  plt::named_plot(name, circles_x, circles_y, format);
}

void plot_circles(const std::vector<racer::math::point> &points,
                  const std::shared_ptr<racer::occupancy_grid> occupancy_grid,
                  const double radius, unsigned char *img)
{
  img = new unsigned char[occupancy_grid->cols() * occupancy_grid->rows() * 4];
  const auto raw_grid = occupancy_grid->raw_data();
  for (std::size_t row = 0; row < (std::size_t)occupancy_grid->rows(); ++row)
  {
    for (std::size_t col = 0; col < (std::size_t)occupancy_grid->cols();
         ++col)
    {
      racer::math::point c(col * occupancy_grid->cell_size(),
                           row * occupancy_grid->cell_size());

      std::size_t number_of_circles = 0;
      for (const auto &p : points)
      {
        if (c.distance_sq(p) < std::pow(radius, 2))
        {
          number_of_circles++;
        }
      }

      std::size_t index = row * occupancy_grid->cols() * 4 + col * 4;
      img[index] = 0;
      img[index + 1] = 0;
      img[index + 2] = 0;
      img[index + 3] = number_of_circles * 25;
    }
  }

  plt::imshow(img, occupancy_grid->rows(), occupancy_grid->cols(), 4);
}

void plot_points(const std::string &name,
                 const std::vector<racer::math::point> &points,
                 const std::string &format, const double cell_size)
{
  std::vector<double> points_x, points_y;
  for (const auto &point : points)
  {
    points_x.push_back(point.x() / cell_size);
    points_y.push_back(point.y() / cell_size);
  }

  plt::named_plot(name, points_x, points_y, format);
}

void plot_waypoints(const std::shared_ptr<racer::circuit> circuit,
                    unsigned char *img)
{
  img = new unsigned char[circuit->grid->cols() * circuit->grid->rows() * 4];
  const auto raw_grid = circuit->grid->raw_data();
  for (std::size_t row = 0; row < (std::size_t)circuit->grid->rows(); ++row)
  {
    for (std::size_t col = 0; col < (std::size_t)circuit->grid->cols(); ++col)
    {
      racer::math::point c(col * circuit->grid->cell_size(),
                           row * circuit->grid->cell_size());

      std::size_t number_of_waypoints = 0;
      for (std::size_t i = 0; i < circuit->waypoints.size(); ++i)
      {
        if (circuit->passes_waypoint(c, i))
        {
          ++number_of_waypoints;
        }
      }

      std::size_t index = row * circuit->grid->cols() * 4 + col * 4;
      img[index] = 0;
      img[index + 1] = 0;
      img[index + 2] = 255;
      img[index + 3] = number_of_waypoints * 35;
    }
  }

  plt::imshow(img, circuit->grid->rows(), circuit->grid->cols(), 4,
              {{"zorder", "6"}});
}

void plot_trajectory(
    const racer::trajectory<kinematic::state> trajectory,
    const double cell_size,
    const std::shared_ptr<racer::vehicle_model::kinematic::model> vehicle)
{
  std::vector<racer::math::point> states, every_second;
  std::vector<double> speeds;
  for (const auto &step : trajectory.steps())
  {
    states.push_back(step.state().position());
    speeds.push_back(
        vehicle->chassis->motor->normalize_rpm(step.state().motor_rpm()));

    if (step.timestamp() >= 1 &&
        step.timestamp() - std::floor(step.timestamp()) <
            trajectory.time_step())
    {
      every_second.push_back(states.back());
    }
  }

  const auto steps = trajectory.steps();
  auto prev = steps.front().state().position();

  for (std::size_t i{0}; i < steps.size(); ++i)
  {
    const auto pos = steps[i].state().position();

    std::map<std::string, std::string> keywords = {
        {"color", speed_color(speeds[i])},
        {"linewidth",
         std::to_string(vehicle->chassis->radius() * 0.5 / cell_size)}};

    plt::plot(std::vector<double>{prev.x() / cell_size, pos.x() / cell_size},
              std::vector<double>{prev.y() / cell_size, pos.y() / cell_size},
              keywords);

    prev = pos;
  }
}

void plot_grid(const std::shared_ptr<racer::occupancy_grid> occupancy_grid,
               unsigned char *img)
{
  img = new unsigned char[occupancy_grid->cols() * occupancy_grid->rows() * 4];
  const auto raw_grid = occupancy_grid->raw_data();
  for (std::size_t i = 0; i < raw_grid.size(); ++i)
  {
    std::size_t index = i * 4;
    img[index] = img[index + 1] = img[index + 2] =
        (unsigned char)(255 - raw_grid[i]);       // make obstacles dark
    img[index + 3] = raw_grid[i] < 50 ? 255 : 50; // alpha
  }

  plt::imshow(img, occupancy_grid->rows(), occupancy_grid->cols(), 4);
}

void plot_track_analysis(const track_analysis_input &config,
                         const racer::track::centerline &centerline,
                         const std::vector<racer::math::point> &raw_waypoints,
                         const std::vector<racer::math::point> &waypoints,
                         const double waypoint_radius)
{
  plt::title(config.name);
  plt::grid(true);
  plt::xlim(0, config.occupancy_grid->cols());
  plt::ylim(0, config.occupancy_grid->rows());
  plt::subplot(1, 1, 1);

  unsigned char *grid_img = nullptr;
  plot_grid(config.occupancy_grid, grid_img);
  plot_points("Centerline", centerline.points(), "k.",
              config.occupancy_grid->cell_size());
  plot_points("Skipped candidates", raw_waypoints, "ro",
              config.occupancy_grid->cell_size());

  unsigned char *circles_img = nullptr;
  unsigned char *waypoints_img = nullptr;
  plot_circles(waypoints, config.occupancy_grid, waypoint_radius,
               circles_img);

  auto circuit = std::make_shared<racer::circuit>(waypoints, waypoint_radius, config.occupancy_grid);
  plot_waypoints(circuit, waypoints_img);
  plot_points("Corners", waypoints, "bo", config.occupancy_grid->cell_size());
  plot_vehicle_configuration(config.initial_position, "green",
                             config.occupancy_grid->cell_size());

  plt::legend();
  // std::stringstream trajectory_file_name;
  // trajectory_file_name << config.name << ".pdf";
  // plt::save(trajectory_file_name.str());
  plt::show();

  delete[] grid_img;
  delete[] circles_img;
  delete[] waypoints_img;
}

void plot_trajectory(
    const track_analysis_input &config,
    const racer::vehicle_configuration initial_configuration,
    const racer::trajectory<kinematic::state> &trajectory,
    const std::shared_ptr<racer::vehicle_model::kinematic::model> vehicle,
    const std::shared_ptr<racer::circuit> circuit, const std::string name)
{
  std::vector<racer::math::point> rpm_points, steering_angle_points,
      speed_points, throttle, steering;

  double min_time = HUGE_VAL;
  double max_time = 0;
  double max_speed = 0;
  double avg_speed = 0;

  for (const auto &step : trajectory.steps())
  {
    const double speed = vehicle->calculate_speed_with_no_slip_assumption(
        step.state().motor_rpm());
    rpm_points.emplace_back(step.timestamp(),
                            step.state().motor_rpm() /
                                vehicle->chassis->motor->max_rpm());
    steering_angle_points.emplace_back(step.timestamp(),
                                       double(step.state().steering_angle()));
    speed_points.emplace_back(step.timestamp(),
                              speed);
    throttle.emplace_back(step.timestamp(), step.previous_action().throttle());
    steering.emplace_back(step.timestamp(),
                          step.previous_action().target_steering_angle());

    avg_speed = (avg_speed * (speed_points.size() - 1) + speed) / double(speed_points.size());
    max_speed = std::max(speed, max_speed);
    min_time = std::min(step.timestamp(), min_time);
    max_time = std::max(step.timestamp(), max_time);
  }

  plt::title(config.name);

  plt::subplot(1, 1, 1);
  plt::grid(true);
  plt::xlim(0, config.occupancy_grid->cols());
  plt::ylim(0, config.occupancy_grid->rows());

  unsigned char *grid_img = nullptr;
  unsigned char *circles_img = nullptr;
  unsigned char *waypoints_img = nullptr;
  plot_grid(config.occupancy_grid, grid_img);
  plot_circles(circuit->waypoints, circuit->grid, circuit->waypoint_radius,
               circles_img);
  plot_waypoints(circuit, waypoints_img);

  plot_trajectory(trajectory, config.occupancy_grid->cell_size(), vehicle);
  plot_vehicle_configuration(trajectory.steps().back().state().configuration(),
                             "red", config.occupancy_grid->cell_size());
  plot_vehicle_configuration(initial_configuration, "green",
                             config.occupancy_grid->cell_size());

  plt::legend();
  // std::stringstream trajectory_file_name;
  // trajectory_file_name << name << "_trajectory.pdf";
  // plt::save(trajectory_file_name.str());
  plt::show();

  // actuators state profile
  plt::subplot(3, 1, 1);
  plt::ylim(-1.1, 1.1);

  plot_points("Throttle level", throttle, "r-", 1.0);
  plot_points("Steering input", steering, "b-", 1.0);
  plt::legend();

  plt::subplot(3, 1, 2);
  plot_points("Motor RPM (normalized)", rpm_points, "r-", 1.0);
  plot_points("Steering angle [rad]", steering_angle_points, "b-", 1.0);
  plt::legend();

  // speed profile
  plt::subplot(3, 1, 3);
  plt::ylabel("speed [m/s]");
  plt::xlabel("time [s]");

  // hack: draw the max speed line
  plt::ylim(0.1, 1.1 * max_speed);
  std::vector<racer::math::point> max_speed_points{
      {min_time, max_speed}, {max_time, max_speed}};
  std::vector<racer::math::point> avg_speed_points{
      {min_time, avg_speed}, {max_time, avg_speed}};
  std::stringstream max_speed_label;
  max_speed_label << std::setprecision(2);
  max_speed_label << max_speed << " m/s";
  std::stringstream avg_speed_label;
  avg_speed_label << std::setprecision(2);
  avg_speed_label << avg_speed << " m/s";
  plot_points(max_speed_label.str(), max_speed_points, "k--", 1.0);
  plot_points(avg_speed_label.str(), avg_speed_points, "k--", 1.0);

  // colored speed profile
  racer::math::point prev = speed_points.front();
  for (auto speed_pt : speed_points)
  {
    std::map<std::string, std::string> keywords = {{"color", speed_color(speed_pt.y() / vehicle->top_speed())}};
    plt::plot(std::vector<double>{prev.x(), speed_pt.x()},
              std::vector<double>{prev.y(), speed_pt.y()},
              keywords);

    prev = speed_pt;
  }

  plt::legend();
  // std::stringstream actuators_file_name;
  // actuators_file_name << name << "_actuators.pdf";
  // plt::save(actuators_file_name.str());
  plt::show();

  delete[] grid_img;
  delete[] circles_img;
  delete[] waypoints_img;
}
