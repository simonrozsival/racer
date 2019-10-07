#include <iostream>
#include <chrono>
#include <numeric>
#include <cmath>

#include "standalone-experiments/input.h"

#include "racer/math/primitives.h"
#include "racer/track_analysis.h"
#include "racer/sehs/space_exploration.h"

int main(int argc, char *argv[])
{
    std::cout << "Track analysis BENCHMARK standalone experiment - The Racer project" << std::endl
              << std::endl;

    if (argc == 1)
    {
        std::cerr << "At least one argument is requried - path of a circuit definition file." << std::endl;
        return 1;
    }

    std::vector<std::shared_ptr<track_analysis_input>> configs;
    for (int i = 1; i < argc; ++i)
    {
        std::filesystem::path input_file_name = argv[i];
        const std::shared_ptr<track_analysis_input> config = track_analysis_input::load(input_file_name);
        if (!config)
        {
            std::cerr << "Loading configuration from '" << input_file_name << "' failed." << std::endl;
            return 2;
        }

        configs.push_back(config);
    }

    const std::size_t repetitions = 5;

    // Step 1
    std::cout << "1. Space exploration" << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << "circuit name, neighbor circles, number of circles, path length, computation time in ms" << std::endl;

    std::vector<std::list<racer::math::circle>> paths_of_circles;
    for (std::size_t i = 0; i < configs.size(); ++i)
    {
        const auto config = configs[i];
        for (int neighbor_circles = 4; neighbor_circles < 16; ++neighbor_circles)
        {
            std::list<long long> ms;
            std::list<racer::math::circle> path;

            for (std::size_t i = 0; i < repetitions; ++i)
            {
                const auto start = std::chrono::steady_clock::now();

                racer::sehs::space_exploration se(config->occupancy_grid, config->radius, 10 * config->radius, neighbor_circles);
                const auto circles = se.explore_grid(config->initial_position, config->checkpoints);

                const auto end = std::chrono::steady_clock::now();
                const auto t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                ms.push_back(t.count());

                if (i == 0)
                {
                    path = circles;
                }
            }

            if (path.size() > 0)
            {
                if (paths_of_circles.size() == i)
                {
                    paths_of_circles.push_back(path);
                }
                else if (path.size() < paths_of_circles[i].size())
                {
                    paths_of_circles[i] = path;
                }
            }

            double path_length = 0;
            if (path.size() > 0)
            {
                racer::math::circle last = path.front();
                for (const auto &c : path)
                {
                    path_length += c.center.distance(last.center);
                }
            }

            std::cout << config->name << ", " << neighbor_circles << ", " << path.size() << ", " << path_length << ", " << (std::accumulate(ms.begin(), ms.end(), 0) / ms.size()) << std::endl;
        }
    }

    // Step 2
    std::cout << std::endl;
    std::cout << "2. Find pivot points" << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << "circuit name, number of pivot points, computation time in ms" << std::endl;

    std::vector<std::vector<racer::math::point>> pivot_points_outputs;
    for (std::size_t i = 0; i < configs.size(); ++i)
    {
        const auto config = configs[i];

        std::list<long long> ms;
        for (std::size_t j = 0; j < repetitions; ++j)
        {
            const auto start = std::chrono::steady_clock::now();
            racer::track_analysis analysis(config->occupancy_grid, config->min_distance_between_waypoints);
            const auto pivot_points = analysis.find_pivot_points(paths_of_circles[i]);

            const auto end = std::chrono::steady_clock::now();
            const auto t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            ms.push_back(t.count());

            if (j == 0)
            {
                pivot_points_outputs.push_back(pivot_points);
            }
        }

        std::cout << config->name << ", " << pivot_points_outputs[i].size() << ", " << (std::accumulate(ms.begin(), ms.end(), 0) / ms.size()) << std::endl;
    }

    // // this is just for visualization and it does not have to be stopwatched
    // const auto track_analysis_raw_start = std::chrono::steady_clock::now();
    // const auto raw_waypoints = analysis.find_corners(config->radius, config->initial_position, config->checkpoints, false);
    // stop_stopwatch("track analysis without merging", track_analysis_raw_start);

    // // This requires Linux or WSL+Xserver
    // std::cout << "Show interactive plot" << std::endl;
    // plot_track_analysis(*config, circles, raw_waypoints, waypoints);

    std::cout << "Done." << std::endl;
    return 0;
}