#include <iostream>
#include <chrono>
#include <numeric>
#include <cmath>
#include <vector>

#include "standalone-experiments/input.h"
#include "standalone-experiments/plot.h"

#include "racer/math.h"
#include "racer/action.h"
#include "racer/trajectory.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/track_analysis.h"
#include "racer/astar/sehs.h"
#include "racer/sehs/space_exploration.h"

using namespace racer::vehicle_model;
using namespace racer::astar::sehs;

using search_problem = racer::astar::discretized_search_problem<
    racer::astar::sehs::kinematic::discrete_state,
    racer::vehicle_model::kinematic::state>;

int main(int argc, char *argv[])
{
    std::cout << "Trajectory Planning BENCHMARK standalone experiment - The Racer project" << std::endl
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
    std::cout << "1. SEHS" << std::endl;
    std::cout << "--------------------" << std::endl;

    std::map<std::string, std::chrono::milliseconds> space_exploration_benchmark_results;

    const double time_step_s = 1.0 / 25.0;
    racer::vehicle_model::vehicle vehicle(
        0.155,               // cog_offset
        0.31,                // wheelbase
        0.35,                // safe width
        0.55,                // safe length
        2.0 / 3.0 * M_PI,    // steering speed (rad/s)
        24.0 / 180.0 * M_PI, // max steering angle (rad)
        200.0,               // speed (ms^-1)
        -3.0,                // reversing speed (ms^-1)
        3.0                  // acceleration (ms^-2)
    );
    auto transition_model = std::make_shared<racer::vehicle_model::kinematic::model>(vehicle, time_step_s);

    std::cout << "circuit name, from waypoint, to waypoint, found solution, open nodes, closed nodes, time to finish, computation time in ms" << std::endl;

    for (std::size_t i = 0; i < configs.size(); ++i)
    {
        const auto config = configs[i];
        const auto inflated_grid = config->occupancy_grid.inflate(vehicle.radius() / config->occupancy_grid.cell_size());

        const auto start_disc = std::chrono::steady_clock::now();

        racer::sehs::space_exploration exploration(config->occupancy_grid, vehicle.radius(), 2 * vehicle.radius(), config->neighbor_circles);
        const auto circle_path = exploration.explore_grid(config->initial_position, config->checkpoints);

        const auto end_disc = std::chrono::steady_clock::now();
        const auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(end_disc - start_disc);
        space_exploration_benchmark_results[config->name] = dt;

        auto discretization = std::make_shared<racer::astar::sehs::kinematic::discretization>(
            circle_path,
            M_PI / 12.0,
            0.25);

        const racer::vehicle_model::kinematic::state state(
            config->initial_position, 0, 0);

        const double max_angle = M_PI * (4.0 / 5.0);
        racer::track_analysis analysis(config->occupancy_grid, config->min_distance_between_waypoints);
        const auto waypoints = analysis.find_corners(analysis.find_pivot_points(circle_path), max_angle);

        for (std::size_t i = 0; i < repetitions; ++i)
        {
            const auto start = std::chrono::steady_clock::now();

            auto discrete_initial_state = discretization->discretize(state);

            racer::circuit circuit(
                config->initial_position,
                waypoints,
                5 * vehicle.radius(),
                inflated_grid);

            auto initial_state{state};
            auto problem = std::make_unique<search_problem>(
                initial_state,
                discrete_initial_state,
                time_step_s,
                vehicle,
                racer::action::create_actions(5, 9),
                discretization,
                transition_model,
                circuit);

            const auto result = racer::astar::search<
                racer::astar::sehs::kinematic::discrete_state,
                racer::vehicle_model::kinematic::state>(std::move(problem), 1000000000);

            const auto end = std::chrono::steady_clock::now();
            const auto t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            std::cout << config->name << ", " << (result.was_successful() ? "yes" : "no") << "," << result.number_of_opened_nodes << ", " << result.number_of_expanded_nodes << ", " << result.final_cost << ", " << t.count() << std::endl;

            if (i == repetitions - 1 && result.was_successful())
            {
                plot_trajectory(*config, result.found_trajectory);
            }
        }
    }

    std::cout << "space exploration benchmark" << std::endl;
    std::cout << "circuit name, space exploration time in ms" << std::endl;

    for (auto res : space_exploration_benchmark_results)
    {
        std::cout << res.first << ", " << res.second.count() << std::endl;
    }

    // // Hybrid A*
    // std::cout << "1. Hybrid A*" << std::endl;
    // std::cout << "--------------------" << std::endl;

    std::cout << "Done." << std::endl;
    return 0;
}